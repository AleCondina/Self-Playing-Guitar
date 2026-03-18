# Calibration V2: Adaptive Open-Frequency PI Lock + Bug Fixes

## Context

The calibration system uses hardcoded `static const` frequencies (`s_open_freq`, `cal_theoretical_freq`) based on standard tuning. If the motor's position 0 doesn't perfectly correspond to the unfretted string position (e.g., motor slightly pressing the string), all fret targets are offset. This plan adds a PI-based "find the true open" step before fret calibration begins.

Additionally, two bugs were supposedly fixed in this session (before plan mode) we need to test to see if these were truely fixed:
- **HOME_PI_LOCK timeout never fires** — `s_pi_lock_consec == 0u` condition prevented timeout when PI had seen any in-band reading. Changed to `< CAL_LOCK_CONSEC_HOPS`. (DONE)
- **Debug_UART_ResetRx() race condition** — async `HAL_UART_AbortReceive_IT` left RxState as BUSY. Rewrote to force `RxState = HAL_UART_STATE_READY` with IRQ disabled. (DONE)

---

## Current Calibration Flow (before this change)

```
STR_INIT → WAIT_PLUCK → WAIT_HOME(pos 0) → SOL_BURST(fret 1) → PI_LOCK(fret 1) → RECORD
  → SOL_BURST(fret 2) → PI_LOCK(fret 2) → RECORD → ... → fret 7 → BUILD → HOME phases → DONE
```

**Problem**: The system assumes position 0 = perfect unfretted position. All fret targets use hardcoded frequencies. If the motor is even slightly off, every fret measurement is shifted.

## Proposed V2 Flow

```
STR_INIT → WAIT_PLUCK → WAIT_HOME(pos 0)
  → OPEN_SOL_BURST → OPEN_BURST_REENABLE → OPEN_PI_LOCK → OPEN_DONE
  → SOL_BURST(fret 1) → PI_LOCK(fret 1) → RECORD
  → SOL_BURST(fret 2) → PI_LOCK(fret 2) → RECORD → ... → fret 7
  → BUILD → HOME_SOL_BURST → HOME_PI_LOCK → HOME_DONE → DONE
```

**New**: After homing to position 0, the system excites the string and PI-locks onto the nominal open frequency (e.g., 82.4 Hz for E2). The motor position at lock becomes the "true zero" offset applied to all subsequent fret targets.

---

## New Feature: PI-based open-frequency lock before fret calibration

**File:** `CM7/App/Src/calibration_controller.c`

### New phases (insert between WAIT_HOME and SOL_BURST in CalPhase_t enum)

- `CAL_PHASE_OPEN_SOL_BURST` — Fire solenoid at detected/nominal open freq, inhibit FFT
- `CAL_PHASE_OPEN_BURST_REENABLE` — Re-enable FFT, reset PI
- `CAL_PHASE_OPEN_PI_LOCK` — PI targets `s_open_freq[s_str]`, adjusts motor position
- `CAL_PHASE_OPEN_DONE` — Record position offset, proceed to fret 1

### New state variable

```c
static int32_t s_open_position_offset = 0;  /* Motor position where open freq was found */
```

### Detailed phase logic

#### 1. WAIT_PLUCK modification

Change `s_after_home` from `CAL_PHASE_SOL_BURST` to `CAL_PHASE_OPEN_SOL_BURST` at both call sites (lines ~711 and ~722). This routes the flow through the open-frequency measurement before starting fret calibration.

#### 2. OPEN_SOL_BURST

```
- Fire solenoid at s_open_detected_freq (from pluck detection) or s_open_freq[s_str] (nominal)
- Inhibit FFT (fft_sensing_enabled = 0)
- Reset PI state (integral, cmd_steps, hop_cnt, lock_consec, no_sig_cnt, last_valid_freq, retry_cnt)
- WaitThen(CAL_BURST_WAIT_HOPS, CAL_PHASE_OPEN_BURST_REENABLE)
```

#### 3. OPEN_BURST_REENABLE

```
- fft_sensing_enabled = 1
- Transition to OPEN_PI_LOCK
```

#### 4. OPEN_PI_LOCK

Same PI logic as existing fret PI_LOCK but:
- **Target**: `s_open_freq[s_str]` (nominal open frequency)
- **Search band**: ±15% of target
- **Motor clamp**: `TUNING_STEPS_MIN_DEFAULT` (-50) to ~200 steps (open position should be near 0)
- **Lock criteria**: `|error| <= TUNING_LOCK_BAND_IN_HZ` for `CAL_LOCK_CONSEC_HOPS` consecutive hops
- **Re-excite on signal loss**: up to `CAL_MAX_RETRIES` attempts
- **Timeout**: `CAL_PI_LOCK_TIMEOUT` hops → accept current position (offset may be 0 if PI couldn't converge)

#### 5. OPEN_DONE

```c
s_open_position_offset = g_shared_memory.current_positions[s_str];
printf("[CAL] str%u: open freq locked at %.2f Hz, offset=%ld steps\r\n",
       (unsigned)s_str, (double)s_last_valid_freq,
       (long)s_open_position_offset);
s_phase = CAL_PHASE_SOL_BURST;  /* Proceed to fret 1 */
```

### Modifications to existing code

#### SOL_BURST polynomial target (line ~801)

After computing the polynomial estimate, add the offset so fret targets are relative to the measured open position:

```c
float _poly = _a + _b * _tf + _c * _tf * _tf;
if (_poly < 0.0f) _poly = 0.0f;
s_steps_expected = (int32_t)CalClampf(_poly, 0.0f,
                                      (float)s_cal_steps_max[s_str])
                   + s_open_position_offset;
```

#### HOME_SOL_BURST motor target (line ~1114)

Change motor home target from position 0 to the measured open position:

```c
InterCore_SendMotorGoto(s_str, s_open_position_offset);  /* was: 0 */
```

#### HOME_PI_LOCK PI center (line ~1180)

Change PI command center from 0 to the measured open position:

```c
int32_t cmd = (int32_t)CalClampf(
    (float)s_open_position_offset + trim,  /* was: (float)0 + trim */
```

#### STR_INIT reset (line ~648 area)

Add `s_open_position_offset = 0` alongside other state resets.

---

## Edge case: PI can't find nominal open frequency

If the string is genuinely out of tune (e.g., tuned to 90 Hz but nominal is 82.4 Hz), the motor **cannot** reach 82.4 Hz — it can only press the string to increase frequency, not decrease it. The PI will timeout after `CAL_PI_LOCK_TIMEOUT` hops. In `OPEN_DONE`, `s_open_position_offset` will be whatever position the motor ended at (likely ~0). This is acceptable — calibration proceeds with the same behavior as the current system.

The motor CAN find the nominal frequency if:
- The string is tuned slightly sharp (e.g., 85 Hz vs 82.4 Hz nominal) — motor backs off to find 82.4 Hz
- Position 0 is slightly past the true unfretted position — motor backs off to the true open

---

## Physical intuition

```
Motor position:  -50 ......... 0 ......... 200 ......... 5000+
String state:    [fully open]  [nominal]   [slightly pressed]  [fret 7]
Frequency:       [lowest]      [~nominal]  [slightly higher]   [highest]

The OPEN_PI_LOCK explores the -50 to ~200 range to find exactly where
the string produces the nominal open frequency. Typically the offset
is small (0-50 steps).
```

---

## Files modified

| File | Change |
|------|--------|
| `CM7/App/Src/calibration_controller.c` | Add 4 OPEN phases + `s_open_position_offset`, modify WAIT_PLUCK routing, modify SOL_BURST polynomial offset, modify HOME targets to use offset, reset in STR_INIT |

Already done (no further changes needed):
| `CM7/App/Src/debug_uart.c` | `Debug_UART_ResetRx()` rewrite |
| `CM7/App/Src/calibration_controller.c` | PI_LOCK timeout condition fix (both fret and home) |

No header file changes needed — all new phases/variables are file-static.

---

## Verification

1. Flash and run `calibrate 1` — watch for `[CAL] str1: open freq locked at ~110 Hz, offset=X steps` BEFORE fret 1 begins
2. Verify fret measurements still lock correctly (polynomial targets now include the offset)
3. After all frets complete, HOME_PI_LOCK should return motor to `s_open_position_offset` (not 0)
4. After calibration completes, type `status` — CLI should respond immediately (UART fix)
5. Run with a well-tuned string — offset should be near 0
6. Edge case: if string is significantly out of tune, OPEN_PI_LOCK should timeout gracefully and calibration proceeds with offset ≈ 0

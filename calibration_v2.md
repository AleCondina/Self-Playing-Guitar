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

---

## V2.1: Half-Semitone Spacing + −1 Fret Anchor (2026-03-25)

### Problem

The polynomial fit `steps = a + b·f + c·f²` was fitted using only 7 measured points (frets 1–7 from `s_buf`). The open-string data point `(0 steps, nominal open freq)` was **not** included in the polynomial — it was only hardcoded as LUT breakpoint 0 in `Cal_BuildTable`. This meant:
- The polynomial wasn't constrained to pass through the open-string region
- The nominal frequency (e.g., 82.41 Hz for E2) was assumed rather than measured
- Only 7 data points gave a coarse quadratic fit across the fret range

### Changes

#### Updated calibration flow

```
STR_INIT → WAIT_PLUCK → WAIT_HOME(pos 0)
  → OPEN_SOL_BURST → OPEN_BURST_REENABLE → OPEN_PI_LOCK → OPEN_DONE
  → NEG_SOL_BURST → NEG_BURST_REENABLE → NEG_PI_LOCK → NEG_RECORD   ← NEW
  → SOL_BURST(t1, +0.5st) → PI_LOCK → RECORD                        ← half-semitone
  → SOL_BURST(t2, +1.0st) → PI_LOCK → RECORD
  → ... → SOL_BURST(t14, +7.0st) → PI_LOCK → RECORD
  → BUILD → HOME_SOL_BURST → HOME_PI_LOCK → HOME_DONE → DONE
```

#### 1. Constants changed

| Constant | Old | New | Notes |
|----------|-----|-----|-------|
| `CAL_NUM_FRETS` | 8 | **removed** | Replaced by `CAL_NUM_TARGETS` |
| `CAL_NUM_TARGETS` | — | 14 | Half-semitone targets: idx 1 = +0.5st, idx 14 = +7.0st |
| `CAL_MAX_SAMPLES` | 8 | 18 | 1 open + 1 neg + 14 targets + headroom |

#### 2. Replaced static frequency table with computed function

Removed `cal_theoretical_freq[NUM_STRINGS][CAL_NUM_FRETS]` (hardcoded fret frequencies). Replaced with:

```c
static inline float Cal_TargetFreq(uint8_t str, uint8_t idx) {
    float semitones = (float)idx * 0.5f;
    return s_open_freq[str] * powf(2.0f, semitones / 12.0f);
}
```

This computes equal-temperament frequencies on the fly at half-semitone resolution.

#### 3. Measured open-string point added to `s_buf`

In `CAL_PHASE_OPEN_DONE`, after motor re-zero:
```c
s_buf[0].steps = 0.0f;
s_buf[0].freq  = s_last_valid_freq;   // measured, NOT nominal
s_buf_n = 1u;
```

The polynomial is now fitted through the **actual measured** open-string frequency, not the textbook value. This eliminates the assumption that position 0 = perfect nominal frequency.

#### 4. −1 fret measurement (4 new phases)

New phases added to `CalPhase_t`:
- `CAL_PHASE_NEG_SOL_BURST` — Move motor to negative position, fire solenoid at measured open freq
- `CAL_PHASE_NEG_BURST_REENABLE` — Re-enable FFT
- `CAL_PHASE_NEG_PI_LOCK` — PI locks to `open_freq / 2^(1/12)` (1 semitone below open)
- `CAL_PHASE_NEG_RECORD` — Record `(actual_steps, measured_freq)` into `s_buf[1]`

Motor target for −1 fret: estimated from polynomial, clamped to `[TUNING_STEPS_MIN_DEFAULT (-800), 0]`, fallback −300 steps. This gives the polynomial a data point below open, properly anchoring the low end.

#### 5. Half-semitone fret loop (14 targets)

`s_fret` now ranges 1–14 (was 1–7). Each increment is 0.5 semitones:
- `s_fret=1` → +0.5 semitones (halfway to fret 1)
- `s_fret=2` → +1.0 semitone (fret 1)
- `s_fret=3` → +1.5 semitones (halfway to fret 2)
- ...
- `s_fret=14` → +7.0 semitones (fret 7)

**Motor limit check**: After each RECORD, the polynomial estimates the step position for the next target. If it exceeds `s_cal_steps_max[str]`, calibration skips directly to BUILD.

#### 6. `Cal_BuildTable` uses measured open freq

LUT breakpoint 0 now uses `s_buf[0].freq` (measured) instead of `s_open_freq[str]` (nominal 82.41 Hz). Both the LUT and polynomial are fitted entirely from real measured data — no assumed-perfect values anywhere.

### Expected data points per string

```
Index  Semitones  Type          Example (E2)
  0      0.0      Open string   ~82.xx Hz (measured), 0 steps
  1     −1.0      Neg fret      ~77.xx Hz (PI locked), ~−300 steps
  2     +0.5      Half fret 1   ~84.8 Hz
  3     +1.0      Fret 1        ~87.3 Hz
  4     +1.5      Half fret 2   ~89.9 Hz
  ...
 15     +7.0      Fret 7        ~123.5 Hz
```

Total: up to **16 data points** per string for the polynomial fit (was 7).

### Files modified

| File | Change |
|------|--------|
| `CM7/App/Src/calibration_controller.c` | Replace `CAL_NUM_FRETS`→`CAL_NUM_TARGETS`, `CAL_MAX_SAMPLES` 8→18, replace `cal_theoretical_freq` table with `Cal_TargetFreq()`, add measured open point to `s_buf`, add 4 NEG phases, half-semitone loop termination with motor limit check, `Cal_BuildTable` uses measured open freq |
| `CM7/App/Inc/calibration_controller.h` | Updated header comment |

### Verification

1. Build CM7 — no compile errors
2. Run `cal 1` on E2 string over serial
3. UART output should show: open measured point, −1 fret point, then 14 half-semitone targets
4. Check `[CAL-POLY]` coefficients — polynomial should evaluate to ~0 steps at open freq
5. Motor limit check should trigger before exceeding `s_cal_steps_max` for any string
6. Calibration time per string ~2× longer (16 measurements vs 7)

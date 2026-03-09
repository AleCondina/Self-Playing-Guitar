# Tuning Controller — Change Documentation

> **Branch:** `Control_system`
> **Date:** 2026-03-09
> **Scope:** Closed-loop autonomous guitar string tuning system

---

## 1. Overview

A closed-loop tuning controller was added to the CM7 firmware.  It runs at every audio processing hop (~170 ms), reads per-string pitch estimates from the Goertzel estimator, computes absolute motor-step targets via a feed-forward lookup table (FF-LUT) plus a discrete PI trim, and dispatches motor position commands and solenoid actuation commands to CM4 through the existing inter-core shared-memory channel.

No existing low-level drivers (motor, solenoid, ADC, DMA, FFT) were rewritten.  All changes are either new files or minimal additions to existing files.

---

## 2. Files Added

### `CM7/App/Inc/tuning_controller.h`

Public API and all compile-time parameters for the tuning controller.

**Key contents**

| Symbol | Value | Notes |
|--------|-------|-------|
| `TUNING_SAMPLE_TIME_S` | `(FFT_SIZE/2) / AUDIO_SAMPLE_RATE_HZ` ≈ 0.171 s | Actual hop period |
| `TUNING_DELAY_FRAMES` | `3` | Hops to hold PI off after a note change |
| `TUNING_LOCK_BAND_IN_HZ` | `1.0` | Enter lock threshold |
| `TUNING_LOCK_BAND_OUT_HZ` | `2.5` | Exit lock threshold (hysteresis) |
| `TUNING_MIN_MOVE_STEPS` | `2.0` | Backlash dead-band (microsteps) |
| `TUNING_OUT_LIMIT_STEPS` | `400.0` | PI trim output clamp |
| `TUNING_MAX_STEP_RATE_SPS` | `5000.0` | Slew limit (microsteps/s) |
| `TUNING_KP` | `3.0` | Proportional gain (steps/Hz) — **TUNE FOR HARDWARE** |
| `TUNING_KI` | `1.5` | Integral gain (steps/Hz/s) — **TUNE FOR HARDWARE** |
| `TUNING_SNR_THRESHOLD` | `500.0` | Goertzel magnitude gate — **TUNE FOR HARDWARE** |
| `TUNING_STALL_TIMEOUT_HOPS` | `20` | Hops before stall fault is flagged |
| `TUNING_FF_NUM_POINTS` | `4` | Breakpoints per string in the FF-LUT |
| `TUNING_STEPS_MAX_DEFAULT` | `2000` | Mechanical upper limit (microsteps) |
| `TUNING_STEPS_MIN_DEFAULT` | `-50` | Mechanical lower limit (microsteps) |

**Per-string run-time state struct** — `TuningStringState_t`

```c
typedef struct {
    float    last_target_freq_hz;   // target at previous hop
    float    last_valid_freq_hz;    // last frequency accepted above SNR threshold
    float    ff_steps_abs;          // feed-forward absolute position (microsteps)
    float    trim_steps;            // PI output
    float    integral_state;        // PI integrator accumulator
    int32_t  cmd_steps_abs;         // final integer command sent to motor
    int32_t  prev_cmd_steps_abs;    // command at previous hop (for change detection)
    int32_t  current_abs_steps;     // motor position read back from CM4
    uint8_t  lock_state;            // 1 = in-tune and stable
    uint8_t  prev_lock_state;       // for solenoid edge detection
    uint8_t  pi_enable;             // 1 = PI accumulating this hop
    uint16_t delay_hold_counter;    // hops remaining in post-note hold
    uint8_t  fault_flags;           // TUNING_FAULT_STALL | TUNING_FAULT_SATURATED
    uint32_t stall_counter;
    uint32_t hop_count;
} TuningStringState_t;
```

**Public API**

```c
void TuningCtrl_Init(void);
void TuningCtrl_RunHop(void);                          // main entry point
void TuningCtrl_Update(uint8_t str, float target,
                       float measured, float confidence);
int32_t TuningCtrl_GetCmdSteps(uint8_t string_id);
uint8_t TuningCtrl_GetLockState(uint8_t string_id);
const TuningStringState_t *TuningCtrl_GetStringState(uint8_t string_id);
void TuningCtrl_ResetString(uint8_t string_id);
void TuningCtrl_ResetAll(void);
```

---

### `CM7/App/Src/tuning_controller.c`

Implementation of the tuning controller.  Runs entirely on CM7; no direct HAL calls except through the inter-core API.

#### Feed-forward LUT (`FF_Interpolate`)

Two 6×4 `const float` arrays define the mapping from target frequency to absolute motor position in microsteps:

```
tuning_ff_freq_bp[string][breakpoint]  — frequency (Hz) at each breakpoint
tuning_ff_steps_bp[string][breakpoint] — absolute microsteps at each breakpoint
```

`FF_Interpolate(str, freq)` performs piecewise-linear interpolation; clips (does not extrapolate) outside the table range.

> **These are placeholder values.** They must be replaced with measured data during hardware commissioning (home each motor, step slowly, record Goertzel frequency vs. position, fill in the table).

#### Control hop — `TuningCtrl_Update()`

Executes the following steps in order every hop:

| Step | Action |
|------|--------|
| 1 | **Note-change detection** — if `|target - last_target| > 0.05 Hz`: reset integrator, set `delay_hold_counter = TUNING_DELAY_FRAMES`, update FF immediately |
| 2 | **Delay hold countdown** — if `delay_hold_counter > 0`: decrement, keep `pi_enable = 0` |
| 3 | **SNR gate** — if `confidence < TUNING_SNR_THRESHOLD`: freeze integral, hold last valid frequency |
| 4 | **Frequency error** — `e = target_freq - use_freq` |
| 5 | **Lock-band hysteresis** — enter lock if `|e| ≤ 1.0 Hz`; exit lock if `|e| ≥ 2.5 Hz` |
| 6 | **Discrete PI with anti-windup** — `integral += Ki × e × dt`; `trim = Kp×e + integral`; clamp output; back-calculate integrator when clamped; apply `min_move` dead-band |
| 7 | **Raw command** — `raw_cmd = ff_steps_abs + trim_steps` |
| 8 | **Slew limit** — `delta = clamp(raw_cmd - prev_cmd, ±max_step_rate × dt)` |
| 9 | **Mechanical bounds** — clamp to `[STEPS_MIN, STEPS_MAX]` |
| 10 | **Store** — save `cmd_steps_abs`; mirror `current_abs_steps` from `g_shared_memory.current_positions` |
| 11 | **Stall detection** — if large trim with no frequency movement for `STALL_TIMEOUT_HOPS` hops: set `TUNING_FAULT_STALL`, freeze command at current position |

#### High-level runner — `TuningCtrl_RunHop()`

Called once per audio hop from `app_cm7.c`.  Orchestrates:

1. Read active note from `SongSched_GetStringId()` / `SongSched_GetTargetFreq()`
2. Read Goertzel estimates: `Goertzel_GetExactFreq(str)`, `Goertzel_GetMagnitude(str)`
3. Call `TuningCtrl_Update()`
4. If `cmd_steps_abs` changed: call `InterCore_SendMotorGoto(str, cmd_steps_abs)`
5. Call `ApplyActuation()` for solenoid edge-detection

#### Solenoid actuation — `ApplyActuation()`

Detects lock-state edges:

- **Rising edge (unlocked → locked):** calls `InterCore_TriggerSolenoidHz(str, freq_hz, 50%, INFINITE, 0, 0)` — solenoid drives at the exact target frequency via `Solenoid_SetFrequencyHz()` on CM4.
- **Falling edge (locked → unlocked):** sets `CMD_STOP_SOLENOID` in shared memory and notifies CM4.

#### Debug CSV logging

Every 5th hop the following line is printed over UART (prefix `[TC]`):

```
timestamp_ms, string_id, target_hz, measured_hz, error_hz,
ff_steps, trim_steps, cmd_steps, cur_steps,
lock, pi_en, delay_ctr, confidence, faults
```

The CSV header is printed once at startup by `app_cm7.c`.  Capture the UART output and open in any spreadsheet or Python/MATLAB for offline gain tuning and system identification.

---

### `CM7/App/Inc/song_scheduler.h`

API for the note schedule.

```c
void     SongSched_Init(void);
void     SongSched_Tick(uint32_t now_ms);   // call every hop
uint8_t  SongSched_GetStringId(void);
float    SongSched_GetTargetFreq(void);
uint8_t  SongSched_IsActive(void);
void     SongSched_Restart(void);
uint32_t SongSched_GetNoteCount(void);
uint32_t SongSched_GetNoteIndex(void);
```

Each note is described by a `SongNote_t`:

```c
typedef struct {
    uint8_t  string_id;       // 0=E2, 1=A2, 2=D3, 3=G3, 4=B3, 5=E4
    float    target_freq_hz;
    uint32_t duration_ms;
} SongNote_t;
```

---

### `CM7/App/Src/song_scheduler.c`

Implements a hardcoded 20-note demo song on strings E2, A2, D3 (pentatonic phrase).  The `song_notes[]` array at the top of the file is the only thing that needs to change to load a different song.  `SongSched_Tick()` advances the note index based on elapsed milliseconds; the song halts (does not loop) when the last note expires.

---

## 3. Files Modified

### `CM7/App/Src/app_cm7.c`

Added includes for the two new modules:

```c
#include "tuning_controller.h"
#include "song_scheduler.h"
```

Added initialization calls (in `App_CM7_Main`, before the main loop):

```c
Goertzel_Init();          // was not previously called
TuningCtrl_Init();
SongSched_Init();
// print CSV log header
```

Replaced the audio-processing section in `STATE_RUNNING` with the full control-hop sequence:

```c
if (Audio_HasNewSamples()) {
    uint16_t *audio_buf = Audio_GetBuffer();
    FFT_ProcessBlock(audio_buf);          // polyphonic diagnostics (unchanged)
    Goertzel_ProcessBlock(audio_buf);     // per-string freq + magnitude (NEW)
    Audio_MarkProcessed();
    SongSched_Tick(HAL_GetTick());        // NEW
    TuningCtrl_RunHop();                  // NEW
    // print FFT results (unchanged)
}
```

Added a 5-second heartbeat print showing current song note index and CM4 loop rate.

---

### `CM7/CMakeLists.txt`

Added the two new source files to `target_sources` so CMake includes them in the build and regenerates `compile_commands.json` for IDE indexing:

```cmake
${CMAKE_CURRENT_SOURCE_DIR}/App/Src/tuning_controller.c
${CMAKE_CURRENT_SOURCE_DIR}/App/Src/song_scheduler.c
```

The `CM7/App/Inc/` include path already existed; no new include path was needed.

---

### `Shared/Inc/shared_memory.h`

**New command bit:**

```c
#define CMD_TRIGGER_SOLENOID_HZ  (1U << 13)
// Trigger solenoid at exact Hz via Solenoid_SetFrequencyHz() on CM4.
// Previously all solenoid frequency commands went through a string/fret LUT.
```

**New shared memory field** (added to `SharedMemory_t`, after `solenoid_padding`):

```c
volatile uint16_t solenoid_freq_hz;
// Target drive frequency in Hz for CMD_TRIGGER_SOLENOID_HZ.
// CM4 computes ARR = (640000 / solenoid_freq_hz) - 1 and applies it directly.
```

---

### `CM7/App/Inc/inter_core_cm7.h`

Added declaration:

```c
void InterCore_TriggerSolenoidHz(uint8_t  solenoid_id,
                                  uint16_t freq_hz,
                                  uint8_t  duty_percent,
                                  uint8_t  mode,
                                  uint16_t time_ms,
                                  uint16_t burst_cycles);
```

---

### `CM7/App/Src/inter_core_cm7.c`

Added implementation of `InterCore_TriggerSolenoidHz()`.  Writes `solenoid_id`, `solenoid_freq_hz`, `solenoid_duty_cycle`, `solenoid_mode`, `solenoid_activation_time_ms`, `solenoid_burst_cycles` into shared memory, sets `CMD_TRIGGER_SOLENOID_HZ`, calls `__DSB()` and `InterCore_NotifyCM4()`.

---

### `CM4/App/Inc/solenoid_control.h`

Added declaration:

```c
int32_t Solenoid_SetFrequencyHz(uint8_t solenoid_id, uint32_t freq_hz);
// Sets ARR = (640000 / freq_hz) - 1 and updates CCR for current duty cycle.
// Valid range: 40–800 Hz. Does NOT start PWM.
```

---

### `CM4/App/Src/solenoid_control.c`

Added implementation:

```c
int32_t Solenoid_SetFrequencyHz(uint8_t solenoid_id, uint32_t freq_hz)
{
    // guard: solenoid_id < 6, 40 <= freq_hz <= 800
    uint16_t new_arr = (uint16_t)((640000UL / freq_hz) - 1UL);
    uint16_t new_ccr = (uint16_t)((new_arr * state->duty_cycle) / 100u);
    __HAL_TIM_SET_AUTORELOAD(state->htim, new_arr);
    __HAL_TIM_SET_COMPARE(state->htim, state->channel, new_ccr);
    return 0;
}
```

The timer count frequency (640 kHz) is derived from the existing timer configuration: APB1 64 MHz, prescaler 99.

---

### `CM4/App/Src/app_cm4.c`

Added a new command handler block after the existing `CMD_MOTOR_ZERO` block:

```c
if (commands & CMD_TRIGGER_SOLENOID_HZ) {
    uint8_t  sol_id  = g_shared_memory.solenoid_id;
    uint16_t freq    = g_shared_memory.solenoid_freq_hz;
    // ... validate, call Solenoid_SetFrequencyHz(), Solenoid_SetDutyCycle(),
    //     Solenoid_StartTimed(), update solenoid_active_mask ...
    InterCore_AckCommand(CMD_TRIGGER_SOLENOID_HZ);
}
```

---

## 4. Data-flow Diagram

```
CM7 (every ~170 ms)                         CM4 (every 1 ms)
─────────────────────────────────────        ───────────────────────────
Audio_HasNewSamples()
  │
  ├─ FFT_ProcessBlock()   (diagnostics)
  │
  ├─ Goertzel_ProcessBlock()
  │     └─ per-string freq + magnitude
  │
  ├─ SongSched_Tick()
  │     └─ advances note index on timer
  │
  └─ TuningCtrl_RunHop()
        │
        ├─ Goertzel_GetExactFreq(str)  ──────> measured_freq_hz
        ├─ Goertzel_GetMagnitude(str)  ──────> confidence
        ├─ SongSched_Get{StringId,TargetFreq}
        │
        ├─ TuningCtrl_Update()
        │    ├─ note-change detection → reset PI, update FF
        │    ├─ SNR gate → freeze integral if confidence low
        │    ├─ lock-band hysteresis
        │    ├─ discrete PI + anti-windup
        │    ├─ slew limit
        │    └─ mechanical bounds
        │
        ├─ if cmd changed:
        │    InterCore_SendMotorGoto(str, cmd_steps_abs)
        │         └─ shared_memory.motor_cmd_position ──> Motor_MoveToPosition()
        │
        └─ ApplyActuation()
             ├─ lock↑: InterCore_TriggerSolenoidHz()
             │         └─ solenoid_freq_hz ──> Solenoid_SetFrequencyHz()
             │                                 Solenoid_StartTimed(INFINITE)
             └─ lock↓: CMD_STOP_SOLENOID ──> Solenoid_Stop()
```

---

## 5. Required Calibration Before Use

| # | What | How |
|---|------|-----|
| 1 | **Feed-forward LUT** (`tuning_ff_freq_bp`, `tuning_ff_steps_bp` in `tuning_controller.c`) | Home each motor. Step slowly in increments. Record `Goertzel_GetExactFreq()` vs. `Motor_GetState()->position`. Fill in ~4 breakpoints per string covering the note range. |
| 2 | **PI gains** `TUNING_KP`, `TUNING_KI` in `tuning_controller.h` | Increase `Kp` from the default until sustained oscillation appears; halve it. Then slowly increase `Ki`. Validate using the CSV log. |
| 3 | **SNR threshold** `TUNING_SNR_THRESHOLD` | Run with no string plucked. Read the `confidence` column in the CSV log. Set threshold to ~3× the observed noise-floor magnitude. |
| 4 | **Delay frames** `TUNING_DELAY_FRAMES` | After a motor move, count how many hops elapse before `measured_hz` stabilises. Set `DELAY_FRAMES` to that count + 1. |
| 5 | **Lock bands** `TUNING_LOCK_BAND_IN_HZ` / `OUT_HZ` | Observe the `error_hz` column at steady state. Set `IN` ≤ residual steady-state error. Set `OUT` at 2–3× `IN`. |

---

## 6. CSV Log Format

Printed over UART every 5 hops with prefix `[TC]`.  Header printed once at boot.

```
[TC] timestamp_ms, string_id, target_hz, measured_hz, error_hz,
     ff_steps, trim_steps, cmd_steps, cur_steps,
     lock, pi_en, delay_ctr, confidence, faults
```

| Column | Unit | Description |
|--------|------|-------------|
| `timestamp_ms` | ms | `HAL_GetTick()` |
| `string_id` | 0–5 | Active string |
| `target_hz` | Hz | Commanded pitch |
| `measured_hz` | Hz | Goertzel estimate |
| `error_hz` | Hz | `target - measured` |
| `ff_steps` | microsteps | FF-LUT output |
| `trim_steps` | microsteps | PI output |
| `cmd_steps` | microsteps | Final motor command |
| `cur_steps` | microsteps | Actual motor position (from CM4) |
| `lock` | 0/1 | Lock state |
| `pi_en` | 0/1 | PI enabled this hop |
| `delay_ctr` | hops | Remaining hold hops after note change |
| `confidence` | — | Goertzel magnitude |
| `faults` | bitmask | `TUNING_FAULT_STALL=1`, `TUNING_FAULT_SATURATED=2` |

# Phase 4 Context

Last verified against the codebase: 2026-03-18 (calibration section updated same date)

## Fast Mental Model

- The project is a dual-core STM32H755ZI system: CM7 does sensing/control, CM4 does actuator execution.
- The codebase still models 6 logical guitar strings everywhere.
- Stepper support is only commissioned in code for motors `0-3` (`E2`, `A2`, `D3`, `G3`).
- Solenoid support is only timer-backed for logical solenoids `1-3` (`A2`, `D3`, `G3`).
- Pitch sensing is live now: `ADC3 + BDMA + TIM1 TRGO + 2048-point FFT`.
- The active tuning path is FFT-based. Goertzel code still exists on disk but is not part of the runtime build.
- The active commissioning tools are:
  - `calibrate`: fully automatic per-string calibration with PI lock-in, SRAM persistence, and polynomial feedforward
  - `mapfrets`: manual-pluck LUT builder, restricted to configured motors `0-3`
- Feed-forward LUTs persist across reset via VBAT-backed backup SRAM (`0x38800000`, 4 KB).
- Calibration also fits and stores a quadratic polynomial `steps = a + b·f + c·f²` per string.

## What Is Actually Implemented

| Area | Current implementation | Important caveat |
| --- | --- | --- |
| Board | STM32H755ZI Nucleo, CM7 + CM4 split | Same board is the active target in this repo |
| Strings | 6 logical strings across shared IDs, tuning, scheduler, calibration, UART commands | Logical 6-string model is ahead of commissioned hardware |
| Steppers | 6 logical motor IDs exist | Only motors `0-3` have GPIO assignments in `motor_control.c` |
| Solenoids | 6 logical solenoid IDs exist | Only solenoids `1-3` have live PWM timers; `0`, `4`, `5` have `htim == NULL` |
| Pitch sensing | Live ADC capture with FFT peak detection | Trigger timer is `TIM1`, not `TIM6` |
| Tuning control | Live FF + PI controller on CM7 with automatic motor power management, 1/16 microstep safeguard, and cents-based lock bands | Uses 7-point LUTs; all gains/limits calibrated for 1/16 microstepping; controller halts if motors are not in 1/16 mode |
| Motor safeguard | `Motor_Update()` skips disabled motors — no stepping, no position drift | Software position freezes while disabled; may not match physical position on re-enable |
| Motor power mgmt | Tuning controller auto-enables motors for active strings, disables when idle | Uses `CMD_MOTOR_ENABLE_MASK` bitmask via `InterCore_SendMotorEnableMask()` |
| Playback | Hardcoded 12-note "Smoke on the Water" riff | No looping; only two-string D3/G3 chord data exists |
| Calibration | Active PI-lock sweep with feedforward homing, SRAM persistence, polynomial fitting | Solenoid excitation only live for strings `1-3`; E2 relies on manual pluck ring-down |
| LUT builder | Active fret-mapping state machine | Only supports configured motors `0-3` |
| Persistence | Backup SRAM (`0x38800000`, VBAT-backed) via `Cal_SaveToSRAM()` / `Cal_LoadFromSRAM()` | Restored at boot; magic + XOR checksum guard |

## Source Of Truth

If this file conflicts with older docs, trust the code below.

- `CM7/App/Src/app_cm7.c`
  Top-level CM7 state machine and runtime flow.
- `CM7/Core/Src/main.c`
  CM7 clocks, ADC3, BDMA, TIM1 trigger, UART COM init, boot flow.
- `CM7/App/Src/audio_input.c`
  DMA audio buffering and 50 percent overlap assembly.
- `CM7/App/Src/fft_processing.c`
  Active pitch-detection frontend.
- `CM7/App/Src/tuning_controller.c`
  Active closed-loop tuning logic and exact-Hz solenoid coordination.
- `CM7/App/Src/song_scheduler.c`
  Hardcoded song data and playback scheduler.
- `CM7/App/Src/calibration_controller.c`
  Automatic sweep-based LUT calibration.
- `CM7/App/Src/lut_builder.c`
  Manual-pluck fret-mapping tool.
- `CM7/App/Src/serial_cmd.c`
  Operator console and workflow commands.
- `CM4/App/Src/app_cm4.c`
  CM4 1 kHz loop and command execution.
- `CM4/App/Src/motor_control.c`
  Stepper mapping and software pulse generation.
- `CM4/App/Src/solenoid_control.c`
  Actual live PWM-backed solenoid implementation.
- `Shared/Inc/shared_memory.h`
  Inter-core contract.
- `Shared/Inc/application_config.h`
  Shared constants, including several stale comments/macros that do not match runtime reality.

## Where To Look First

- Audio and pitch path: `CM7/Core/Src/main.c`, `CM7/App/Src/audio_input.c`, `CM7/App/Src/fft_processing.c`
- Tuning behavior: `CM7/App/Src/tuning_controller.c`
- Playback behavior: `CM7/App/Src/song_scheduler.c`, `CM7/App/Src/app_cm7.c`
- Calibration, tuning, and commissioning: `CM7/App/Src/calibration_controller.c` (includes both calibration and tune state machines), `CM7/App/Src/lut_builder.c`
- Manual commands and workflows: `CM7/App/Src/serial_cmd.c`
- Stepper hardware/runtime: `CM4/App/Src/motor_control.c`
- Solenoid hardware/runtime: `CM4/App/Src/solenoid_control.c`, `CM4/Core/Src/main.c`
- Shared-memory fields and command flags: `Shared/Inc/shared_memory.h`, `CM7/App/Src/inter_core_cm7.c`, `CM4/App/Src/app_cm4.c`

## Runtime Architecture

### Core split

- CM7 owns:
  - audio capture orchestration
  - FFT processing
  - serial command parsing
  - song scheduling
  - tuning control
  - motor power management (auto enable/disable based on active strings)
  - calibration state machine
  - LUT builder state machine
- CM4 owns:
  - 1 kHz actuator loop
  - software-generated DRV8825 step pulses
  - PWM solenoid drive
  - timed/burst auto-stop logic
  - actuator status publication back to CM7

### Shared-memory link

- `g_shared_memory` lives in AXI SRAM and is shared by both cores.
- CM7 marks the shared region non-cacheable in `InterCore_ConfigMPU()`.
- HSEM is used for boot synchronization and core-to-core notifications.
- CM4 still polls command flags every `1 ms` in its control loop, so command handling does not depend purely on interrupts.

### Boot and startup flow

1. CM7 boots first, zeroes shared memory, configures clocks, configures shared-memory MPU attributes, and releases CM4 from the boot barrier.
2. CM4 wakes, initializes inter-core support and PWM timers, then `App_CM4_Main()` waits on the secondary boot semaphore before continuing.
3. CM4 calls `Motor_Init()`, sets motors `0-3` to `1/16` microstepping, initializes the solenoid subsystem, attempts to preload default logical open-string frequencies, and reports `MOTOR_STATUS_READY`.
4. CM7 waits for `MOTOR_STATUS_READY`, sends `CMD_HOME_MOTORS`, sets motors `0-3` to `1/16` microstepping again, initializes UART RX, FFT, restores calibration from backup SRAM (or seeds polynomial from base LUT), restores tune offsets from backup SRAM, initializes tuning control, and starts audio capture, then starts `TIM1`.
5. CM7 transitions through `STATE_IDLE` into `STATE_RUNNING`.

### Important startup reality

- CM4 startup does not enable motors.
- `Motor_Init()` resets software position state to `0`, but that is not sensor-backed homing.
- `CMD_HOME_MOTORS` means "move configured motors to software position 0", not "find a physical reference switch".

### Active CM7 states

`app_cm7.c` currently uses:

- `STATE_INIT`
- `STATE_IDLE`
- `STATE_CALIBRATING`
- `STATE_LUT_BUILDING`
- `STATE_TUNING`
- `STATE_RUNNING`
- `STATE_ERROR`

## Audio And Pitch Path

- ADC instance: `ADC3`
- Analog input: `PC2_C / ADC3_INP0`
- DMA: `BDMA_Channel0`
- Trigger timer: `TIM1` update TRGO
- Sample rate target: `6000 Hz`
- FFT size: `2048`
- DMA hop size: `1024` samples
- Effective control hop: `1024 / 6000 = 0.1707 s`, about `5.86 Hz`

### Buffering behavior

- DMA writes into a circular `audio_buffer`.
- Half-complete and complete callbacks mark which half was just filled.
- `Audio_GetBuffer()` shifts the previous half-window left and appends the newest half-buffer.
- The FFT always sees a full `2048`-sample window with `50 percent` overlap.

### FFT behavior

The active measurement path is `fft_processing.c`.

- Removes DC per block.
- Applies a Hanning window.
- Runs CMSIS real FFT.
- Computes magnitudes.
- Searches a bin range that works out to about `58.6 Hz` to `442 Hz` because `FFT_RESOLUTION` is integer-truncated in macros.
- Uses an adaptive per-bin noise floor (EMA) with configurable `NOISE_FLOOR_ALPHA`, `NOISE_FLOOR_SNR`, and `NOISE_FLOOR_MIN`.
- Uses local-maximum detection, a "fat peak" rejection heuristic, parabolic interpolation, and harmonic rejection.
- Sorts candidates by magnitude (strongest first) and keeps the top `NUM_STRINGS`.
- Legacy `NOISE_GATE_THRESHOLD` and `TUNING_SNR_THRESHOLD` have been removed — the adaptive floor replaces both.

### Current FFT outputs

- `active_frequencies[NUM_STRINGS]`
- `active_magnitudes[MAX_HARMONICS]`
- `active_string_count`
- `fft_dbg_mag[3]`, `fft_dbg_nf[3]`, `fft_dbg_peak` — debug snapshots (only populated when noise floor debug is active)

`fftprint` or `debug fft` toggles live UART printing of the accepted peaks (printed in `app_cm7.c`, outside the processing timer).

### Goertzel status

- `goertzel_processing.c/.h` still exist in the repo.
- `CM7/CMakeLists.txt` does not compile `goertzel_processing.c`.
- `app_cm7.c` never calls Goertzel functions.
- Any doc or comment that still describes Goertzel as the active runtime path is stale.

## Playback, Tuning, And Commissioning

### Song scheduler

`song_scheduler.c` contains one hardcoded song:

- "Smoke on the Water" intro
- 12 note events
- all notes are D3 + G3 two-string power chords
- no loop/repeat logic

### Tuning controller

`tuning_controller.c` is live and FFT-driven.

Current behavior:

- All positions, gains, and limits are in **1/16 microstep** units.
- Uses 7-point per-string feed-forward LUTs:
  - `tuning_ff_freq_bp[6][7]`
  - `tuning_ff_steps_bp[6][7]`
- FF lookup uses calibrated quadratic polynomial `steps = a + b·f + c·f²` via `FF_Interpolate()`, plus tune offset from `Cal_TuneGetOffset(str)`.
- Finds the closest accepted FFT peak to the target frequency (`GetFFTRadarLock`). No secondary magnitude threshold — peaks are already SNR-vetted by the FFT's adaptive noise floor.
- Measurement trust is now binary: `measured_freq_hz > 0.0f` means a valid lock was found.
- Applies per-string FF lookup plus discrete PI trim.
- Uses post-note delay, **cents-based lock-band hysteresis** (±10 cents in, ±20 cents out — computed at runtime relative to target frequency), minimum-move threshold (`16` microsteps), slew limit (`65000` microsteps/s), and default mechanical bounds of `-800` to `8000` microsteps.
- PI gains: `Kp = 48.0` microsteps/Hz, `Ki = 24.0` microsteps/Hz/s, trim clamp ±`6400` microsteps.
- Sends absolute motor targets through `InterCore_SendMotorGoto()`.
- Starts exact-Hz solenoid drive only on a rising lock edge.
- Stops the solenoid on a falling lock edge.

Microstep safeguard:

- At the top of every `TuningCtrl_RunHop()`, the controller reads `g_shared_memory.motor_microstep_status[0..3]` and verifies all configured motors are in mode `4` (1/16 step).
- If any motor is not in 1/16 mode, the controller prints a `*** HALT ***` warning over UART, disables all motors via `InterCore_SendMotorEnableMask(0)`, and returns immediately without issuing any motor commands.
- This safeguard exists because all controller limits and gains are calibrated for 1/16 microstepping. Running with a different mode would produce incorrect position commands that can damage the mechanical system.

Motor power management:

- The tuning controller tracks which motors are needed via `s_motor_enable_mask`.
- At each hop, it builds a bitmask from the song scheduler's active strings and sends `InterCore_SendMotorEnableMask()` when the mask changes.
- When the song ends (`!SongSched_IsActive()`), all motors are disabled.
- This minimizes current draw — only motors for strings in the current note are powered.

Important current limitation:

- There is no explicit "attack" or "pluck at note start" path in the tuning/playback loop.
- The tuning controller only enables solenoid excitation after a string is already considered locked.
- When a motor is re-enabled after being disabled, physical position may have drifted (no holding torque). The PI loop corrects via FFT feedback, but the first 2-5 hops may show larger pitch error.

### Automatic calibration (V3 — PI lock-in)

`calibration_controller.c` is active and compiled. Triggered by `calibrate <string>` or `calibrate all`.

#### Full state machine (per string)

```
STR_INIT → WAIT_PLUCK → WAIT_HOME
  → OPEN_SOL_BURST → OPEN_BURST_REENABLE → OPEN_PI_LOCK → OPEN_DONE
  → [for frets 1–7]:
      SOL_BURST → BURST_REENABLE → PI_LOCK → RECORD
  → BUILD
  → HOME_SOL_BURST → HOME_BURST_REENABLE → HOME_PI_LOCK → HOME_DONE
  → [next string or DONE]
```

#### Phase descriptions

| Phase | What happens |
|---|---|
| `STR_INIT` | Enables motor, resets all state, prompts user to pluck |
| `WAIT_PLUCK` | Listens for any FFT peak in 70–180% of nominal open freq with magnitude > 50. Wide window accepts any fret position. On detect: captures `s_open_detected_freq`, computes polynomial FF delta, sends motor to `s_open_homing_target`. On timeout (25 hops ≈ 4.3 s): sends motor to 0 and proceeds. |
| `WAIT_HOME` | Polls `|current_pos - s_open_homing_target| ≤ 50` steps. Timeout after 30 hops. |
| `OPEN_SOL_BURST` | Fires solenoid at nominal open freq for 1000 ms. Inhibits FFT. Resets PI state seeded from `s_open_homing_target`. |
| `OPEN_BURST_REENABLE` | Re-enables FFT sensing. |
| `OPEN_PI_LOCK` | PI targets `s_open_freq[str]`. **Dynamic trim cap**: if frequency error is shrinking hop-over-hop, cap = `s_cal_steps_max` (full range); if not converging for 3 consecutive hops (`CAL_OPEN_STALL_HOPS`), hard cap reverts to `CAL_PI_MAX_TRIM` (600 steps). Motor clamp = `±s_cal_steps_max`. Up to `CAL_MAX_RETRIES` (3) re-excites. |
| `OPEN_DONE` | Calls `InterCore_SendMotorZero()` — re-zeros motor at locked physical position. Sets `s_open_position_offset = 0`. Fret calibration now proceeds from this true home. |
| `SOL_BURST` | Fires solenoid at last measured frequency (or open freq for fret 1). Starts motor moving toward next fret via polynomial estimate + `s_open_position_offset`. Inhibits FFT. |
| `BURST_REENABLE` | Re-enables FFT, resets PI. |
| `PI_LOCK` | PI fine-tunes motor position using FFT feedback. Lock band ±1 Hz, 2 consecutive locked hops to confirm. Up to `CAL_MAX_RETRIES` (3) re-excites on signal loss or timeout. |
| `RECORD` | Saves `(actual_motor_steps, measured_freq)` to `s_buf`. Falls back to `s_last_valid_freq` or theoretical estimate if PI never locked. Advances `s_fret`. |
| `BUILD` | Calls `Cal_BuildTable()` (7-point LUT from samples), `Cal_FitPolynomial()` (quadratic least-squares), `Cal_SaveToSRAM()`. |
| `HOME_SOL_BURST` | Fires solenoid at last measured fret frequency. Commands motor to `s_open_position_offset` (= 0 after re-zero). |
| `HOME_PI_LOCK` | Same PI logic as `OPEN_PI_LOCK` targeting `s_open_freq[str]`. Uses **`CAL_HOME_MAX_RETRIES = 8`** (not the fret `CAL_MAX_RETRIES = 3`) and **`CAL_HOME_REEXCITE_MS = 700 ms`** bursts (not 300 ms) — string is decayed after full calibration sweep and needs more patient re-excitation. |
| `HOME_DONE` | Advances to next string in mask, or transitions to `CAL_PHASE_DONE`. |

#### Polynomial feedforward homing (WAIT_PLUCK → OPEN_PI_LOCK)

When the string is out of tune at pluck time (e.g., E2 at 90 Hz instead of 82.41 Hz):

1. `s_open_detected_freq` = pluck reading (e.g., 90 Hz)
2. `delta = poly(open_freq) - poly(detected_freq)` = 0 − 1080 = −1080 steps
3. `s_open_homing_target = current_pos + delta` = −1080 (clamped to ±`s_cal_steps_max`)
4. Motor moves to −1080. `WAIT_HOME` polls against this target (not against 0).
5. `OPEN_PI_LOCK` starts from −1080 with dynamic trim cap — converges to 82.41 Hz regardless of how far off the LUT was.
6. `OPEN_DONE` re-zeros motor. Position 0 = true open-string frequency. Fret polynomial targets are now accurate.

If no pluck detected (timeout): `s_open_homing_target = 0`, same as old behavior.

#### Key calibration parameters

| Constant | Value | Meaning |
|---|---|---|
| `CAL_NUM_FRETS` | 8 | Frets measured: 0 (anchor) + frets 1–7 |
| `CAL_MOTOR_SPEED_FAST` | 3000 steps/s | Inter-fret transit speed |
| `CAL_HOME_POS_THRESHOLD` | 50 steps | Arrival tolerance for `WAIT_HOME` |
| `CAL_HOME_TIMEOUT_HOPS` | 30 hops | Max wait for motor to reach homing target |
| `CAL_PLUCK_TIMEOUT` | 25 hops ≈ 4.3 s | Max wait for user pluck before proceeding |
| `CAL_SOL_DURATION_MS` | 1000 ms | Primary burst duration |
| `CAL_REEXCITE_DURATION_MS` | 300 ms | Fret re-excite burst duration |
| `CAL_HOME_REEXCITE_MS` | 700 ms | Home re-excite burst duration (longer for decayed string) |
| `CAL_BURST_WAIT_HOPS` | 8 hops ≈ 1.36 s | FFT blackout after primary or home burst |
| `CAL_REEXCITE_WAIT_HOPS` | 4 hops ≈ 0.68 s | FFT blackout after fret re-excite |
| `CAL_SOLENOID_DUTY` | 50% | Solenoid PWM duty cycle |
| `CAL_PI_KP` | 3.0 | Proportional gain (steps/Hz) |
| `CAL_PI_KI` | 1.5 | Integral gain (steps/Hz/s) |
| `CAL_PI_MAX_TRIM` | 600 steps | Hard trim cap (safety, active when PI is NOT converging) |
| `CAL_PI_LOCK_TIMEOUT` | 25 hops ≈ 4.3 s | PI lock timeout per attempt |
| `CAL_DEAD_HOPS` | 3 hops | No-signal hops before early-exit re-excite |
| `CAL_LOCK_CONSEC_HOPS` | 2 hops | Consecutive in-band hops to confirm lock |
| `CAL_MAX_RETRIES` | 3 | Max re-excites per fret PI_LOCK |
| `CAL_HOME_MAX_RETRIES` | 8 | Max re-excites for HOME_PI_LOCK |
| `CAL_OPEN_STALL_HOPS` | 3 hops | Non-converging hops before dynamic cap reverts to hard limit |
| `CAL_STALL_WARN_STEPS` | 250 steps | Position error threshold for stall warning print |
| `s_cal_steps_max[str]` | per-string | Hard motor position ceiling during calibration |

#### Per-string hardware capability during calibration

| String | Motor | Solenoid | Calibration capability |
|---|---|---|---|
| 0 (E2) | Configured | **None** (htim = NULL) | Relies entirely on manual pluck ring-down. OPEN/fret/HOME solenoid calls are safe no-ops. Polynomial LUT less accurate (old data). |
| 1 (A2) | Configured | Live (TIM3_CH2) | Fully automatic. Best calibrated. |
| 2 (D3) | Configured | Live (TIM2_CH1) | Fully automatic. |
| 3 (G3) | Configured | Live (TIM4_CH1) | Fully automatic. |
| 4 (B3) | **Unconfigured** | None | Do not calibrate. |
| 5 (E4) | **Unconfigured** | None | Do not calibrate. |

#### Measured LUT data (last run — loaded into tuning_controller.c)

**A2 (string 1):**

| Fret | Freq (Hz) | Steps |
|---|---|---|
| 0 | 110.00 | 0 |
| 1 | 115.54 | 751 |
| 2 | 121.57 | 1549 |
| 3 | 127.57 | 2370 |
| 4 | 139.23 | 4103 |
| 5 | 145.69 | 5013 |
| 6 | 151.22 | 5955 |

**D3 (string 2):**

| Fret | Freq (Hz) | Steps |
|---|---|---|
| 0 | 146.83 | 0 |
| 1 | 160.03 | 2331 |
| 2 | 170.43 | 3363 |
| 3 | 179.45 | 4417 |
| 4 | 188.51 | 5477 |
| 5 | 197.45 | 6533 |
| 6 | 205.19 | 7499 |

**G3 (string 3):**

| Fret | Freq (Hz) | Steps |
|---|---|---|
| 0 | 196.00 | 0 |
| 1 | 207.65 | 910 |
| 2 | 233.38 | 2583 |
| 3 | 248.76 | 3485 |
| 4 | 262.49 | 4480 |
| 5 | 277.12 | 5523 |
| 6 | 292.19 | 6646 |

**E2 (string 0):** older data, less reliable — `{82.41, 82.69, 89.97, 96.05, 100.07, 106.37, 112.82}` Hz, `{0, 267, 1080, 1623, 2029, 2706, 3385}` steps.

#### Persistence (backup SRAM)

- `Cal_LoadFromSRAM()`: called at boot. Restores `tuning_ff_freq_bp`, `tuning_ff_steps_bp`, and `cal_poly_coeff` if magic word (`0xCA115701`) and XOR checksum both pass. Returns 1 on success.
- `Cal_SaveToSRAM()`: called at end of each string's `BUILD` phase. Writes all 6 strings.
- `Cal_FitFromBaseLUT()`: called at boot if `Cal_LoadFromSRAM()` returns 0. Seeds `cal_poly_coeff` from the hardcoded LUT so the tuning controller has a valid polynomial from first boot.
- SRAM region: `0x38800000`, 4 KB, powered by VBAT regulator. `Cal_EnableBkpSRAM()` calls `HAL_PWR_EnableBkUpAccess()` + `__HAL_RCC_BKPRAM_CLK_ENABLE()` + `HAL_PWREx_EnableBkUpReg()` before every read or write.

#### Dynamic trim cap (OPEN_PI_LOCK only)

Protects against motor runaway when FFT gives a sustained wrong reading, while allowing the PI to travel as far as physically needed when it is genuinely converging:

- `s_pi_last_abs_error`: absolute frequency error from the previous hop. Initialized to 9999 so the first hop always counts as converging.
- `s_pi_stall_cnt`: counts consecutive hops where error is NOT shrinking. Reset to 0 whenever error shrinks.
- If `s_pi_stall_cnt < CAL_OPEN_STALL_HOPS (3)`: `trim_cap = s_cal_steps_max` — full range allowed.
- If `s_pi_stall_cnt >= 3`: `trim_cap = CAL_PI_MAX_TRIM (600)` — hard safety cap re-engages.
- Both variables reset in `STR_INIT` and `OPEN_SOL_BURST`.
- This logic is **only in OPEN_PI_LOCK**. Fret `PI_LOCK` and `HOME_PI_LOCK` use the fixed `CAL_PI_MAX_TRIM` cap.

### LUT builder

`lut_builder.c` is also active and compiled.

Current behavior:

- Triggered by `mapfrets`.
- Manual-pluck workflow.
- Measures frets `0-6`.
- Restricts the requested string mask to `0x0F`, so only strings `0-3` are allowed.
- Enables the requested motors before starting.
- Writes measured fret data into the same runtime LUT arrays.

This is the commissioning tool that best matches the currently wired stepper hardware.

## Hardware Mapping

### Logical string order

The code consistently uses:

- `0 = E2`
- `1 = A2`
- `2 = D3`
- `3 = G3`
- `4 = B3`
- `5 = E4`

### Stepper motors

| Motor ID | Logical string | STEP | DIR | M0 | M1 | M2 | EN | Code status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | E2 | `PE10` | `PE11` | `PE12` | `PE13` | `PE14` | `PD11` | Configured |
| 1 | A2 | `PD12` | `PD13` | `PE2` | `PE4` | `PE7` | `PC9` | Configured |
| 2 | D3 | `PA8` | `PC7` | `PC6` | `PC8` | `PA3` | `PC0` | Configured |
| 3 | G3 | `PC3` | `PB1` | `PF10` | `PG12` | `PG14` | `PG6` | Configured |
| 4 | B3 | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Unconfigured |
| 5 | E4 | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Unconfigured |

### Solenoids

Logical solenoid IDs still run from `0` to `5`, but only three are timer-backed in the current CM4 runtime.

| Solenoid ID | Logical string | Timer / channel | Pin | Runtime status |
| --- | --- | --- | --- | --- |
| 0 | E2 | None | None | No PWM timer assigned |
| 1 | A2 | `TIM3_CH2` | `PB5` | Live |
| 2 | D3 | `TIM2_CH1` | `PA5` | Live |
| 3 | G3 | `TIM4_CH1` | `PB6` | Live |
| 4 | B3 | None | None | No PWM timer assigned |
| 5 | E4 | None | None | No PWM timer assigned |

Why this is true:

- `TIM1` was moved to CM7 and is used as the ADC trigger timer.
- `TIM16` and `TIM17` still appear in `CM4/Core/Src/stm32h7xx_hal_msp.c`, but CM4 `main.c` only initializes `TIM2`, `TIM3`, and `TIM4`.
- `solenoid_control.c` sets `htim = NULL` for logical solenoids `0`, `4`, and `5`.

## CM4 Actuation Behavior

### Motor control

`motor_control.c` is the live stepper implementation.

- Step pulses are generated in software, not by timer PWM.
- Pulse width uses the DWT cycle counter with a `3 us` HIGH time.
- `Motor_Update()` runs at `1 kHz`.
- `Motor_Update()` skips disabled motors entirely: no STEP pin toggling, no position counter advancement, no speed accumulation. This prevents software/hardware position drift when the DRV8825 EN pin is HIGH.
- Position mode is a simple proportional target-speed rule with braking-distance logic.
- Speed ramps are acceleration-limited.

### Solenoid control

`solenoid_control.c` supports:

- string/fret lookup-table frequency setting
- direct exact-Hz frequency setting
- duty-cycle adjustment
- infinite, timed, and burst activation modes
- 1 ms auto-stop updates

The exact-Hz path is real and active:

- CM7 uses `InterCore_TriggerSolenoidHz()`
- shared memory sets `CMD_TRIGGER_SOLENOID_HZ`
- CM4 handles that command in `app_cm4.c`
- CM4 calls `Solenoid_SetFrequencyHz()`

The batch motor enable/disable path is real and active:

- CM7 uses `InterCore_SendMotorEnableMask(uint8_t mask)`
- shared memory sets `CMD_MOTOR_ENABLE_MASK` (bit 14) with `motor_cmd_enable_mask` bitmask
- CM4 loops through all 6 motors, enabling if the bit is set, disabling if clear
- The tuning controller uses this to atomically manage motor power for chord transitions

## Operator Workflows And Commands

### UART console

- Runs on CM7 over board COM1 / `USART3`
- `115200` baud
- `printf()` is redirected to UART
- RX is interrupt-driven and line-buffered

### Main command groups

- Motor control: `start`, `speed`, `accel`, `maxspeed`, `move`, `goto`, `microstep`, `enable`, `disable`, `zero`, `home`, `stop`, `status`
- Solenoid control: `sol`, `solstop`, `solstatus`
- Commissioning: `calibrate`, `tune`, `mapfrets`, `printlut`, `fftprint`
- Debug: `debug [hop|nf|fft|all|off]` — toggles hop timer, noise floor, FFT peaks, all, or all off
- Playback/demo: `smoke`, `demo`
- Presets/help: `save`, `delete`, `presets`, `help`

### Workflow caveats that matter

- `start` only unlocks manual movement commands in the CM7 CLI. It does not enable motors.
- `sol` catches null-timer solenoids (`0`, `4`, `5`) on the CM7 side with a warning message and early return, preventing a silent no-op.
- `mapfrets` enables the requested motors. `calibrate` does not.
- `smoke` only targets D3/G3, which matches the currently live playback-friendly channels. The tuning controller now auto-enables the needed motors during playback and disables them when the song ends.
- `demo` still targets solenoid `0`, which has no live PWM timer in the current runtime.

## Timer Calibration

- All timers (ADC trigger, solenoid PWM) share the same PLL-derived clock, which runs ~0.52% fast.
- `TIMER_CALIBRATION_FACTOR = 1.005245182f` is defined in `application_config.h` and used in:
  - FFT parabolic interpolation (frequency correction)
  - Solenoid ARR calculations (`Solenoid_SetFrequency`, `Solenoid_SetFrequencyHz`, `main.c` timer inits)
  - Solenoid `GetFrequency` reverse calculation
- CCR duty cycle calculations consistently use `(ARR + 1)` as the period (timer counts 0 to ARR inclusive).

## Debug And Timing Infrastructure

- DWT cycle counter (Cortex-M7, 396 MHz, ~2.5 ns resolution) measures hop interval and processing time.
- `app_debug_flags` (toggled via `debug` command) controls:
  - `DEBUG_FLAG_HOP_TIMER`: periodic 5s print of hop interval, proc time, and proc max
  - `DEBUG_FLAG_NOISE_FLOOR`: per-bin noise floor vs magnitude at key bins (~1 Hz decimated)
- `fft_print_enabled` (toggled via `fftprint` or `debug fft`): live FFT peak output
- All debug prints run AFTER the proc timer measurement in `app_cm7.c` to avoid inflating processing time.
- Measured baseline: hop interval ~170 ms, processing time ~1950 us (1.1% of budget).

## UART RX Fix

- The UART RX callback (`HAL_UART_RxCpltCallback`) no longer calls blocking `HAL_UART_Transmit` in ISR context.
- Echo characters are buffered in a ring buffer and drained by `Debug_UART_FlushEcho()` from the main loop.
- This prevents a race condition where ISR TX and main-loop `printf` TX could deadlock the UART handle, permanently killing RX re-arming.

## Current Verified Gotchas

These are real current behaviors or mismatches that future sessions should keep in mind.

### 1. ~~Startup and normal playback do not enable motors~~ (RESOLVED)

- Motors still start disabled.
- The tuning controller now auto-enables motors for active strings during song playback via `InterCore_SendMotorEnableMask()` and disables them when the song ends.
- `calibrate` still does not enable motors. `mapfrets` and explicit `enable` commands still work as before.

### 2. ~~Motor stepping ignores the enabled flag~~ (RESOLVED)

- `Motor_Update()` now skips disabled motors entirely: no STEP pin toggling, no position counter advancement, no speed/accumulator state.
- Software position freezes while disabled and resumes from the frozen value on re-enable.
- Caveat: physical position may drift while disabled (no holding torque). The PI loop corrects on re-enable via FFT feedback.

### 3. `calibrate all` is partially misaligned with current hardware

- Strings `1` (A2), `2` (D3), `3` (G3): fully automatic with solenoid excitation. **Use these.**
- String `0` (E2): motor is configured but solenoid `0` has no timer (`htim = NULL`). Calibration proceeds but relies entirely on the decay of the initial manual pluck. Works but is less reliable — solenoid re-excites during `OPEN_PI_LOCK`, `PI_LOCK`, and `HOME_PI_LOCK` are silent no-ops.
- Strings `4` (B3) and `5` (E4): motors not configured, no solenoids. **Do not calibrate.**
- Recommended: `calibrate 1`, `calibrate 2`, `calibrate 3` individually, or mask `0x0E` for A2+D3+G3.

### 4. `demo` is stale relative to current solenoid mapping

- The demo uses solenoid `0` for E2.
- Solenoid `0` has no timer assigned.
- So the demo sequence no longer matches live hardware support.

### 5. Replay state in the song scheduler is not fully reset-safe

- `SongSched_Tick()` uses a function-local static `first_tick`.
- `SongSched_Init()` does not reset that flag.
- The first play is clean; subsequent restarts are not fully reset-safe.

### 6. Several comments and headers are now stale

- `audio_input.h` still says audio acquisition is a placeholder.
- `tuning_controller.h` still describes a Goertzel-based runtime.
- `inter_core_cm7.h` still says homing uses limit switches.
- `motor_control.h` still calls homing a future placeholder even though software zero-return is implemented.
- `app_cm4.c` still talks about all 6 solenoids having independent timers.

### 7. Shared config macros are partially stale

- `NUM_SOLENOIDS` is still `3` in `application_config.h`.
- `CM7_AUDIO_PROCESS_FREQ_HZ` is still `100`, but the real hop rate is about `5.86 Hz`.
- Shared-memory comments still describe the solenoid path as "currently 3, expandable to 6" even though the runtime now has 6 logical IDs and only 3 timer-backed channels.

### 8. Legacy solenoid fields still exist in shared memory

- `solenoid_triggers`
- `solenoid_duration_ms`

The active CM4 handler actually uses:

- `solenoid_id`
- `solenoid_string`
- `solenoid_fret`
- `solenoid_duty_cycle`
- `solenoid_mode`
- `solenoid_activation_time_ms`
- `solenoid_burst_cycles`
- `solenoid_freq_hz` for exact-Hz mode

## Build And Flash

Current build setup in the repo:

- Top-level build uses CMake with Ninja.
- `CMakePresets.json` defines `Debug` and `Release`.
- `CM7/CMakeLists.txt` and `CM4/CMakeLists.txt` build the two cores separately under the shared top-level build.

## Motor Speed Benchmarking (2026-03-25)

### Test setup

- Motor 1 (A2 string), 1/16 microstepping
- Move distance: 5955 microsteps (fret 0 → fret 6 equivalent)
- Timing measured via `[BENCH]` print in `serial_cmd.c` (HAL_GetTick delta from move start to `motor_is_moving` going false)
- Step pulse width: DRV8825 `STEP_PULSE_US` in `motor_control.c`

### Results

| Accel (steps/s²) | Max Speed (steps/s) | Pulse (µs) | Time (ms) | Notes |
|---|---|---|---|---|
| 100,000 | 20,000 | 3 | 729 | Baseline (default settings) |
| 50,000 | 20,000 | 3 | 894 | Accel too low, slow ramp hurts |
| 200,000 | 20,000 | 3 | 1066 | Motor stalling — accel too aggressive |
| 100,000 | 20,000 | 3 | 730 | Baseline confirmed |
| 100,000 | 40,000 | 3 | 615 | Speed increase helps |
| 100,000 | 60,000 | 3 | 611 | **Plateau — no further gain** |
| 100,000 | 40,000 | 2 | 610 | Reduced pulse width, no gain |
| 100,000 | 60,000 | 2 | 611 | Still plateaued at ~610 ms |

### Analysis

- **Acceleration sweet spot**: 100,000 steps/s². Lower is too slow to ramp; higher causes stalling (stepper can't follow aggressive torque demands at low speed).
- **Speed ceiling**: ~40,000 microsteps/s at 1/16 microstepping = 2,500 full steps/s = 12.5 rev/s. Beyond this the motor physically cannot keep up.
- **Pulse width**: Reducing from 3µs to 2µs (DRV8825 min is 1.9µs) did not help because the bottleneck is the motor, not the software pulse generation.
- **610 ms plateau**: Consistent across all speed/pulse combinations above 40k steps/s.

### Step generation bottleneck (software)

- Step pulses are bit-banged in `Motor_Update()` which runs in the 1kHz control loop (`app_cm4.c`).
- Each pulse blocks for `STEP_PULSE_US` via DWT cycle-counter busy-wait.
- At 40,000 steps/s: 40 pulses/tick × 2µs = 80µs blocking per tick (out of 1000µs budget). Manageable.
- At higher speeds, the motor stalls before the software budget is exhausted.
- With multiple motors moving simultaneously, blocking time per tick scales linearly (e.g., 4 motors × 40 pulses × 2µs = 320µs).

### Options to go faster

1. **Reduce microstepping** (e.g., 1/16 → 1/4): same physical distance = 4× fewer pulses. GPIO pins for M0/M1/M2 are already wired and `Motor_SetMicrostepMode()` exists. Tradeoff: more vibration, higher risk of skipped steps.
2. **Higher supply voltage** to DRV8825: more voltage = more torque at speed, raising the physical RPM ceiling.
3. **Hardware timer pulse generation** (planned for final custom PCB): decouple pulse generation from the 1kHz control loop entirely. One high-frequency timer (~500kHz) as a shared timebase with per-motor accumulators in its ISR, or one dedicated timer per motor. Eliminates the blocking `Motor_DelayUs()` call and allows much higher pulse rates.
4. **Hybrid microstepping**: use full/quarter steps for fast travel, switch to 1/16 for fine positioning. Requires runtime microstep mode changes (already supported in code).

Current flash addresses from `README.md`:

- CM7: `0x08000000`
- CM4: `0x08100000`

## Older Docs To Treat As History, Not Truth

These files are still useful for background, but they do not fully match the current implementation:

- `PROJECT_CONTEXT.md`
- `TUNING_CONTROLLER_CHANGES.md`
- `MOTOR_CONTROL_API.md`
- parts of `audio_input.h`
- parts of `tuning_controller.h`
- parts of `inter_core_cm7.h`
- parts of `motor_control.h`
- parts of `app_cm4.c`
- parts of `solenoid_control.c`
- parts of `solenoid_control.h`

## Bottom Line

The current repo is best understood as:

- a live 6-string logical architecture
- 4 configured stepper channels (0–3)
- 3 live timer-backed solenoid channels (1–3)
- a real CM7 audio/FFT sensing loop
- a real CM7 tuning controller with calibrated polynomial feedforward
- a fully functional automatic calibration system with VBAT-backed SRAM persistence
- polynomial feedforward homing: pluck detection → polynomial delta → motor moves to estimated open position → PI locks → motor re-zeros at true open position
- dynamic PI trim cap in OPEN_PI_LOCK: expands to full mechanical range when converging, falls back to 600-step hard cap when stalled
- patient HOME_PI_LOCK: 8 retries with 700 ms bursts (vs 3 retries / 300 ms for frets)
- motor safeguard preventing disabled motors from stepping or drifting position
- microstep safeguard preventing the tuning controller from running if motors are not in 1/16 mode
- automatic motor power management driven by the tuning controller and song scheduler
- a partially commissioned hardware path with several stale comments and a few remaining behavioral notes

If a future session needs bench-realistic assumptions, use this stricter model:

- Motors wired and calibrated: `1–3` (A2, D3, G3). Motor `0` (E2) wired but no solenoid.
- Solenoids with live PWM: `1–3`
- Playback path that best matches current commissioned strings: the D3/G3 `smoke` workflow
- Best calibration targets: `calibrate 1`, `calibrate 2`, `calibrate 3`
- Commissioning path for manual measurement: `mapfrets` on strings `0-3`

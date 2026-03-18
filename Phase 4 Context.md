# Phase 4 Context

Last verified against the codebase: 2026-03-18

## Fast Mental Model

- The project is a dual-core STM32H755ZI system: CM7 does sensing/control, CM4 does actuator execution.
- The codebase still models 6 logical guitar strings everywhere.
- Stepper support is only commissioned in code for motors `0-3` (`E2`, `A2`, `D3`, `G3`).
- Solenoid support is only timer-backed for logical solenoids `1-3` (`A2`, `D3`, `G3`).
- Pitch sensing is live now: `ADC3 + BDMA + TIM1 TRGO + 2048-point FFT`.
- The active tuning path is FFT-based. Goertzel code still exists on disk but is not part of the runtime build.
- The active commissioning tools are:
  - `calibrate`: PI lock-in calibration with solenoid burst excitation, per-fret sweep, polynomial + LUT fitting, SRAM-persisted
  - `mapfrets`: manual-pluck LUT builder, restricted to configured motors `0-3`
  - `tune`: open-string pitch correction — detects frequency via FFT, adjusts motor bidirectionally (forward if flat, backward if sharp), saves offset to SRAM
- Feed-forward LUTs and polynomial coefficients persist to backup SRAM (0x38800000, VBAT-backed, 4 KB). Tune offsets also persist.

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
| Calibration | PI lock-in fret calibration with solenoid burst, polynomial fit, SRAM persistence | Real hardware support is incomplete for all 6 strings |
| Tune | Open-string pitch correction with bidirectional motor adjustment and SRAM persistence | Only strings with solenoids (1-3) get automatic re-excite; others need manual re-pluck |
| LUT builder | Active fret-mapping state machine | Only supports configured motors `0-3` |
| Persistence | Backup SRAM (VBAT-backed) stores calibration LUT, polynomial, and tune offsets | Survives power cycles; tune offsets cleared on recalibration |

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

`calibration_controller.c` is active and compiled.

Current behavior:

- Triggered by `calibrate [str ...]`.
- Uses FFT results, not Goertzel.
- Per-string flow: prompt pluck → detect pluck → home motor → for each fret 1–7:
  - Fire solenoid at previous fret's measured frequency (1 s burst) while motor moves toward next fret's polynomial-estimated position.
  - After burst+wait (8 hops), re-enable FFT and run inline PI controller (KP=3.0, KI=1.5, max trim ±600 microsteps) to lock onto the target fret frequency.
  - Lock criterion: 2 consecutive hops within 1 Hz deadband.
  - If string decays (3 consecutive no-signal hops), re-excite at last valid frequency. Up to 3 retries.
  - Record (actual_motor_steps, measured_freq) on lock or best estimate on failure.
- After all frets: build 7-point LUT (`Cal_BuildTable`) + fit quadratic polynomial `steps = a + b·f + c·f²` (`Cal_FitPolynomial`).
- Persist LUT + polynomial to backup SRAM (0x38800000, VBAT-backed, 4 KB) with magic number + XOR checksum.
- Boot: `Cal_LoadFromSRAM()` restores previous calibration; falls back to `Cal_FitFromBaseLUT()` to seed polynomial from hardcoded LUT.
- Per-string safety caps prevent motor over-travel: E2=5000, A2=6500, D3=9000, G3=7000.

What it assumes:

- The target motor can physically move.
- The target solenoid can be driven continuously at exact Hz (for re-excitation).

That assumption is only valid today for part of the system.

### Open-string tuning

`calibration_controller.c` also contains the tune controller (added alongside calibration, sharing helpers).

Current behavior:

- Triggered by `tune [str ...]` (default mask 0x0F = motors 0-3).
- Per-string flow: prompt pluck → detect pluck via FFT → compute initial motor position using polynomial derivative sensitivity → move motor → PI lock to open-string frequency.
- **Bidirectional**: motor moves forward (positive steps) if string is flat, backward (negative steps) if sharp. Clamped to [TUNING_STEPS_MIN_DEFAULT (-800), s_cal_steps_max[str]].
- Solenoid re-excite on signal decay for strings 1-3 (have solenoids). Strings 0/4/5 re-prompt the user to pluck.
- On lock, the motor position is saved as `tune_offset[str]` and applied in `FF_Interpolate()` so all subsequent tuning controller commands are shifted.
- Tune offsets persist to backup SRAM (separate block after calibration data, own magic + checksum).
- Tune offsets are cleared when `Cal_Start()` runs (recalibration invalidates offsets).

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

### 3. `calibrate all` is not aligned with current hardware

- The calibration path assumes both a working motor and a timer-backed exact-Hz solenoid.
- That is not true for `E2`, `B3`, or `E4`.
- Motors `4` and `5` are not configured.
- Solenoids `0`, `4`, and `5` have no live timer.

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
- 4 configured stepper channels
- 3 live timer-backed solenoid channels
- a real CM7 audio/FFT sensing loop
- a real CM7 tuning controller with cents-based lock bands and 1/16 microstep safeguard
- a real LUT-builder, calibration framework (PI lock-in + polynomial fit), and open-string tune controller
- calibration LUT, polynomial, and tune offsets persisted to backup SRAM (VBAT-backed)
- motor safeguard preventing disabled motors from stepping or drifting position
- microstep safeguard preventing the tuning controller from running if motors are not in 1/16 mode
- automatic motor power management driven by the tuning controller and song scheduler
- a partially commissioned hardware path with several stale comments and a few remaining behavioral notes

If a future session needs bench-realistic assumptions, use this stricter model:

- Motors that exist in code and are wired enough to target: `0-3`
- Solenoids that exist in code and can actually generate PWM: `1-3`
- Playback path that best matches current commissioned strings: the D3/G3 `smoke` workflow
- Commissioning path that best matches current hardware: `mapfrets` on strings `0-3`, `calibrate` on strings `1-3` (solenoid-backed), `tune` on strings `0-3`

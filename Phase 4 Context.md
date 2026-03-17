# Phase 4 Context

Last verified against the codebase: 2026-03-17

## Fast Mental Model

- The project is a dual-core STM32H755ZI system: CM7 does sensing/control, CM4 does actuator execution.
- The codebase still models 6 logical guitar strings everywhere.
- Stepper support is only commissioned in code for motors `0-3` (`E2`, `A2`, `D3`, `G3`).
- Solenoid support is only timer-backed for logical solenoids `1-3` (`A2`, `D3`, `G3`).
- Pitch sensing is live now: `ADC3 + BDMA + TIM1 TRGO + 2048-point FFT`.
- The active tuning path is FFT-based. Goertzel code still exists on disk but is not part of the runtime build.
- The active commissioning tools are:
  - `calibrate`: automatic sweep, RAM-only LUT update, not aligned with all current hardware
  - `mapfrets`: manual-pluck LUT builder, restricted to configured motors `0-3`
- Feed-forward LUTs are runtime RAM data only. Nothing persists across reset.

## What Is Actually Implemented

| Area | Current implementation | Important caveat |
| --- | --- | --- |
| Board | STM32H755ZI Nucleo, CM7 + CM4 split | Same board is the active target in this repo |
| Strings | 6 logical strings across shared IDs, tuning, scheduler, calibration, UART commands | Logical 6-string model is ahead of commissioned hardware |
| Steppers | 6 logical motor IDs exist | Only motors `0-3` have GPIO assignments in `motor_control.c` |
| Solenoids | 6 logical solenoid IDs exist | Only solenoids `1-3` have live PWM timers; `0`, `4`, `5` have `htim == NULL` |
| Pitch sensing | Live ADC capture with FFT peak detection | Trigger timer is `TIM1`, not `TIM6` |
| Tuning control | Live FF + PI controller on CM7 | Uses 7-point LUTs, still placeholder until measured |
| Playback | Hardcoded 12-note "Smoke on the Water" riff | No looping; only two-string D3/G3 chord data exists |
| Calibration | Active automatic sweep state machine | Real hardware support is incomplete for all 6 strings |
| LUT builder | Active fret-mapping state machine | Only supports configured motors `0-3` |
| Persistence | None | LUT changes are lost on reset |

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
- Calibration and commissioning: `CM7/App/Src/calibration_controller.c`, `CM7/App/Src/lut_builder.c`
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
4. CM7 waits for `MOTOR_STATUS_READY`, sends `CMD_HOME_MOTORS`, sets motors `0-3` to `1/16` microstepping again, initializes UART RX, FFT, tuning control, and audio capture, then starts `TIM1`.
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
- Uses local-maximum detection, a "fat peak" rejection heuristic, parabolic interpolation, and harmonic rejection.
- Sorts candidates by magnitude and keeps the top `NUM_STRINGS`.

### Current FFT outputs

- `active_frequencies[NUM_STRINGS]`
- `active_magnitudes[MAX_HARMONICS]`
- `active_string_count`

`fftprint` toggles live UART printing of the accepted peaks.

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

- Uses 7-point per-string feed-forward LUTs:
  - `tuning_ff_freq_bp[6][7]`
  - `tuning_ff_steps_bp[6][7]`
- Finds the closest accepted FFT peak to the target frequency.
- Applies per-string FF lookup plus discrete PI trim.
- Uses post-note delay, lock-band hysteresis, minimum-move threshold, slew limit, and default mechanical bounds of `-50` to `2000` steps.
- Sends absolute motor targets through `InterCore_SendMotorGoto()`.
- Starts exact-Hz solenoid drive only on a rising lock edge.
- Stops the solenoid on a falling lock edge.

Important current limitation:

- There is no explicit "attack" or "pluck at note start" path in the tuning/playback loop.
- The tuning controller only enables solenoid excitation after a string is already considered locked.

### Automatic calibration

`calibration_controller.c` is active and compiled.

Current behavior:

- Triggered by `calibrate`.
- Uses FFT results, not Goertzel.
- Sweeps in `20` microstep increments.
- Stops at `1200` steps max or once fret-7 range is reached.
- Updates `tuning_ff_*` arrays in RAM only.

What it assumes:

- The target motor can physically move.
- The target solenoid can be driven continuously at exact Hz.

That assumption is only valid today for part of the system.

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

## Operator Workflows And Commands

### UART console

- Runs on CM7 over board COM1 / `USART3`
- `115200` baud
- `printf()` is redirected to UART
- RX is interrupt-driven and line-buffered

### Main command groups

- Motor control: `start`, `speed`, `accel`, `maxspeed`, `move`, `goto`, `microstep`, `enable`, `disable`, `zero`, `home`, `stop`, `status`
- Solenoid control: `sol`, `solstop`, `solstatus`
- Commissioning: `calibrate`, `mapfrets`, `printlut`, `fftprint`
- Playback/demo: `smoke`, `demo`
- Presets/help: `save`, `delete`, `presets`, `help`

### Workflow caveats that matter

- `start` only unlocks manual movement commands in the CM7 CLI. It does not enable motors.
- `sol` intentionally warns and returns for solenoids `0`, `4`, and `5` because those have no timer assigned.
- `mapfrets` enables the requested motors. `smoke` and `calibrate` do not.
- `smoke` only targets D3/G3, which matches the currently live playback-friendly channels, but it still depends on motor enable state and the tuning-controller lock-before-actuation behavior.
- `demo` still targets solenoid `0`, which has no live PWM timer in the current runtime.

## Current Verified Gotchas

These are real current behaviors or mismatches that future sessions should keep in mind.

### 1. Startup and normal playback do not enable motors

- Motors start disabled.
- `smoke`, normal tuning, and `calibrate` do not enable them.
- Only explicit `enable` commands or `mapfrets` currently enable motors in normal operator workflows.

### 2. Motor stepping ignores the enabled flag

- `Motor_Update()` still toggles STEP pins and advances software positions even if the driver is disabled.
- This means software can believe a move happened while the real motor did not.
- Combined with the startup behavior above, this is a major source of bench confusion.

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
- `fft_processing.c` still names its correction factor `TIM6_CALIBRATION_FACTOR` even though the live trigger is `TIM1`.

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
- a real CM7 tuning controller
- a real LUT-builder and calibration framework
- a partially commissioned hardware path with several stale comments and a few important behavioral traps

If a future session needs bench-realistic assumptions, use this stricter model:

- Motors that exist in code and are wired enough to target: `0-3`
- Solenoids that exist in code and can actually generate PWM: `1-3`
- Playback path that best matches current commissioned strings: the D3/G3 `smoke` workflow
- Commissioning path that best matches current hardware: `mapfrets` on strings `0-3`

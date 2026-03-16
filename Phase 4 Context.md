# Phase 4 Context

Last updated: 2026-03-16

## Scope And Source Of Truth

This document describes the current `Self-Playing-Guitar` codebase as it exists in this repository today.

- Source of truth: current project code in `CM7/App`, `CM4/App`, `Shared`, the generated startup/build files that directly affect runtime, and the current top-level build files.
- Older markdown docs such as `PROJECT_CONTEXT.md`, `TUNING_CONTROLLER_CHANGES.md`, and parts of `MOTOR_CONTROL_API.md` are useful historical references, but they do not fully match the current implementation.
- Vendor libraries under `Drivers/` and other generated support code are not treated as the behavioral source of truth unless they directly define pinmux, clocks, or boot flow used by the project.
- Physical bench population is only partially discoverable from source. This file separates "current firmware support" from "confirmed current commissioned channels" and from "target future system".

If this file conflicts with an older doc, prefer the code and this file.

## Current Setup At A Glance

| Area | Current codebase state | Target / future direction |
| --- | --- | --- |
| Board | STM32H755ZI Nucleo dual-core split across CM7 + CM4 | Same board for bring-up, then likely custom PCB later |
| Strings | 6 logical strings everywhere in shared IDs and tuning logic | Same |
| Steppers | 6 logical motor IDs exist, but only motors `0-3` have GPIO pin assignments and are enabled at startup | Full 6 stepper channels |
| Solenoids | Firmware initializes and controls 6 PWM solenoid channels | Full physical 6-solenoid rollout on hardware |
| Pitch sensing | Live ADC3 + BDMA + FFT pipeline on CM7 | Continue tuning and validate on real instrument |
| Tuning control | Closed-loop FF + PI controller on CM7 using FFT results | Replace placeholder LUTs with measured data, finish full-string commissioning |
| Song playback | Hardcoded "Smoke on the Water" intro using D3 + G3 power chords | More songs and better scheduler / song loading |
| Calibration | Runtime LUT sweep exists and writes RAM-resident FF tables | Persist measured LUTs and calibrate all strings |

## Current System Summary

The current firmware is a dual-core split:

- CM7 handles audio capture orchestration, FFT processing, serial command parsing, song scheduling, closed-loop tuning control, and calibration control.
- CM4 handles the 1 kHz real-time actuator loop, software-generated step pulses for the DRV8825 drivers, PWM solenoid drive, command execution, and status publication back to CM7.
- CM7 and CM4 communicate through AXI SRAM shared memory plus hardware semaphores.

Important current reality:

- The codebase is already beyond the older "future FFT" stage. ADC3, DMA, TIM6 triggering, FFT, and the tuning control hop are active now.
- The codebase is not yet a fully commissioned 6-stepper hardware system. Motors `4` and `5` still have no GPIO assignment in `motor_control.c`.
- The solenoid firmware path is more complete than the stepper hardware path. Six PWM solenoid channels are initialized in firmware, but some comments and config macros still describe an older 3-solenoid stage.

## Repository Map

### Top-level areas

- `CM7/`
  CM7 application logic, DSP, scheduler, calibration, serial console, and ADC runtime.
- `CM4/`
  CM4 actuator runtime: motor control, solenoid control, and command execution.
- `Shared/`
  Shared memory definition, global config macros, command/status flags.
- `Common/`
  Dual-core STM32 startup support.
- `README.md`
  Current build and flashing notes.
- `PROJECT_CONTEXT.md`
  Older context snapshot; not fully current anymore.

### Most important authored files

- `CM7/App/Src/app_cm7.c`
  Top-level CM7 state machine.
- `CM7/App/Src/audio_input.c`
  ADC3 DMA audio buffering with 50 percent overlap.
- `CM7/App/Src/fft_processing.c`
  FFT-based peak detection and polyphonic frequency list generation.
- `CM7/App/Src/tuning_controller.c`
  Closed-loop string tuning logic.
- `CM7/App/Src/calibration_controller.c`
  LUT sweep and runtime feed-forward breakpoint updates.
- `CM7/App/Src/song_scheduler.c`
  Hardcoded song playback schedule.
- `CM7/App/Src/serial_cmd.c`
  UART command parser and operator interface.
- `CM4/App/Src/app_cm4.c`
  CM4 main loop and command execution.
- `CM4/App/Src/motor_control.c`
  Stepper GPIO mapping and motion update loop.
- `CM4/App/Src/solenoid_control.c`
  Multi-timer PWM solenoid driver.
- `Shared/Inc/shared_memory.h`
  Inter-core contract.
- `Shared/Inc/application_config.h`
  Global constants and several current config mismatches.

## Hardware And Timing

### Clocks

Current code-level clock assumptions:

- CM7 system clock: `480 MHz`
- CM4 clock: `240 MHz`
- Solenoid timer peripheral clock: `240 MHz`
- Solenoid timer prescaled clock: `2.4 MHz` via prescaler `99`
- CM4 control loop target: `1 kHz`
- Audio sample rate target: `6000 Hz`
- FFT size: `2048`
- Effective audio processing hop: `1024 / 6000 = 0.1707 s`, about `170 ms`

### Shared memory location

- Shared memory is placed in AXI SRAM and marked non-cacheable from CM7.
- CM7 configures MPU attributes for the shared region.
- CM4 accesses the same region directly and uses HSEM notification for command/status signaling.

## Dual-Core Startup And Runtime Flow

### Boot sequence

1. CM7 performs the normal HAL and clock startup, configures shared memory access, and releases the CM4 boot barrier.
2. CM4 waits on the HSEM boot semaphore until CM7 releases it.
3. CM4 initializes the motor subsystem and enables / zeroes motors `0-3`.
4. CM4 initializes the solenoid subsystem, preloads string frequencies for all 6 solenoid channels, configures PWM output pins, then sets `MOTOR_STATUS_READY`.
5. CM7 waits for `MOTOR_STATUS_READY`.
6. CM7 sends a home command. In current code, "home" means "move configured motors to software position 0", not physical limit-switch homing.
7. CM7 sets motors `0-3` to `1/16` microstepping.
8. CM7 starts UART RX, FFT, tuning controller, ADC calibration, ADC DMA audio capture, and TIM6.
9. CM7 enters `STATE_RUNNING` after a short `STATE_IDLE` handoff.

### Current CM7 states

`app_cm7.c` currently uses:

- `STATE_INIT`
- `STATE_IDLE`
- `STATE_CALIBRATING`
- `STATE_RUNNING`
- `STATE_ERROR`

There is no separate LUT-builder state in the active CM7 state machine.

### Current CM4 loop

CM4 runs a fixed `1 ms` loop that:

1. Reads pending command flags from shared memory.
2. Executes emergency stop, motor commands, and solenoid commands.
3. Updates timed/burst solenoid auto-stop state.
4. Calls `Motor_Update()` for all configured motors.
5. Publishes current motor state, loop timing, and heartbeat back to shared memory.

## Current Hardware Mapping

### Logical string order used across the project

The code consistently uses:

- `0 = E2`
- `1 = A2`
- `2 = D3`
- `3 = G3`
- `4 = B3`
- `5 = E4`

The tuning controller, song scheduler, calibration controller, motor IDs, and solenoid IDs all assume this logical order.

### Current commissioned stepper channels

The code defines 6 logical motors, but only 4 are currently configured in `motor_control.c`.

| Motor ID | Logical string | STEP | DIR | M0 | M1 | M2 | EN | Status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | E2 | `PE10` | `PE11` | `PE12` | `PE13` | `PE14` | `PD11` | Configured |
| 1 | A2 | `PD12` | `PD13` | `PE2` | `PE4` | `PE7` | `PC9` | Configured |
| 2 | D3 | `PA8` | `PC7` | `PC6` | `PC8` | `PA3` | `PC0` | Configured |
| 3 | G3 | `PC3` | `PB1` | `PF10` | `PG12` | `PG14` | `PG6` | Configured |
| 4 | B3 | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Future |
| 5 | E4 | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Not assigned | Future |

Notes:

- Only motors `0-3` are enabled and zeroed in `App_CM4_Main()`.
- Only motors `0-3` are set to `1/16` microstepping in `App_CM7_Main()`.
- `NUM_MOTORS` is still `6`, so many APIs accept `0-5` even though only `0-3` are commissioned.

### Current solenoid PWM channels in firmware

The active firmware path initializes 6 solenoid channels:

| Solenoid ID | Logical string | Timer / channel | Current code pin |
| --- | --- | --- | --- |
| 0 | E2 | `TIM12_CH1` | `PB14` |
| 1 | A2 | `TIM14_CH1` | `PF9` |
| 2 | D3 | `TIM13_CH1` | `PF8` |
| 3 | G3 | `TIM15_CH1` | `PE5` |
| 4 | B3 | `TIM16_CH1` | `PF6` |
| 5 | E4 | `TIM17_CH1` | `PF7` |

Important note:

- Several comments in `solenoid_control.h`, `solenoid_control.c`, and `app_cm4.c` still mention the older `PA7` / `PA6` mapping for solenoids `1` and `2`.
- The active pinmux code in `CM4/Core/Src/stm32h7xx_hal_msp.c` and the direct GPIO configuration in `app_cm4.c` both route those channels to `PF9` and `PF8`.

### Audio input hardware path

Current CM7 audio path:

- ADC instance: `ADC3`
- Analog input: `PC2_C` / `ADC3_INP0`
- DMA: `BDMA_Channel0`
- Trigger timer: `TIM6` TRGO
- Sampling style: circular DMA into a `2048` sample buffer
- Processing window: `2048` samples with `50 percent` overlap

## Shared Memory And Inter-Core Contract

### Core mechanism

The shared contract is defined in `Shared/Inc/shared_memory.h`.

Major groups:

- CM7 to CM4 command flags
- CM7 to CM4 motor command payload
- CM7 to CM4 solenoid command payload
- CM4 to CM7 motor status and diagnostics
- Heartbeats and message counters for health monitoring

### Current command patterns actually used

Common live paths:

- `CMD_HOME_MOTORS`
- `CMD_SET_MICROSTEP`
- `CMD_MOTOR_MOVE_ABS`
- `CMD_MOTOR_MOVE_REL`
- `CMD_SET_MOTOR_ACCEL`
- `CMD_SET_MOTOR_MAXSPEED`
- `CMD_MOTOR_ENABLE`
- `CMD_MOTOR_ZERO`
- `CMD_STOP_SOLENOID`
- `CMD_TRIGGER_SOLENOID`
- `CMD_TRIGGER_SOLENOID_HZ`
- `CMD_EMERGENCY_STOP`

### Current diagnostics published by CM4

CM4 writes:

- motor positions
- motor speeds
- target speeds
- max speed limits
- max acceleration limits
- enabled state
- microstep mode
- moving flags
- aggregate status flags
- heartbeat
- loop rate
- loop overruns
- max observed loop time

## CM7 Subsystem Details

### Serial console and debug UART

The operator console is on the CM7 side:

- UART: board COM port via BSP COM init
- Current baud: `115200`
- `printf()` is redirected to UART
- RX is interrupt-driven and line-buffered
- Command processing is non-blocking inside the CM7 main loop

Main command groups currently implemented:

- motor control: `start`, `speed`, `accel`, `maxspeed`, `move`, `goto`, `microstep`, `enable`, `disable`, `zero`, `home`, `stop`, `status`
- preset management: `save`, `delete`, `presets`
- solenoid control: `sol`, `solstop`, `solstatus`
- system workflows: `demo`, `calibrate`, `smoke`, `help`

Behavior note:

- Manual movement commands require `start`.
- Playback and calibration are request-driven from serial commands and are acted on during the next audio hop.

### Audio buffering

`audio_input.c` currently implements real acquisition, despite older placeholder comments.

Behavior:

- DMA writes into a circular `audio_buffer`.
- Half-transfer and transfer-complete callbacks flag new data.
- `Audio_GetBuffer()` shifts the previous `1024` samples left and appends the newest half-buffer.
- The returned processing block is a full `2048` sample window with `50 percent` overlap.

### FFT processing

`fft_processing.c` is the active pitch-detection frontend.

Current algorithm:

1. Compute block mean and remove DC.
2. Apply a Hanning window.
3. Run CMSIS real FFT.
4. Compute magnitudes.
5. Search roughly `60 Hz` to `440 Hz`.
6. Apply local-maximum detection.
7. Reject "fat" peaks that look smeared by neighboring resonances.
8. Apply parabolic interpolation for sub-bin frequency estimation.
9. Reject harmonics against previously accepted active peaks.
10. Publish accepted peaks into `active_frequencies[]`, `active_magnitudes[]`, and `active_string_count`.

Current role of the FFT:

- It is the active measurement source used by the tuning controller.
- It is also used during calibration to measure the string frequency after each step increment.
- It is printed as a diagnostic during playback.

### Song scheduler

The current scheduler is hardcoded in `song_scheduler.c`.

Current song:

- "Smoke on the Water" intro
- implemented as a sequence of D3 + G3 two-string power chords
- finite note list, no automatic looping

Current scheduler data model:

- one note entry can contain either one string or a two-string chord
- each note stores target frequency and duration in milliseconds

### Tuning controller

The tuning controller is active and FFT-driven.

Current control structure:

1. Read current target note(s) from the song scheduler.
2. Find the closest FFT-detected peak to the target frequency.
3. Use a per-string feed-forward LUT to estimate absolute motor steps.
4. Apply a discrete PI trim after a hold delay following note changes.
5. Apply lock-band hysteresis.
6. Apply a minimum-move threshold to reduce backlash dither.
7. Apply a slew limit before sending a new absolute motor target to CM4.
8. Stop or start solenoid drive based on lock-state transitions.

Current tuning assumptions:

- home position `0` means open string
- positive steps tighten the string and raise pitch
- FF LUTs are arrays of `4` breakpoints per string
- default mechanical bounds are `-50` to `2000` microsteps

Important current limitation:

- The FF LUTs in `tuning_controller.c` are still just startup values until calibration is run.
- Calibration writes them in RAM only; there is no persistence to flash or file.

### Calibration controller

Calibration is active and integrated into the CM7 state machine.

Current flow:

1. Serial `calibrate` command requests calibration for selected strings.
2. On the next audio hop, CM7 enters `STATE_CALIBRATING`.
3. For each selected string, CM7 moves the target motor to `0`.
4. CM7 starts exact-Hz solenoid drive for that string.
5. CM7 waits a configurable number of audio hops for settling.
6. CM7 records FFT-measured frequency versus step position.
7. CM7 advances the motor in `20` microstep increments.
8. When enough sweep data is collected, CM7 writes downsampled breakpoints into `tuning_ff_freq_bp[][]` and `tuning_ff_steps_bp[][]`.
9. CM7 returns to `STATE_RUNNING` when done or on error.

Current calibration characteristics:

- max sweep: `1200` steps
- buffer size: `60` samples per string
- settle time: `4` audio hops between measurements
- minimum data to build a LUT: `TUNING_FF_NUM_POINTS`

## CM4 Subsystem Details

### Motor control

`motor_control.c` is the live stepper implementation.

Current behavior:

- Each configured motor has hardcoded GPIO pins for STEP, DIR, EN, M0, M1, M2.
- STEP pulses are generated in software, not by timer PWM.
- Pulse timing uses the DWT cycle counter for a `3 us` HIGH pulse.
- The update loop runs at `1 kHz`.
- Velocity control ramps current speed toward target speed with acceleration limiting.
- Position control uses a proportional target-speed rule with a braking-distance check to avoid overshoot.

Current homing semantics:

- `Motor_Home()` does not reference a limit switch.
- It simply commands every configured motor to move back to position `0`.

### Solenoid control

`solenoid_control.c` is the live PWM driver for string excitation.

Current capabilities:

- 6 independent timer-backed PWM channels
- string/fret lookup table for open string through fret `7`
- direct duty-cycle adjustment
- explicit start / stop
- timed auto-stop mode
- burst-cycle auto-stop mode
- direct exact-Hz frequency setting for tuning / calibration use

Current frequency model:

- lookup table ARR values are based on `ARR = (2400000 / f) - 1`
- the table covers all 6 strings for frets `0-7`

### Exact-Hz solenoid path

The codebase has a dedicated "drive at exact target frequency" path:

- CM7 calls `InterCore_TriggerSolenoidHz()`
- shared memory stores `solenoid_id`, `solenoid_freq_hz`, duty, mode, time, burst count
- CM4 handles `CMD_TRIGGER_SOLENOID_HZ`
- CM4 calls `Solenoid_SetFrequencyHz()`

This path is intended for:

- keeping a string ringing while tuning is locked
- keeping a string ringing during calibration sweeps

## Current Operational Workflows

### Power-up workflow

On a normal power-up:

1. CM4 initializes first-level actuator control and reports ready.
2. CM7 waits for ready, sends home-to-zero, sets microstepping, and starts the console and audio path.
3. The system reaches `STATE_RUNNING`.
4. The operator can then use serial commands.

### Manual actuator workflow

- Use `start` before manual movement commands.
- Use motor commands for direct testing of configured stepper channels.
- Use `sol` and `solstop` for direct solenoid checks.

### Playback workflow

- Use `smoke`.
- CM7 starts the song scheduler on the next audio hop.
- Each audio hop advances the scheduler and runs the tuning controller.
- CM7 sends absolute motor positions to CM4 and coordinates solenoid drive according to lock state.

### Calibration workflow

- Use `calibrate` with no arguments for all strings or with string IDs for selected strings.
- Calibration pauses normal playback/tuning control and temporarily owns the audio hop.
- Calibration updates RAM feed-forward tables only.

## Build And Flashing

Current build system:

- Root build uses CMake with Ninja.
- There are separate CM7 and CM4 targets.
- The root README still contains the active high-level build and flash flow.

Current build notes from repo:

- build from a top-level `build/` directory with `cmake .. -G "Ninja"`
- then run `cmake --build . -j8`

Current flash addresses in README:

- CM7 image address: `0x08000000`
- CM4 image address: `0x08100000`

## Current Code / Documentation Mismatches

These are important because they explain why older context documents drifted.

### 1. Solenoid count macro is stale

`application_config.h` still says:

- `NUM_SOLENOIDS 3`

But the active solenoid driver, timer init, and CM4 runtime clearly operate on 6 logical solenoids.

### 2. Some solenoid pin comments are stale

Several comments still describe:

- solenoid `1` on `PA7`
- solenoid `2` on `PA6`

The active pinmux and direct GPIO configuration use:

- solenoid `1` on `PF9`
- solenoid `2` on `PF8`

### 3. Audio headers still describe audio as future / placeholder

`audio_input.h` still has placeholder wording even though ADC3 + DMA audio capture is live.

### 4. Goertzel documentation is stale relative to the current build

The codebase still contains `goertzel_processing.c/.h`, and some tuning docs/comments still describe a Goertzel-based measurement path.

Current reality:

- the active runtime path uses FFT outputs
- `CM7/CMakeLists.txt` does not currently compile `goertzel_processing.c`
- `app_cm7.c` does not use Goertzel in the current control hop

### 5. Processing-frequency macro does not match real hop timing

`CM7_AUDIO_PROCESS_FREQ_HZ` is still `100`, but the actual processing hop is tied to `1024` new samples at `6 kHz`, which is about `5.86 Hz`.

### 6. TIM6 calibration comment is stale

`fft_processing.c` still carries a comment that explains `TIM6_CALIBRATION_FACTOR` in terms of an older `64 MHz` clock story. That comment no longer matches the current clock configuration.

### 7. Legacy solenoid trigger fields are stale

`InterCore_TriggerSolenoids()` still writes:

- `solenoid_triggers`
- `solenoid_duration_ms`

But the current CM4 `CMD_TRIGGER_SOLENOID` handler actually reads:

- `solenoid_id`
- `solenoid_string`
- `solenoid_fret`
- `solenoid_duty_cycle`
- `solenoid_mode`
- `solenoid_activation_time_ms`
- `solenoid_burst_cycles`

So the older "bitmask trigger" fields remain in shared memory, but they are not the fields used by the current CM4 trigger implementation.

## Current Known Risks And Probable Bugs

These are not design goals. They are current behaviors or mismatches visible in the code.

### 1. Exact-Hz solenoid actuation is currently gated by an impossible threshold

The exact-Hz trigger path checks `freq > 1e6` in places where normal note frequencies are expected.

That means the intended "drive the solenoid at the target note frequency" path will not activate for real guitar frequencies unless this condition is fixed.

### 2. FFT peak list can overrun `active_frequencies[]`

Current arrays:

- `active_frequencies[NUM_STRINGS]` -> size `6`
- `active_magnitudes[MAX_HARMONICS]` -> size `8`

Current store guard:

- `if (active_string_count < MAX_HARMONICS)`

That means accepted peaks `6` and `7` would overflow `active_frequencies[]`.

### 3. Song restart logic is not fully reset-safe

`SongSched_Tick()` uses a function-local static `first_tick` flag that is not reset by `SongSched_Init()`.

The first playback works, but replay / restart behavior after the first run is not fully clean in the current implementation.

### 4. "Home" is software zero-return, not hardware homing

The system uses position `0` as home, but there is no limit-switch or sensor-backed homing in the current motor implementation.

### 5. Solenoid pinmux is configured in two places

The solenoid PWM pin routing exists both in:

- `CM4/Core/Src/stm32h7xx_hal_msp.c`
- direct register writes in `CM4/App/Src/app_cm4.c`

This works as a bring-up technique, but it is duplicated configuration that can drift.

## Current Files That Are Most Out Of Date

As of this Phase 4 snapshot, the following should be treated carefully:

- `PROJECT_CONTEXT.md`
- `TUNING_CONTROLLER_CHANGES.md`
- `MOTOR_CONTROL_API.md`
- `audio_input.h`
- `solenoid_control.h`
- parts of `solenoid_control.c`
- parts of `app_cm4.c`
- parts of `inter_core_cm7.h`

They still contain useful history, but they should not override the active implementation.

## Recommended Mental Model For This Project Right Now

The cleanest way to think about the current system is:

- a 6-string logical architecture
- a 4-stepper currently commissioned hardware/control path
- a 6-channel solenoid firmware path
- a live CM7 FFT-based sensing loop
- a live CM7 closed-loop tuning controller
- a live CM7 runtime calibration path that updates feed-forward tables in RAM
- a CM4 real-time executor that turns shared-memory commands into step pulses and solenoid PWM

That mental model is more accurate than either:

- "everything is still future"
- or "all 6 strings are fully commissioned already"

## Future Implementations And Obvious Next Steps

These are the most natural next engineering steps implied by the current code.

### Full actuator commissioning

- add GPIO assignments and bring-up for motors `4` and `5`
- enable and validate motors `4` and `5` in CM4 startup
- include motors `4` and `5` in CM7 microstepping initialization
- mechanically validate B3 and E4 tuning travel

### Solenoid rollout cleanup

- confirm the actual physical installed solenoid count on the bench
- make the docs and config macros match the real 6-channel firmware path
- remove stale `PA6` / `PA7` references
- consolidate the pinmux setup into one authoritative place

### Tuning-controller maturity

- fix the exact-Hz trigger path
- replace placeholder FF LUTs with measured calibration data for all commissioned strings
- persist calibrated LUTs instead of keeping them only in RAM
- re-tune PI gains on real hardware

### Sensing and scheduler cleanup

- decide whether Goertzel is dead code or a future alternate estimator
- fix replay / restart state in the scheduler
- generalize the song scheduler beyond the hardcoded Smoke on the Water demo

### Homing and hardware safety

- implement true physical homing if limit switches or another reference sensor are added
- add explicit mechanical soft limits per string instead of shared defaults

## Bottom Line

Phase 4 is a partially commissioned but already integrated closed-loop guitar controller.

What is real today:

- dual-core CM7 / CM4 architecture
- shared-memory command channel
- live audio capture
- live FFT-based pitch detection
- live motor control for 4 configured steppers
- live 6-channel solenoid PWM driver
- live tuning-controller framework
- live calibration workflow

What is still transitional:

- full 6-stepper hardware support
- exact physical bench solenoid rollout
- persistent calibrated LUTs
- cleanup of stale comments, stale config constants, and older docs

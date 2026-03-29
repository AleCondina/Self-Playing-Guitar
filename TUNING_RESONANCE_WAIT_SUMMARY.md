# Tuning Resonance Wait Summary

## Overview

This change updates the CM7 tuning controller so a new note no longer starts feedforward on the same hop as the solenoid burst.

The new runtime sequence is:

`solenoid burst -> 1 hop resonance wait -> feedforward move -> PI after clean FFT + motor done`

The goal is to give the string a short breathing window before motion starts, and to prevent PI from reacting to burst-contaminated FFT data or to a string that is still moving mechanically.

## Key Changes

### `CM7/App/Inc/tuning_controller.h`

- Added `TUNING_RESONANCE_WAIT_HOPS` as a fixed one-hop breathing delay.
- Added `TUNE_STATE_RESONANCE_WAIT` to the tuning state machine.
- Added two new per-string counters:
  - `resonance_wait_hops_remaining`
  - `fft_clear_wait_hops_remaining`
- Updated the header comments to describe the new note-transition flow.

### `CM7/App/Src/tuning_controller.c`

- Moved note-start ownership out of `TuningCtrl_Update()` and into `TuningCtrl_RunHop()`.
- Added helper logic to:
  - detect true new-note transitions
  - determine whether a string can actually fire a solenoid
  - compute the post-burst FFT-clear wait in hop units
  - initialize per-string state for a new note
- New-note behavior now splits into two paths:
  - If a burst is actually fired, the string enters `TUNE_STATE_RESONANCE_WAIT`.
  - If no burst is fired, the string goes directly to `TUNE_STATE_FF_MOVE`.
- `TUNE_STATE_RESONANCE_WAIT` holds the previous motor command until the one-hop breathing delay expires.
- `TUNE_STATE_FF_MOVE` now computes the feedforward target when the state executes, instead of precomputing it during note detection.
- `TUNE_STATE_FF_WAIT` now requires both:
  - the FFT-clear counter to expire
  - `g_shared_memory.motor_is_moving[string_id] == 0`
  before PI can start.
- While `fft_clear_wait_hops_remaining > 0`, FFT measurements are ignored for control and do not overwrite the accepted measurement state.
- Cold-start behavior still defers the first burst, but the note-transition state now starts only when that deferred burst is actually issued.
- Reset/init code was updated to clear the new counters.

### `CM7/App/Src/song_scheduler.c`

- Updated the timing comments so they match the new live controller behavior.
- Replaced outdated comments that described immediate FF timing or delayed note actuation.

## Timing Details

The implementation remains hop-driven.

- Hop period: about `1024 / 6000 = 170.7 ms`
- Resonance wait: `1` hop
- FFT-clear delay: computed from:
  - burst duration
  - full FFT window time
  - hop time

With the current audio settings:

- `72-110 ms` bursts hold PI off for about `3` hops
- `175 ms` bursts hold PI off for about `4` hops

This means short notes can remain mostly feedforward-dominant, which is expected.

## Solenoid Capability Assumption

The resonance-wait path only activates when the controller believes a real burst was fired.

The CM7 logic currently treats string IDs `< NUM_SOLENOIDS` as burst-capable. In the current shared configuration that means the first three strings are treated as having live solenoid hardware.

## Files Changed

- `CM7/App/Inc/tuning_controller.h`
- `CM7/App/Src/tuning_controller.c`
- `CM7/App/Src/song_scheduler.c`

## Build Verification

The CM7 firmware builds successfully after the change:

```sh
cmake --build build --target NUCLEO_H755ZI_CM7
```

## Remaining Hardware Validation

These checks still need to be done on the board:

- Confirm the state sequence on a live note change:
  `RESONANCE_WAIT -> FF_MOVE -> FF_WAIT -> PI_ACTIVE/IN_TUNE`
- Confirm there is no motor target jump on the burst hop.
- Confirm PI stays blocked while `motor_is_moving` is still true.
- Confirm the FFT-clear wait behaves as expected for `72 ms`, `110 ms`, and `175 ms` bursts.
- Confirm short notes, especially the `500 ms` note, behave acceptably as mostly feedforward-only notes.


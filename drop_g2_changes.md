# Drop G2 Tuning & Song0 Rewrite – Change Summary

## Overview
- Added `TUNING_DROP_G2` tuning mode (string 1 tuned from A2 down to G2)
- Rewrote Song0 as Smoke on the Water in key of E, alternating notes between E2 (str0) and G2 (str1)
- Added `dropg2` serial command

## Song0 Note Layout

| # | String | Freq | Fret | Role |
|---|--------|------|------|------|
| 0 | str0 E2 | E2 (82.41) | 0 | root |
| 1 | str1 G2 | G2 (98.00) | 0 | m3 |
| 2 | str0 E2 | A2 (110.00) | 5 | P4 (held) |
| 3 | str0 E2 | E2 (82.41) | 0 | root |
| 4 | str1 G2 | G2 (98.00) | 0 | m3 |
| 5 | str1 G2 | Bb2 (116.54) | 3 | tritone (short) |
| 6 | str0 E2 | A2 (110.00) | 5 | P4 (held) |
| 7 | str0 E2 | E2 (82.41) | 0 | root |
| 8 | str1 G2 | G2 (98.00) | 0 | m3 |
| 9 | str0 E2 | A2 (110.00) | 5 | P4 (held) |
| 10 | str1 G2 | G2 (98.00) | 0 | m3 |
| 11 | str0 E2 | E2 (82.41) | 0 | root (end) |

Timing: 1635ms standard, 1435ms short (#5), 1885ms held (#2,6,9,11). Same as Song1.

## Files Changed

| File | What |
|------|------|
| `Shared/Inc/application_config.h` | `TUNING_DROP_G2 = 2u`, `NUM_TUNINGS = 3u` |
| `CM7/App/Src/song_scheduler.c` | Rewrote `song0_notes[]` (12 alternating single notes) |
| `CM7/App/Src/calibration_controller.c` | Added G2 rows to `s_open_freq`, `cal_theoretical_freq` |
| `CM7/App/Src/tuning_controller.c` | Added `s_dropg2_ff_freq/steps_bp` LUTs, `s_open_string_hz` row, `SetTuning` branch |
| `CM7/App/Src/lut_builder.c` | Added G2 row to `s_open_freq`, `s_string_names` |
| `CM4/App/Src/solenoid_control.c` | Added `SOLENOID_FREQ_TABLE_DROP_G2`, `SetTuning` branch |
| `CM4/App/Inc/solenoid_control.h` | Updated `Solenoid_SetTuning` docstring |
| `CM7/App/Src/serial_cmd.c` | Added `dropg2` command + `Cmd_DropG2()` handler |

## Usage
1. Flash both CM7 and CM4
2. Send `dropg2` over serial to switch tuning
3. Run `lutbuild` to calibrate string 1 (G2) — placeholder LUT values until calibrated
4. Play song 0

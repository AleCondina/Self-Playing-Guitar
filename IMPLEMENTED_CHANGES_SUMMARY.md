# Implemented Changes Summary

Date: 2026-03-15

This document summarizes the code and configuration changes made in the current workspace update. It covers two main areas:

1. Power and clock reconfiguration to run the board in direct SMPS mode at 400 MHz.
2. Solenoid PWM cleanup so all 6 solenoids can run independently with one timer per output.

No files were renamed. The files below were modified in place.

## Final System State

### Power and clocks

- Supply mode: `PWR_DIRECT_SMPS_SUPPLY`
- CM7 clock: `400 MHz`
- CM4 clock: `200 MHz`
- Timer kernel clock used by the solenoid PWM code: `200 MHz`
- Solenoid timer count frequency after prescaler: `2.0 MHz`

### Solenoid timer map

Each solenoid now uses its own timer instance, so frequency changes are independent:

| Solenoid | Timer | Channel | Pin |
| --- | --- | --- | --- |
| 0 | `TIM12` | `CH1` | `PB14` |
| 1 | `TIM14` | `CH1` | `PA7` |
| 2 | `TIM13` | `CH1` | `PA6` |
| 3 | `TIM15` | `CH1` | `PE5` |
| 4 | `TIM16` | `CH1` | `PB8` |
| 5 | `TIM17` | `CH1` | `PB9` |

## Why These Changes Were Needed

### 1. Direct SMPS at 400 MHz

The board was previously configured for a 480 MHz setup that requires LDO operation. To stay in SMPS mode cleanly, the project had to be moved to a valid direct-SMPS operating point:

- `400 MHz` on CM7
- `200 MHz` on CM4
- `VOS1` instead of `VOS0`
- updated PLL and bus clocks

This avoids the LDO-only 480 MHz path while keeping the project within the STM32H755 operating limits for SMPS mode.

### 2. Independent solenoid PWM outputs

The current repo already used six timer instances, which is the correct architecture for independent PWM frequency control. The actual fault was inconsistent GPIO routing:

- the HAL MSP init routed some timers to `PF` pins
- the application layer manually re-routed pins again
- the documented wiring expected `PA/PB/PE` pins
- `TIM13_CH1` could be routed to more than one GPIO path

That mismatch could make one command appear on the wrong output pin or make behavior look like timer coupling.

The fix was to remove duplicated routing logic and standardize all code and configuration on a single documented timer-to-pin map.

## File-by-File Change List

| File | Change | Why |
| --- | --- | --- |
| `CM7/Core/Src/main.c` | Switched runtime power configuration to direct SMPS, changed voltage scaling to `VOS1`, reduced PLL1 from `N=60` to `N=50`, and updated flash latency. Added a helper to validate the supply configuration. | Makes the CM7 clock tree valid for direct SMPS operation at 400 MHz. |
| `Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.c` | Updated the early boot power-path register writes so reset startup forces clean LDO or direct-SMPS regulator states instead of partially leaving old bits behind. | Prevents unsafe or ambiguous regulator state during early boot before normal clock init. |
| `Shared/Inc/application_config.h` | Updated shared clock constants from `480/240/2.4 MHz` to `400/200/2.0 MHz`. | Keeps timing calculations, delays, and timer math aligned with the new SMPS-safe clocks. |
| `CM4/mx-generated.cmake` | Changed the CubeMX-generated supply define from `USE_PWR_LDO_SUPPLY` to `USE_PWR_DIRECT_SMPS_SUPPLY`. | Keeps the CM4 build configuration aligned with the runtime power mode. |
| `CM7/mx-generated.cmake` | Changed the CubeMX-generated supply define from `USE_PWR_LDO_SUPPLY` to `USE_PWR_DIRECT_SMPS_SUPPLY`. | Keeps the CM7 build configuration aligned with the runtime power mode. |
| `NUCLEO_H755ZI.ioc` | Updated the clock tree values for 400 MHz direct SMPS operation, changed the supply source, adjusted `TIM6.Period` for the new timing base, and added explicit PWM pin assignments for the six solenoid outputs. | Makes the `.ioc` reflect the current intended hardware and clock configuration used by the code. |
| `CM4/App/Src/motor_control.c` | Updated the `Motor_DelayUs()` comment so it matches the new clock-dependent behavior. | Prevents stale documentation after the CM4 clock changed from 240 MHz to 200 MHz. |
| `CM4/Core/Src/main.c` | Updated timer comments and timer init comments to the final solenoid pin map, and updated prescaler comments to the 2.0 MHz timer count base. | Keeps the generated timer setup documentation consistent with the actual routing and timing. |
| `CM4/Core/Src/stm32h7xx_hal_msp.c` | Re-routed PWM GPIO init/deinit so the six timer outputs use `PB14`, `PA7`, `PA6`, `PE5`, `PB8`, and `PB9`. | Makes the HAL layer own the correct physical routing for all six solenoid outputs. |
| `CM4/App/Src/app_cm4.c` | Removed the manual GPIO alternate-function remap block, updated debug comments, and aligned the solenoid pin references to the final map. | Eliminates duplicated pin-routing logic that could override CubeMX/HAL setup and create wrong or duplicated outputs. |
| `CM4/App/Src/solenoid_control.c` | Updated pin references, recalculated the solenoid ARR lookup table for a 2.0 MHz timer base, and corrected frequency calculation comments. | Preserves the intended musical frequencies after the timer clock changed from 2.4 MHz to 2.0 MHz. |
| `CM4/App/Inc/solenoid_control.h` | Updated the documented pin map and timer math notes to match the final routing and 2.0 MHz count frequency. | Keeps the public driver documentation accurate. |
| `PROJECT_CONTEXT.md` | Updated the architecture description, timer clock description, and issue history to reflect the 400 MHz SMPS configuration and the timer-routing cleanup. | Keeps project-level documentation aligned with the current codebase. |

## Important Solenoid-Specific Notes

### What was wrong before

- Solenoid 1 was documented on `PA7`, but code routed `TIM14_CH1` to `PF9`.
- Solenoid 2 was documented on `PA6`, but the code path also allowed `TIM13_CH1` to appear elsewhere.
- Solenoids 4 and 5 were documented on `PB8` and `PB9`, but code routed them to `PF6` and `PF7`.
- The application code manually configured PWM pins after initialization, which created a second routing authority separate from HAL/CubeMX.

### What is correct now

- Each solenoid has one timer instance.
- Each timer uses `CH1` only.
- Each timer output is routed to one intended GPIO pin.
- The application no longer reprograms timer GPIOs after init.

## Verification Performed

The project was rebuilt after the changes:

```bash
cmake -S . -B build -G Ninja
cmake --build build -j8
```

Build status:

- CM4: success
- CM7: success

## Recommended Next Hardware Check

If you still observe one solenoid affecting another after this software cleanup, the next likely causes are hardware rather than timer sharing:

- harness miswiring between MCU pins and MOSFET inputs
- missing or weak gate pull-downs
- flyback or layout issues
- shared ground or supply coupling under load

## Modified Files

For quick reference, these are the files changed in this workspace update:

- `CM4/App/Inc/solenoid_control.h`
- `CM4/App/Src/app_cm4.c`
- `CM4/App/Src/motor_control.c`
- `CM4/App/Src/solenoid_control.c`
- `CM4/Core/Src/main.c`
- `CM4/Core/Src/stm32h7xx_hal_msp.c`
- `CM4/mx-generated.cmake`
- `CM7/Core/Src/main.c`
- `CM7/mx-generated.cmake`
- `Common/Src/system_stm32h7xx_dualcore_boot_cm4_cm7.c`
- `NUCLEO_H755ZI.ioc`
- `PROJECT_CONTEXT.md`
- `Shared/Inc/application_config.h`

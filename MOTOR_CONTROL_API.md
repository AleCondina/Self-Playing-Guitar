# Motor Control API - Usage Guide

This document describes the motor control API for the self-playing guitar project.

## Table of Contents
- [Serial Command Workflow](#serial-command-workflow)
- [Overview](#overview)
- [Pin Configuration](#pin-configuration)
- [API Functions](#api-functions)
- [Usage Examples](#usage-examples)
- [Integration Guide](#integration-guide)

---

## Serial Command Workflow

Commands are sent via UART (USART3, 115200 baud) and flow through the system as follows:

```
You type "speed 0 1000\r"
    |
    v
USART3 RX pin (PD9) receives bytes one at a time
    |
    v
USART3_IRQHandler() -> HAL_UART_IRQHandler() -> HAL_UART_RxCpltCallback()
    |  accumulates chars, sets rx_line_ready=1 on Enter
    v
SerialCmd_Process() [CM7 main loop, every 1ms]
    |  parses "speed", "0", "1000"
    v
InterCore_SendMotorSpeed(0, 1000) [CM7]
    |  writes to shared memory + sets flag + releases HSEM
    v
InterCore_ReadCommands() -> Motor_SetSpeed(0, 1000) [CM4, 1kHz loop]
    |  sets target_speed = 1000
    v
Motor_Update() [CM4, every 1ms]
    |  ramps current_speed toward 1000 (limited by max_accel)
    |  generates STEP pulses on PE10 via accumulator
    v
DRV8825 driver -> stepper motor spins
```

### Startup Sequence

On boot, the system automatically homes all motors (zeroes positions). You must type `start` before movement commands will be accepted:

```
1. System powers on → CM4 initializes motors → CM7 sends home command
2. All motor positions zeroed (home established)
3. You configure parameters: enable 0, accel 0 500, maxspeed 0 5000
4. You type: start
5. Now movement commands work: speed 0 1000, move 0 200, goto E2
```

### Available Serial Commands

```
start                      Start system (required after homing)
speed <id> <steps/s>       Set motor speed (negative = reverse) *
accel <id> <value>         Set max acceleration (steps/s^2)
maxspeed <id> <value>      Set max speed limit (steps/s)
move <id> <steps>          Relative move (negative = reverse) *
goto <id> <position>       Absolute position move *
goto <preset_name>         Move to saved preset position *
microstep <id> <0-5>       Set microstep mode (full/half/quarter/8th/16th/32nd)
enable <id>                Enable motor driver
disable <id>               Disable motor driver
zero <id>                  Zero position counter
home                       Re-home all motors (zeroes positions, requires 'start' again)
save <name> [id]           Save current position as preset (default motor 0)
presets                    List all saved presets
stop                       Emergency stop all motors
status                     Print all motor states (includes degrees from home)
help                       Show command list

* = requires 'start' first
```

### Preset Positions

Save and recall named positions for quick access:

```
> enable 0
> start
> move 0 200              Move to desired position
> save E2                  Save motor 0's current position as "E2"
> move 0 -200             Move back
> goto E2                  Recall — motor 0 moves to saved "E2" position
> presets                  List all saved presets with degrees from home
```

Up to 16 presets can be stored. Presets persist until reboot. Use `save <name> <id>` to save a specific motor's position (e.g., `save fret5 1` saves motor 1's position as "fret5").

### Key Files in the Command Chain

| Layer | File | Role |
|-------|------|------|
| UART RX | `CM7/App/Src/debug_uart.c` | Interrupt-driven byte reception, line buffering |
| IRQ Handler | `CM7/Core/Src/stm32h7xx_it.c` | Routes USART3 interrupt to HAL |
| Command Parser | `CM7/App/Src/serial_cmd.c` | Parses text commands, dispatches handlers |
| Inter-Core Send | `CM7/App/Src/inter_core_cm7.c` | Writes to shared memory, notifies CM4 |
| Shared Memory | `Shared/Inc/shared_memory.h` | Command flags + parameter fields |
| Inter-Core Recv | `CM4/App/Src/app_cm4.c` | Reads commands, calls Motor API |
| Motor Control | `CM4/App/Src/motor_control.c` | Acceleration limiting, step pulse generation |

---

## Overview

The motor control system provides:
- **Stepper motor control** with acceleration limiting
- **Two control modes**: Position control and velocity control
- **Real-time step pulse generation** using Bresenham-style accumulator
- **Pre-configured GPIO pins** (PE10-PE13 for 2 motors)

### Current Pin Configuration

Pin assignments are **hardcoded** in `motor_control.c` for simplicity:

| Motor | STEP Pin | DIR Pin | Status |
|-------|----------|---------|--------|
| 0     | PE10     | PE11    | Configured |
| 1     | PE12     | PE13    | Configured |
| 2-5   | —        | —       | Not configured (future expansion) |

**To add more motors:** Modify `Motor_Init()` in `motor_control.c` to add GPIO configuration for additional motors.

---

## Data Structure

### MotorState_t
```c
typedef struct {
    int32_t target_speed;       /* Target speed in steps/sec */
    int32_t current_speed;      /* Current speed in steps/sec */
    int32_t position;           /* Current position in steps */
    uint8_t is_moving;          /* 1 if motor is moving */
    uint8_t error_flags;        /* Error flags for this motor */
} MotorState_t;
```

---

## API Functions

### Initialization

#### Motor_Init()
```c
int32_t Motor_Init(void);
```
Initialize motor control system and configure GPIO pins (PE10-PE13).

**Returns:** 0 on success, -1 on error

---

### Control Functions

#### Motor_SetSpeed()
```c
void Motor_SetSpeed(uint8_t motor_id, int32_t speed);
```
Set target speed (velocity control mode).

**Parameters:**
- `motor_id`: Motor index (0-5)
- `speed`: Steps/sec (positive = forward, negative = reverse, 0 = stop)

**Example:**
```c
Motor_SetSpeed(0, 5000);   // Move forward at 5000 steps/sec
Motor_SetSpeed(0, -3000);  // Reverse at 3000 steps/sec
Motor_SetSpeed(0, 0);      // Stop (with deceleration)
```

---

#### Motor_MoveToPosition()
```c
int32_t Motor_MoveToPosition(uint8_t motor_id, int32_t position);
```
Move to absolute position (position control mode).

**Example:**
```c
// Move to fret 5 (assuming 800 steps per fret)
Motor_MoveToPosition(0, 5 * 800);
```

---

#### Motor_MoveRelative()
```c
int32_t Motor_MoveRelative(uint8_t motor_id, int32_t steps);
```
Move by relative number of steps.

**Example:**
```c
Motor_MoveRelative(0, 1600);  // Move forward 1600 steps
Motor_MoveRelative(0, -800);  // Move back 800 steps
```

---

#### Motor_ZeroPosition()
```c
void Motor_ZeroPosition(uint8_t motor_id);
```
Reset position counter to zero (typically after homing).

---

#### Motor_SetLimits()
```c
int32_t Motor_SetLimits(uint8_t motor_id, int32_t max_speed, int32_t max_accel);
```
Set custom speed and acceleration limits.

**Example:** `Motor_SetLimits(0, 10000, 2500);`

---

#### Motor_Enable() / Motor_Disable()
```c
int32_t Motor_Enable(uint8_t motor_id);
int32_t Motor_Disable(uint8_t motor_id);
```
Enable or disable motor (currently no ENABLE pin configured).

---

#### Motor_EmergencyStop()
```c
void Motor_EmergencyStop(void);
```
Immediately stop all motors.

---

### Query Functions

#### Motor_GetState()
```c
MotorState_t* Motor_GetState(uint8_t motor_id);
```
Get current motor state (position, speed, moving status).

**Example:**
```c
MotorState_t* state = Motor_GetState(0);
if (state != NULL) {
    int32_t current_position = state->position;
    uint8_t is_moving = state->is_moving;
}
```

---

#### Motor_IsConfigured()
```c
uint8_t Motor_IsConfigured(uint8_t motor_id);
```
Check if motor has been configured with GPIO pins.

**Returns:** 1 if configured, 0 if not

---

### Update Function

#### Motor_Update()
```c
void Motor_Update(MotorState_t states[NUM_MOTORS]);
```
Update motor control loop (called every 1ms). **This generates the step pulses!**

**Important:** This must be called regularly in the control loop.

---

## Usage Examples

### Example 1: Play a Note (Position Control)

```c
/* Move to fret position to play a note */
void play_note(uint8_t string, uint8_t fret)
{
    const int32_t STEPS_PER_FRET = 800;  // Calibrate this value

    int32_t target_position = fret * STEPS_PER_FRET;
    Motor_MoveToPosition(string, target_position);

    /* Wait for motor to reach position */
    MotorState_t* state = Motor_GetState(string);
    while (state != NULL && state->is_moving) {
        /* Motor_Update() is called in main control loop */
    }
}
```

### Example 2: Continuous Movement (Velocity Control)

```c
/* Move motor at constant speed for tuning/calibration */
void calibrate_motor(uint8_t motor_id)
{
    /* Move slowly forward */
    Motor_SetSpeed(motor_id, 1000);  // 1000 steps/sec

    HAL_Delay(2000);  // Move for 2 seconds

    /* Stop */
    Motor_SetSpeed(motor_id, 0);
}
```

### Example 3: Check Motor Status

```c
void monitor_motors(void)
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (Motor_IsConfigured(i)) {
            MotorState_t* state = Motor_GetState(i);
            if (state != NULL) {
                printf("Motor %d: pos=%ld, speed=%ld, moving=%d\n",
                       i, state->position, state->current_speed, state->is_moving);
            }
        }
    }
}
```

---

## Integration Guide

### Control Loop Integration

In `CM4/App/Src/app_cm4.c`, `Motor_Update()` is called in the 1kHz control loop:

```c
void App_CM4_Main(void)
{
    Motor_Init();  // Initialize motors

    while (1) {
        /* Update motors (generates step pulses) */
        Motor_Update(Motor_GetState(0));

        /* ... rest of control loop ... */
    }
}
```

### Adding More Motors

To add motors 2-5, modify `Motor_Init()` in `motor_control.c`:

```c
int32_t Motor_Init(void)
{
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Add for additional ports

    /* Motor 0: PE10/PE11 (already configured) */
    /* Motor 1: PE12/PE13 (already configured) */

    /* Add Motor 2: Example with PD0/PD1 */
    motor_configs[2].pins.step_port = GPIOD;
    motor_configs[2].pins.step_pin = GPIO_PIN_0;
    motor_configs[2].pins.dir_port = GPIOD;
    motor_configs[2].pins.dir_pin = GPIO_PIN_1;
    motor_configs[2].max_speed = MOTOR_MAX_STEP_FREQ;
    motor_configs[2].max_accel = MOTOR_MAX_ACCEL;
    motor_configs[2].configured = 1;

    Motor_InitGPIO(GPIOD, GPIO_PIN_0);
    Motor_InitGPIO(GPIOD, GPIO_PIN_1);

    return 0;
}
```

---

## Hardware Notes

### Step Pulse Timing
- Pulse width: ~40ns (10 NOPs @ 240 MHz)
- Compatible with A4988, DRV8825, TMC2208 drivers

### Control Loop
- Update rate: 1 kHz (1ms period)
- Maximum practical step rate: ~10 kHz per motor

### Acceleration Limiting
- Default max velocity: `MOTOR_MAX_STEP_FREQ = 10000` steps/sec (from application_config.h)
- Default max acceleration: `MOTOR_MAX_ACCEL = 1000` steps/sec^2 (from application_config.h)
- Both can be changed at runtime via `maxspeed` / `accel` serial commands or `Motor_SetLimits()`
- Prevents motor stalling and missed steps

---

## Troubleshooting

### Motor not moving
1. Check if motor is configured: `Motor_IsConfigured(motor_id)`
2. Verify `Motor_Update()` is being called in control loop
3. Check GPIO connections

### Motor moves wrong direction
1. Swap STEP and DIR pins in configuration
2. Or swap motor coil wires physically

### Motor stalls or skips steps
1. Reduce speed using `Motor_SetLimits()`
2. Increase motor driver current
3. Check mechanical load

---

## API Summary

| Function | Purpose | Mode |
|----------|---------|------|
| `Motor_Init()` | Initialize system | Setup |
| `Motor_SetSpeed()` | Constant velocity | Velocity |
| `Motor_MoveToPosition()` | Move to absolute position | Position |
| `Motor_MoveRelative()` | Move by relative steps | Position |
| `Motor_ZeroPosition()` | Reset position counter | Position |
| `Motor_SetLimits()` | Adjust speed/accel limits | Config |
| `Motor_GetState()` | Query current state | Query |
| `Motor_IsConfigured()` | Check if motor ready | Query |
| `Motor_Update()` | Generate step pulses | Control |
| `Motor_EmergencyStop()` | Stop all motors | Safety |

---

## See Also

- [motor_control.h](CM4/App/Inc/motor_control.h) - Header file
- [motor_control.c](CM4/App/Src/motor_control.c) - Implementation
- [motor_config_example.h](CM4/App/Inc/motor_config_example.h) - Usage examples
- [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md) - Overall project documentation

---

**Last Updated:** 2026-02-13
**Version:** 4.0

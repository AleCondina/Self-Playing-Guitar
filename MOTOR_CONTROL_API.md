# Motor Control API - Custom Configuration Guide

This document describes the enhanced motor control API for the self-playing guitar project, supporting 6 independently configurable stepper motors.

## Table of Contents
- [Overview](#overview)
- [Key Features](#key-features)
- [Configuration Structures](#configuration-structures)
- [API Functions](#api-functions)
- [Usage Examples](#usage-examples)
- [Integration Guide](#integration-guide)

---

## Overview

The motor control system provides:
- **6 independently configurable stepper motors** (one per guitar string)
- **Custom pin assignments** (STEP, DIR, ENABLE pins)
- **Flexible step resolution** (configurable step angle and microstepping)
- **Individual speed and acceleration limits** per motor
- **Two control modes**: Position control and velocity control
- **Real-time step pulse generation** using Bresenham-style accumulator

---

## Key Features

### 1. Custom Pin Configuration
Each motor can use any available GPIO pins for:
- **STEP pin**: Generates step pulses
- **DIR pin**: Controls direction (forward/reverse)
- **ENABLE pin**: Enables/disables driver (optional)

### 2. Configurable Step Resolution
- **Step angle**: 0.9°, 1.8°, or custom
- **Microstepping**: 1, 2, 4, 8, 16, or 32x
- **Effective resolution**: Automatically calculated (steps_per_rev × microsteps)

### 3. Individual Motor Limits
Each motor can have custom:
- Maximum speed (steps/sec)
- Maximum acceleration (steps/sec²)
- Direction inversion flag

### 4. Dual Control Modes

#### Position Control Mode
- Move to absolute position: `Motor_MoveToPosition(motor_id, steps)`
- Move relative: `Motor_MoveRelative(motor_id, steps)`
- Automatic speed calculation with proportional control
- Automatic stop when target reached

#### Velocity Control Mode
- Constant speed: `Motor_SetSpeed(motor_id, speed)`
- Smooth acceleration/deceleration
- Bidirectional (positive = forward, negative = reverse)

---

## Configuration Structures

### MotorPinConfig_t
```c
typedef struct {
    GPIO_TypeDef* step_port;    /* STEP pin GPIO port */
    uint16_t step_pin;          /* STEP pin number */
    GPIO_TypeDef* dir_port;     /* DIR pin GPIO port */
    uint16_t dir_pin;           /* DIR pin number */
    GPIO_TypeDef* enable_port;  /* ENABLE pin GPIO port (optional) */
    uint16_t enable_pin;        /* ENABLE pin number (0 if unused) */
} MotorPinConfig_t;
```

### MotorConfig_t
```c
typedef struct {
    MotorPinConfig_t pins;      /* Pin configuration */
    uint16_t steps_per_rev;     /* Steps per revolution (e.g., 200, 400) */
    uint8_t microsteps;         /* Microstepping factor (1, 2, 4, 8, 16, 32) */
    float step_angle;           /* Step angle in degrees (e.g., 1.8°) */
    int32_t max_speed;          /* Maximum speed in steps/sec */
    int32_t max_accel;          /* Maximum acceleration in steps/sec² */
    uint8_t invert_dir;         /* 1 to invert direction pin logic */
} MotorConfig_t;
```

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

### Configuration Functions

#### Motor_Configure()
```c
int32_t Motor_Configure(uint8_t motor_id, const MotorConfig_t* config);
```
Configure a motor with custom parameters. Must be called before using the motor.

**Parameters:**
- `motor_id`: Motor index (0-5)
- `config`: Pointer to configuration structure

**Returns:** 0 on success, -1 on error

---

#### Motor_GetConfig()
```c
const MotorConfig_t* Motor_GetConfig(uint8_t motor_id);
```
Get the current configuration of a motor.

**Returns:** Pointer to config (NULL if not configured)

---

#### Motor_SetStepAngle()
```c
int32_t Motor_SetStepAngle(uint8_t motor_id, float step_angle);
```
Change the step angle (useful when swapping motors).

**Example:** `Motor_SetStepAngle(0, 0.9f);` // Switch to 0.9° motor (400 steps/rev)

---

#### Motor_SetMicrostepping()
```c
int32_t Motor_SetMicrostepping(uint8_t motor_id, uint8_t microsteps);
```
Change microstepping mode (must match driver configuration).

**Valid values:** 1, 2, 4, 8, 16, 32

**Example:** `Motor_SetMicrostepping(0, 32);` // Switch to 32x microstepping

---

#### Motor_SetLimits()
```c
int32_t Motor_SetLimits(uint8_t motor_id, int32_t max_speed, int32_t max_accel);
```
Set custom speed and acceleration limits.

**Example:** `Motor_SetLimits(0, 10000, 2500);`

---

### Control Functions

#### Motor_SetSpeed()
```c
void Motor_SetSpeed(uint8_t motor_id, int32_t speed);
```
Set target speed (velocity control mode).

**Parameters:**
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

#### Motor_Enable()
```c
int32_t Motor_Enable(uint8_t motor_id);
```
Enable motor driver (pulls ENABLE pin LOW).

---

#### Motor_Disable()
```c
int32_t Motor_Disable(uint8_t motor_id);
```
Disable motor driver and stop movement (pulls ENABLE pin HIGH).

---

#### Motor_EmergencyStop()
```c
void Motor_EmergencyStop(void);
```
Immediately stop all motors and disable all drivers.

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
int32_t current_position = state->position;
uint8_t is_moving = state->is_moving;
```

---

#### Motor_IsConfigured()
```c
uint8_t Motor_IsConfigured(uint8_t motor_id);
```
Check if motor has been configured.

**Returns:** 1 if configured, 0 if not

---

#### Motor_GetEffectiveStepsPerRev()
```c
uint32_t Motor_GetEffectiveStepsPerRev(uint8_t motor_id);
```
Get total steps per revolution (steps_per_rev × microsteps).

**Example:** For 200 steps/rev @ 16x microstepping = 3200 steps/rev

---

## Usage Examples

### Example 1: Configure All 6 Motors

```c
#include "motor_control.h"

void setup_motors(void)
{
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure Motor 0 (String E2) */
    MotorConfig_t motor0 = {
        .pins = {
            .step_port = GPIOD,
            .step_pin = GPIO_PIN_12,
            .dir_port = GPIOD,
            .dir_pin = GPIO_PIN_13,
            .enable_port = GPIOD,
            .enable_pin = GPIO_PIN_14
        },
        .steps_per_rev = 200,
        .microsteps = 16,
        .step_angle = 1.8f,
        .max_speed = 8000,
        .max_accel = 2000,
        .invert_dir = 0
    };
    Motor_Configure(0, &motor0);

    /* Configure remaining motors 1-5 similarly... */

    /* Initialize motor control system */
    Motor_Init();

    /* Enable all motors */
    for (uint8_t i = 0; i < 6; i++) {
        Motor_Enable(i);
    }
}
```

### Example 2: Play a Note (Position Control)

```c
/* Move to fret position to play a note */
void play_note(uint8_t string, uint8_t fret)
{
    const int32_t STEPS_PER_FRET = 800;  // Calibrate this value

    int32_t target_position = fret * STEPS_PER_FRET;
    Motor_MoveToPosition(string, target_position);

    /* Wait for motor to reach position */
    while (Motor_GetState(string)->is_moving) {
        /* Motor_Update() is called in main control loop */
    }
}
```

### Example 3: Continuous Movement (Velocity Control)

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

### Example 4: Homing Sequence

```c
/* Home motor to zero position using limit switch */
void home_motor(uint8_t motor_id, GPIO_TypeDef* limit_port, uint16_t limit_pin)
{
    /* Move slowly backward until limit switch triggered */
    Motor_SetSpeed(motor_id, -500);  // Slow reverse

    /* Wait for limit switch (active LOW) */
    while (HAL_GPIO_ReadPin(limit_port, limit_pin) == GPIO_PIN_SET) {
        /* Continue moving */
    }

    /* Stop and zero position */
    Motor_SetSpeed(motor_id, 0);
    HAL_Delay(100);  // Wait for complete stop
    Motor_ZeroPosition(motor_id);

    /* Move slightly away from limit */
    Motor_MoveRelative(motor_id, 100);
}
```

### Example 5: Check Motor Status

```c
void monitor_motors(void)
{
    for (uint8_t i = 0; i < 6; i++) {
        if (Motor_IsConfigured(i)) {
            MotorState_t* state = Motor_GetState(i);

            printf("Motor %d: pos=%ld, speed=%ld, moving=%d\n",
                   i, state->position, state->current_speed, state->is_moving);
        }
    }
}
```

---

## Integration Guide

### Step 1: Enable GPIO Clocks in main.c

Add in `main.c` before calling motor configuration:

```c
/* USER CODE BEGIN 2 */
__HAL_RCC_GPIOD_CLK_ENABLE();
__HAL_RCC_GPIOG_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
/* USER CODE END 2 */
```

### Step 2: Configure Motors

Call your motor configuration function:

```c
/* USER CODE BEGIN 2 */
Motor_ConfigureAll_Example();  // From motor_config_example.h
Motor_Init();
/* USER CODE END 2 */
```

### Step 3: Call Motor_Update() in Control Loop

In `CM4/App/Src/app_cm4.c`, the `Motor_Update()` function is already called in the 1kHz control loop:

```c
void App_CM4_ControlLoop(void)
{
    /* Update motors (this generates step pulses) */
    Motor_Update(motor_states);

    /* ... rest of control loop ... */
}
```

### Step 4: Send Commands from CM7 (Optional)

If using inter-core communication, add motor control fields to `shared_memory.h`:

```c
/* Motor control commands */
volatile uint8_t motor_id;
volatile int32_t motor_target_position;
volatile int32_t motor_target_speed;
volatile uint8_t motor_command;  // 0=none, 1=set_speed, 2=move_to_pos
```

---

## Hardware Considerations

### Step Pulse Timing
- Minimum pulse width: 1-2 µs (most drivers)
- Current implementation: ~40ns pulse (10 NOPs @ 240 MHz)
- Compatible with A4988, DRV8825, TMC2208, and similar drivers

### Enable Pin Logic
- Most drivers use **active-LOW enable** (LOW = enabled, HIGH = disabled)
- If your driver uses active-HIGH, modify `Motor_Enable()` and `Motor_Disable()`

### Microstepping Configuration
- **Hardware**: Set MS1, MS2, MS3 pins on driver board
- **Software**: Must match `microsteps` parameter in `MotorConfig_t`
- Common settings:
  - Full step: MS1=L, MS2=L, MS3=L → `microsteps = 1`
  - 1/16 step: MS1=H, MS2=H, MS3=H → `microsteps = 16`

### Current Limiting
- Set driver potentiometer for appropriate motor current
- Typical NEMA 17: 0.4A - 1.7A per phase
- Formula: `Vref = Imax × 8 × Rsense` (check driver datasheet)

---

## Performance Notes

### Control Loop Timing
- Update rate: 1 kHz (1ms period)
- Step pulse generation overhead: ~10-50 µs per active motor
- Maximum step rate per motor: Limited by GPIO toggle speed (~50 kHz practical)

### Position Control
- Uses simple P-controller: `speed = error × 10`
- Gain of 10 provides good balance of speed and stability
- For faster response, increase gain in `Motor_Update()` (line with `position_error * 10`)

### Acceleration Limiting
- Prevents motor stalling and missed steps
- Default: 1000-2000 steps/sec²
- Adjust based on motor load and inertia

---

## Troubleshooting

### Motor not moving
1. Check if motor is configured: `Motor_IsConfigured(motor_id)`
2. Check if motor is enabled: `Motor_Enable(motor_id)`
3. Verify GPIO clocks are enabled
4. Check ENABLE pin polarity (some drivers are active-HIGH)

### Motor moves wrong direction
1. Set `invert_dir = 1` in motor configuration
2. Or swap motor coil wires physically

### Motor stalls or skips steps
1. Reduce `max_speed` or `max_accel`
2. Increase motor current (driver potentiometer)
3. Check mechanical load/friction
4. Reduce microstepping (more torque at lower resolution)

### Jittery movement
1. Increase microstepping for smoother motion
2. Check for loose connections
3. Verify stable power supply

---

## API Summary

| Function | Purpose | Mode |
|----------|---------|------|
| `Motor_Configure()` | Set up motor pins and parameters | Setup |
| `Motor_Init()` | Initialize motor control system | Setup |
| `Motor_Enable()` | Enable motor driver | Control |
| `Motor_SetSpeed()` | Constant velocity movement | Velocity |
| `Motor_MoveToPosition()` | Move to absolute position | Position |
| `Motor_MoveRelative()` | Move by relative steps | Position |
| `Motor_ZeroPosition()` | Reset position counter | Position |
| `Motor_GetState()` | Query current state | Query |
| `Motor_EmergencyStop()` | Stop all motors immediately | Safety |

---

## See Also

- [motor_control.h](CM4/App/Inc/motor_control.h) - Full API reference
- [motor_config_example.h](CM4/App/Inc/motor_config_example.h) - Configuration examples
- [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md) - Overall project documentation

---

**Last Updated:** 2026-01-25
**Version:** 1.0
**Author:** Claude Code

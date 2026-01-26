/**
 ******************************************************************************
 * @file    motor_config_example.h
 * @brief   Example motor configuration for 6-string guitar
 * @note    This example shows how to configure 6 stepper motors with custom
 *          pin assignments, step angles, and microstepping settings
 ******************************************************************************
 */

#ifndef MOTOR_CONFIG_EXAMPLE_H
#define MOTOR_CONFIG_EXAMPLE_H

#include "motor_control.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Example: Configure all 6 motors for self-playing guitar
 * @note  This function demonstrates how to set up 6 motors with different
 *        pin configurations and parameters
 *
 * Pin Assignment Example for STM32H755ZI:
 *
 * Motor 0 (String E2):
 *   STEP:   PD12  DIR: PD13  ENABLE: PD14
 *
 * Motor 1 (String A2):
 *   STEP:   PD15  DIR: PG2   ENABLE: PG3
 *
 * Motor 2 (String D3):
 *   STEP:   PG4   DIR: PG5   ENABLE: PG6
 *
 * Motor 3 (String G3):
 *   STEP:   PG7   DIR: PG8   ENABLE: PC6
 *
 * Motor 4 (String B3):
 *   STEP:   PC7   DIR: PC8   ENABLE: PC9
 *
 * Motor 5 (String E4):
 *   STEP:   PA8   DIR: PA9   ENABLE: PA10
 */
static inline void Motor_ConfigureAll_Example(void)
{
    /* Enable GPIO clocks (add these in main.c or elsewhere before calling) */
    /* __HAL_RCC_GPIOD_CLK_ENABLE(); */
    /* __HAL_RCC_GPIOG_CLK_ENABLE(); */
    /* __HAL_RCC_GPIOC_CLK_ENABLE(); */
    /* __HAL_RCC_GPIOA_CLK_ENABLE(); */

    /* ===== Motor 0 Configuration (String E2) ===== */
    MotorConfig_t motor0_cfg = {
        .pins = {
            .step_port = GPIOD,
            .step_pin = GPIO_PIN_12,
            .dir_port = GPIOD,
            .dir_pin = GPIO_PIN_13,
            .enable_port = GPIOD,
            .enable_pin = GPIO_PIN_14
        },
        .steps_per_rev = 200,       /* 1.8° stepper motor */
        .microsteps = 16,            /* 16x microstepping */
        .step_angle = 1.8f,          /* Standard NEMA 17 */
        .max_speed = 8000,           /* 8000 steps/sec */
        .max_accel = 2000,           /* 2000 steps/sec² */
        .invert_dir = 0              /* Normal direction */
    };
    Motor_Configure(0, &motor0_cfg);

    /* ===== Motor 1 Configuration (String A2) ===== */
    MotorConfig_t motor1_cfg = {
        .pins = {
            .step_port = GPIOD,
            .step_pin = GPIO_PIN_15,
            .dir_port = GPIOG,
            .dir_pin = GPIO_PIN_2,
            .enable_port = GPIOG,
            .enable_pin = GPIO_PIN_3
        },
        .steps_per_rev = 200,
        .microsteps = 16,
        .step_angle = 1.8f,
        .max_speed = 8000,
        .max_accel = 2000,
        .invert_dir = 0
    };
    Motor_Configure(1, &motor1_cfg);

    /* ===== Motor 2 Configuration (String D3) ===== */
    MotorConfig_t motor2_cfg = {
        .pins = {
            .step_port = GPIOG,
            .step_pin = GPIO_PIN_4,
            .dir_port = GPIOG,
            .dir_pin = GPIO_PIN_5,
            .enable_port = GPIOG,
            .enable_pin = GPIO_PIN_6
        },
        .steps_per_rev = 200,
        .microsteps = 16,
        .step_angle = 1.8f,
        .max_speed = 8000,
        .max_accel = 2000,
        .invert_dir = 0
    };
    Motor_Configure(2, &motor2_cfg);

    /* ===== Motor 3 Configuration (String G3) ===== */
    MotorConfig_t motor3_cfg = {
        .pins = {
            .step_port = GPIOG,
            .step_pin = GPIO_PIN_7,
            .dir_port = GPIOG,
            .dir_pin = GPIO_PIN_8,
            .enable_port = GPIOC,
            .enable_pin = GPIO_PIN_6
        },
        .steps_per_rev = 200,
        .microsteps = 16,
        .step_angle = 1.8f,
        .max_speed = 8000,
        .max_accel = 2000,
        .invert_dir = 0
    };
    Motor_Configure(3, &motor3_cfg);

    /* ===== Motor 4 Configuration (String B3) ===== */
    MotorConfig_t motor4_cfg = {
        .pins = {
            .step_port = GPIOC,
            .step_pin = GPIO_PIN_7,
            .dir_port = GPIOC,
            .dir_pin = GPIO_PIN_8,
            .enable_port = GPIOC,
            .enable_pin = GPIO_PIN_9
        },
        .steps_per_rev = 200,
        .microsteps = 16,
        .step_angle = 1.8f,
        .max_speed = 8000,
        .max_accel = 2000,
        .invert_dir = 0
    };
    Motor_Configure(4, &motor4_cfg);

    /* ===== Motor 5 Configuration (String E4) ===== */
    MotorConfig_t motor5_cfg = {
        .pins = {
            .step_port = GPIOA,
            .step_pin = GPIO_PIN_8,
            .dir_port = GPIOA,
            .dir_pin = GPIO_PIN_9,
            .enable_port = GPIOA,
            .enable_pin = GPIO_PIN_10
        },
        .steps_per_rev = 200,
        .microsteps = 16,
        .step_angle = 1.8f,
        .max_speed = 8000,
        .max_accel = 2000,
        .invert_dir = 0
    };
    Motor_Configure(5, &motor5_cfg);

    /* Enable all motors */
    for (uint8_t i = 0; i < 6; i++) {
        Motor_Enable(i);
    }
}

/**
 * @brief Example: Configure motors with different microstepping
 * @note  Shows how to use different microstepping for different motors
 */
static inline void Motor_ConfigureMixedMicrostepping_Example(void)
{
    /* Motor 0: Full step (no microstepping) - more torque, less precision */
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
        .microsteps = 1,             /* Full step */
        .step_angle = 1.8f,
        .max_speed = 2000,
        .max_accel = 1000,
        .invert_dir = 0
    };
    Motor_Configure(0, &motor0);

    /* Motor 1: 32x microstepping - maximum precision */
    MotorConfig_t motor1 = {
        .pins = {
            .step_port = GPIOD,
            .step_pin = GPIO_PIN_15,
            .dir_port = GPIOG,
            .dir_pin = GPIO_PIN_2,
            .enable_port = GPIOG,
            .enable_pin = GPIO_PIN_3
        },
        .steps_per_rev = 200,
        .microsteps = 32,            /* Ultra-high precision */
        .step_angle = 1.8f,
        .max_speed = 16000,          /* Higher step rate needed */
        .max_accel = 4000,
        .invert_dir = 0
    };
    Motor_Configure(1, &motor1);
}

/**
 * @brief Example: Runtime motor parameter adjustment
 */
static inline void Motor_RuntimeAdjustment_Example(void)
{
    /* Change microstepping on the fly */
    Motor_SetMicrostepping(0, 8);    /* Switch motor 0 to 8x microstepping */

    /* Adjust speed/acceleration limits */
    Motor_SetLimits(1, 12000, 3000); /* Motor 1: 12000 steps/sec, 3000 accel */

    /* Change step angle (if using different motor) */
    Motor_SetStepAngle(2, 0.9f);     /* Motor 2: 0.9° step angle (400 steps/rev) */
}

/**
 * @brief Example: Position control usage
 */
static inline void Motor_PositionControl_Example(void)
{
    /* Move motor 0 to absolute position (e.g., fret 3) */
    int32_t steps_per_fret = 800;    /* Example: 800 steps between frets */
    Motor_MoveToPosition(0, 3 * steps_per_fret);

    /* Move motor 1 relative (e.g., move up 2 frets) */
    Motor_MoveRelative(1, 2 * steps_per_fret);

    /* Zero position after homing */
    Motor_ZeroPosition(0);

    /* Check if motor reached target (position == target, is_moving == 0) */
    MotorState_t* state = Motor_GetState(0);
    if (!state->is_moving) {
        /* Motor reached target position */
    }
}

/**
 * @brief Example: Velocity control usage
 */
static inline void Motor_VelocityControl_Example(void)
{
    /* Set motor 0 to move at constant speed */
    Motor_SetSpeed(0, 4000);         /* 4000 steps/sec forward */

    /* Reverse direction */
    Motor_SetSpeed(0, -4000);        /* 4000 steps/sec backward */

    /* Stop smoothly (with deceleration) */
    Motor_SetSpeed(0, 0);
}

/**
 * @brief Example: Query motor configuration
 */
static inline void Motor_QueryConfig_Example(void)
{
    /* Check if motor is configured */
    if (Motor_IsConfigured(0)) {
        /* Get configuration */
        const MotorConfig_t* cfg = Motor_GetConfig(0);

        /* Calculate effective resolution */
        uint32_t total_steps_per_rev = Motor_GetEffectiveStepsPerRev(0);
        /* For 200 steps/rev @ 16x microsteps = 3200 steps/rev */

        /* Get current state */
        MotorState_t* state = Motor_GetState(0);
        int32_t current_pos = state->position;
        int32_t current_spd = state->current_speed;
    }
}

#endif /* MOTOR_CONFIG_EXAMPLE_H */

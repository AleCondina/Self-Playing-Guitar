/**
 ******************************************************************************
 * @file    motor_config_example.h
 * @brief   Example motor usage for self-playing guitar
 * @note    This example shows how to use the motor control API
 *
 *          Current Pin Configuration (hardcoded in motor_control.c):
 *          - Motor 0: STEP=PE10, DIR=PE11, M0=PE12, M1=PE13, M2=PE14, EN=PD11
 *          - Motor 1: STEP=PD12, DIR=PD13, M0=PE2,  M1=PE4,  M2=PE7,  EN=PC9 (ZIO CN10-12/10/16/18/20/9)
 *
 *          To add more motors, modify motor_control.c Motor_Init() to configure
 *          additional GPIO pins.
 ******************************************************************************
 */

#ifndef MOTOR_CONFIG_EXAMPLE_H
#define MOTOR_CONFIG_EXAMPLE_H

#include "motor_control.h"

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
    if (state != NULL && !state->is_moving) {
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
 * @brief Example: Query motor state
 */
static inline void Motor_QueryState_Example(void)
{
    /* Check if motor is configured */
    if (Motor_IsConfigured(0)) {
        /* Get current state */
        MotorState_t* state = Motor_GetState(0);
        if (state != NULL) {
            int32_t current_pos = state->position;
            int32_t current_spd = state->current_speed;
            uint8_t is_moving = state->is_moving;

            /* Suppress unused variable warnings in example code */
            (void)current_pos;
            (void)current_spd;
            (void)is_moving;
        }
    }
}

/**
 * @brief Example: Adjust motor limits at runtime
 */
static inline void Motor_AdjustLimits_Example(void)
{
    /* Change speed and acceleration limits for motor 0 */
    Motor_SetLimits(0, 8000, 2000);  /* 8000 steps/sec max, 2000 accel */

    /* Change limits for motor 1 */
    Motor_SetLimits(1, 6000, 1500);  /* 6000 steps/sec max, 1500 accel */
}

/**
 * @brief Example: Emergency stop usage
 */
static inline void Motor_SafetyExample(void)
{
    /* Emergency stop - immediately stops all motors */
    Motor_EmergencyStop();

    /* Disable individual motor */
    Motor_Disable(0);

    /* Re-enable motor when ready */
    Motor_Enable(0);
}

#endif /* MOTOR_CONFIG_EXAMPLE_H */

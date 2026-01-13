/**
 ******************************************************************************
 * @file    motor_control.h
 * @brief   Motor control API for CM4
 * @note    Stepper motor control with acceleration limiting
 ******************************************************************************
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "application_config.h"

/* Motor state structure */
typedef struct {
    int32_t target_speed;       /* Target speed in steps/sec */
    int32_t current_speed;      /* Current speed in steps/sec */
    int32_t position;           /* Current position in steps */
    uint8_t is_moving;          /* 1 if motor is moving */
    uint8_t error_flags;        /* Error flags for this motor */
} MotorState_t;

/**
 * @brief Initialize motor control system
 * @return 0 on success, -1 on error
 * @note  Initializes TIM peripherals (to be configured in CubeMX)
 */
int32_t Motor_Init(void);

/**
 * @brief Update motor control loop (called every 1ms)
 * @param states Array of motor states
 * @note  Implements acceleration limiting and position tracking
 */
void Motor_Update(MotorState_t states[NUM_MOTORS]);

/**
 * @brief Set motor target speed
 * @param motor_id Motor index (0-5)
 * @param speed Target speed in steps/sec (signed for direction)
 */
void Motor_SetSpeed(uint8_t motor_id, int32_t speed);

/**
 * @brief Get motor state
 * @param motor_id Motor index (0-5)
 * @return Pointer to motor state
 */
MotorState_t* Motor_GetState(uint8_t motor_id);

/**
 * @brief Emergency stop all motors
 */
void Motor_EmergencyStop(void);

/**
 * @brief Start homing sequence
 * @note  Placeholder for future implementation
 */
void Motor_Home(void);

#endif /* MOTOR_CONTROL_H */

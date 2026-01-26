/**
 ******************************************************************************
 * @file    motor_control.h
 * @brief   Motor control API for CM4
 * @note    Stepper motor control with acceleration limiting
 *
 *          Hardware Pin Mapping (PE10-PE13 for 3 motors):
 *          - Motor 0: STEP=PE10, DIR=PE11
 *          - Motor 1: STEP=PE12, DIR=PE13
 *          - Motor 2: (future expansion)
 *
 *          Note: For 6 motors, additional GPIO pins will be needed.
 *          Current implementation supports 3 motors with 4 pins (PE10-PE13).
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
 * @note  Configures GPIO pins (PE10-PE13) for motor control
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

/**
 * @brief Set custom speed and acceleration limits for a motor
 * @param motor_id Motor index (0-5)
 * @param max_speed Maximum speed in steps/sec
 * @param max_accel Maximum acceleration in steps/sec²
 * @return 0 on success, -1 on error
 */
int32_t Motor_SetLimits(uint8_t motor_id, int32_t max_speed, int32_t max_accel);

/**
 * @brief Move motor to absolute position
 * @param motor_id Motor index (0-5)
 * @param position Target position in steps
 * @return 0 on success, -1 on error
 */
int32_t Motor_MoveToPosition(uint8_t motor_id, int32_t position);

/**
 * @brief Move motor by relative steps
 * @param motor_id Motor index (0-5)
 * @param steps Number of steps to move (signed)
 * @return 0 on success, -1 on error
 */
int32_t Motor_MoveRelative(uint8_t motor_id, int32_t steps);

/**
 * @brief Zero/reset motor position
 * @param motor_id Motor index (0-5)
 */
void Motor_ZeroPosition(uint8_t motor_id);

/**
 * @brief Enable motor driver
 * @param motor_id Motor index (0-5)
 * @return 0 on success, -1 on error
 */
int32_t Motor_Enable(uint8_t motor_id);

/**
 * @brief Disable motor driver
 * @param motor_id Motor index (0-5)
 * @return 0 on success, -1 on error
 */
int32_t Motor_Disable(uint8_t motor_id);

/**
 * @brief Check if motor is configured
 * @param motor_id Motor index (0-5)
 * @return 1 if configured, 0 if not
 */
uint8_t Motor_IsConfigured(uint8_t motor_id);

#endif /* MOTOR_CONTROL_H */

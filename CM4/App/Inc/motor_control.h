/**
 ******************************************************************************
 * @file    motor_control.h
 * @brief   Motor control API for CM4 with DRV8825 stepper drivers
 * @note    Stepper motor control with acceleration limiting and microstepping
 *
 *          DRV8825 Driver Pin Configuration (6 pins per motor):
 *          - Motor 0: STEP=PE10, DIR=PE11, M0=PE12, M1=PE13, M2=PE14, EN=PD11
 *          - Motor 1: STEP=PD12, DIR=PD13, M0=PE2,  M1=PE4,  M2=PE7,  EN=PC9  (ZIO CN10)
 *          - Motor 2: STEP=PA8,  DIR=PC7,  M0=PC6,  M1=PC8,  M2=PA3,  EN=PC0  (ZIO CN9: D6,D4,D5,D7,A0,A1 - D3)
 *          - Motor 3: STEP=PC3,  DIR=PF3,  M0=PF5,  M1=PF10, M2=PB8,  EN=PB9  (ZIO CN9/CN10: A2,A3,A4,A5,D9,D10 - G3)
 *          - Motors 4-5: Structure ready, pins TBD
 *
 *          Each motor uses: STEP, DIR, EN, M0, M1, M2 (6 pins total)
 *          Total for 6 motors: 36 GPIO pins
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
 * @brief DRV8825 Microstepping modes
 * @note  Values encode M0|M1|M2 pin states as bits [0:2]
 *
 *        Mode bits: [M2:M1:M0]
 *        FULL_STEP     = 0b000 = 0
 *        HALF_STEP     = 0b001 = 1
 *        QUARTER_STEP  = 0b010 = 2
 *        EIGHTH_STEP   = 0b011 = 3
 *        SIXTEENTH     = 0b100 = 4
 *        THIRTYSECOND  = 0b101 = 5
 */
typedef enum {
    MOTOR_MICROSTEP_FULL     = 0,  /* Full step (no microstepping) */
    MOTOR_MICROSTEP_HALF     = 1,  /* 1/2 step */
    MOTOR_MICROSTEP_QUARTER  = 2,  /* 1/4 step */
    MOTOR_MICROSTEP_EIGHTH   = 3,  /* 1/8 step */
    MOTOR_MICROSTEP_16       = 4,  /* 1/16 step */
    MOTOR_MICROSTEP_32       = 5   /* 1/32 step */
} MotorMicrostepMode_t;

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
void Motor_Update(MotorState_t states[NUM_MOTORS]); //array "states" containing all 6 motors states

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

/**
 * @brief Set microstepping mode for DRV8825 driver
 * @param motor_id Motor index (0-5)
 * @param mode Microstepping mode (MotorMicrostepMode_t enum value)
 * @return 0 on success, -1 on error (invalid motor or mode)
 * @note  Changes take effect immediately; motor can be moving
 */
int32_t Motor_SetMicrostepMode(uint8_t motor_id, uint8_t mode);

/**
 * @brief Get motor max speed limit
 * @param motor_id Motor index (0-5)
 * @return Max speed in steps/sec, or 0 if invalid
 */
int32_t Motor_GetMaxSpeed(uint8_t motor_id);

/**
 * @brief Get motor max acceleration limit
 * @param motor_id Motor index (0-5)
 * @return Max accel in steps/sec^2, or 0 if invalid
 */
int32_t Motor_GetMaxAccel(uint8_t motor_id);

/**
 * @brief Get motor enabled state
 * @param motor_id Motor index (0-5)
 * @return 1 if enabled, 0 if disabled or invalid
 */
uint8_t Motor_IsEnabled(uint8_t motor_id);

/**
 * @brief Get motor microstep mode
 * @param motor_id Motor index (0-5)
 * @return Microstep mode (0-5), or 0 if invalid
 */
uint8_t Motor_GetMicrostepMode(uint8_t motor_id);

#endif /* MOTOR_CONTROL_H */

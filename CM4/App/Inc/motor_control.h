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

/* Motor pin configuration structure */
typedef struct {
    GPIO_TypeDef* step_port;    /* STEP pin GPIO port */
    uint16_t step_pin;          /* STEP pin number */
    GPIO_TypeDef* dir_port;     /* DIR pin GPIO port */
    uint16_t dir_pin;           /* DIR pin number */
    GPIO_TypeDef* enable_port;  /* ENABLE pin GPIO port (optional) */
    uint16_t enable_pin;        /* ENABLE pin number (0 if unused) */
} MotorPinConfig_t;

/* Motor hardware configuration structure */
typedef struct {
    MotorPinConfig_t pins;      /* Pin configuration */
    uint16_t steps_per_rev;     /* Steps per revolution (e.g., 200, 400) */
    uint8_t microsteps;         /* Microstepping factor (1, 2, 4, 8, 16, 32) */
    float step_angle;           /* Step angle in degrees (e.g., 1.8° for 200 step motor) */
    int32_t max_speed;          /* Maximum speed in steps/sec for this motor */
    int32_t max_accel;          /* Maximum acceleration in steps/sec² */
    uint8_t invert_dir;         /* 1 to invert direction pin logic */
} MotorConfig_t;

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

/**
 * @brief Configure a single motor with custom parameters
 * @param motor_id Motor index (0-5)
 * @param config Pointer to motor configuration structure
 * @return 0 on success, -1 on error
 * @note  Must be called before Motor_Init() or after for runtime reconfiguration
 */
int32_t Motor_Configure(uint8_t motor_id, const MotorConfig_t* config);

/**
 * @brief Get motor configuration
 * @param motor_id Motor index (0-5)
 * @return Pointer to motor configuration (NULL if invalid ID)
 */
const MotorConfig_t* Motor_GetConfig(uint8_t motor_id);

/**
 * @brief Set custom step angle for a motor
 * @param motor_id Motor index (0-5)
 * @param step_angle Step angle in degrees (e.g., 1.8, 0.9)
 * @return 0 on success, -1 on error
 */
int32_t Motor_SetStepAngle(uint8_t motor_id, float step_angle);

/**
 * @brief Set microstepping mode for a motor
 * @param motor_id Motor index (0-5)
 * @param microsteps Microstepping factor (1, 2, 4, 8, 16, 32)
 * @return 0 on success, -1 on error
 * @note  Updates effective steps per revolution
 */
int32_t Motor_SetMicrostepping(uint8_t motor_id, uint8_t microsteps);

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
 * @brief Enable motor driver (pulls ENABLE pin low if configured)
 * @param motor_id Motor index (0-5)
 * @return 0 on success, -1 on error
 */
int32_t Motor_Enable(uint8_t motor_id);

/**
 * @brief Disable motor driver (pulls ENABLE pin high if configured)
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
 * @brief Get effective steps per revolution (including microstepping)
 * @param motor_id Motor index (0-5)
 * @return Steps per revolution (steps_per_rev * microsteps), or 0 if not configured
 */
uint32_t Motor_GetEffectiveStepsPerRev(uint8_t motor_id);

#endif /* MOTOR_CONTROL_H */

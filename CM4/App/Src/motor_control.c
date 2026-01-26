/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   Motor control implementation
 * @note    Stepper motor control with acceleration limiting (stub)
 ******************************************************************************
 */

#include "motor_control.h"
#include "stm32h7xx_hal.h"

/* Private variables */
static MotorState_t motor_states[NUM_MOTORS] = {0};
static MotorConfig_t motor_configs[NUM_MOTORS] = {0};
static uint8_t motor_configured[NUM_MOTORS] = {0}; /* Track which motors are configured */

/* Target position for position control */
static int32_t target_positions[NUM_MOTORS] = {0};
static uint8_t position_mode[NUM_MOTORS] = {0}; /* 1 if in position control mode */

/* External TIM handles (to be defined in main.c after CubeMX config) */
/* extern TIM_HandleTypeDef htim2, htim3, htim4, htim5; */

/**
 * @brief Initialize motor control system
 */
int32_t Motor_Init(void)
{
    /* Initialize motor states */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_states[i].target_speed = 0;
        motor_states[i].current_speed = 0;
        motor_states[i].position = 0;
        motor_states[i].is_moving = 0;
        motor_states[i].error_flags = 0;
    }

    /* TODO: Start TIM PWM channels when peripherals are configured */
    /* HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); */
    /* HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); */
    /* etc. */

    return 0;
}

/**
 * @brief Update motor control loop (called every 1ms)
 */
void Motor_Update(MotorState_t states[NUM_MOTORS])
{
    static uint32_t step_accumulator[NUM_MOTORS] = {0};
    const uint32_t UPDATE_FREQ_HZ = 1000; /* 1kHz update rate */

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        /* Skip unconfigured motors */
        if (!motor_configured[i]) {
            continue;
        }

        /* Position control mode: calculate speed to reach target */
        if (position_mode[i]) {
            int32_t position_error = target_positions[i] - states[i].position;

            /* Simple proportional control with velocity limiting */
            if (position_error == 0) {
                states[i].target_speed = 0;
                position_mode[i] = 0; /* Exit position mode when target reached */
            } else {
                /* Calculate desired speed (proportional to error, clamped) */
                int32_t desired_speed = position_error * 10; /* P gain = 10 */

                /* Get motor-specific limits */
                int32_t max_spd = motor_configs[i].max_speed > 0 ?
                                 motor_configs[i].max_speed : MOTOR_MAX_STEP_FREQ;

                /* Clamp to motor's maximum speed */
                if (desired_speed > max_spd) {
                    desired_speed = max_spd;
                } else if (desired_speed < -max_spd) {
                    desired_speed = -max_spd;
                }

                states[i].target_speed = desired_speed;
            }
        }

        /* Get motor-specific acceleration limit */
        int32_t max_accel = motor_configs[i].max_accel > 0 ?
                           motor_configs[i].max_accel : MOTOR_MAX_ACCEL;

        /* Acceleration limiting */
        int32_t speed_error = states[i].target_speed - states[i].current_speed;

        /* Clamp acceleration */
        if (speed_error > max_accel) {
            speed_error = max_accel;
        } else if (speed_error < -max_accel) {
            speed_error = -max_accel;
        }

        /* Update current speed */
        states[i].current_speed += speed_error;

        /* Get motor-specific speed limit */
        int32_t max_speed = motor_configs[i].max_speed > 0 ?
                           motor_configs[i].max_speed : MOTOR_MAX_STEP_FREQ;

        /* Clamp to min/max speed */
        if (states[i].current_speed > max_speed) {
            states[i].current_speed = max_speed;
        } else if (states[i].current_speed < -max_speed) {
            states[i].current_speed = -max_speed;
        }

        /* Update direction pin */
        if (motor_configs[i].pins.dir_port != NULL) {
            GPIO_PinState dir_state;
            if (motor_configs[i].invert_dir) {
                dir_state = (states[i].current_speed >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            } else {
                dir_state = (states[i].current_speed >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            }
            HAL_GPIO_WritePin(motor_configs[i].pins.dir_port,
                            motor_configs[i].pins.dir_pin, dir_state);
        }

        /* Generate step pulses using Bresenham-style accumulator */
        uint32_t abs_speed = (states[i].current_speed >= 0) ?
                            states[i].current_speed : -states[i].current_speed;

        if (abs_speed > 0 && motor_configs[i].pins.step_port != NULL) {
            /* Accumulate fractional steps */
            step_accumulator[i] += abs_speed;

            /* Generate steps when accumulator exceeds update frequency */
            while (step_accumulator[i] >= UPDATE_FREQ_HZ) {
                /* Generate step pulse (pulse high then low) */
                HAL_GPIO_WritePin(motor_configs[i].pins.step_port,
                                motor_configs[i].pins.step_pin, GPIO_PIN_SET);

                /* Short delay for step pulse width (minimum 1-2us for most drivers) */
                /* At 240 MHz, each NOP is ~4ns, so 10 NOPs ≈ 40ns (sufficient) */
                __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
                __NOP(); __NOP(); __NOP(); __NOP(); __NOP();

                HAL_GPIO_WritePin(motor_configs[i].pins.step_port,
                                motor_configs[i].pins.step_pin, GPIO_PIN_RESET);

                /* Update position */
                if (states[i].current_speed >= 0) {
                    states[i].position++;
                } else {
                    states[i].position--;
                }

                step_accumulator[i] -= UPDATE_FREQ_HZ;
            }
        } else {
            step_accumulator[i] = 0;
        }

        /* Update is_moving flag */
        states[i].is_moving = (states[i].current_speed != 0);
    }
}

/**
 * @brief Set motor target speed
 */
void Motor_SetSpeed(uint8_t motor_id, int32_t speed)
{
    if (motor_id >= NUM_MOTORS) {
        return;
    }

    /* Use motor-specific speed limit if configured, otherwise use global default */
    int32_t max_speed = (motor_configured[motor_id] && motor_configs[motor_id].max_speed > 0) ?
                        motor_configs[motor_id].max_speed : MOTOR_MAX_STEP_FREQ;

    /* Clamp speed to valid range */
    if (speed > max_speed) {
        speed = max_speed;
    } else if (speed < -max_speed) {
        speed = -max_speed;
    }

    /* Exit position control mode when speed is set manually */
    position_mode[motor_id] = 0;

    motor_states[motor_id].target_speed = speed;
}

/**
 * @brief Get motor state
 */
MotorState_t* Motor_GetState(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) {
        return NULL;
    }
    return &motor_states[motor_id];
}

/**
 * @brief Emergency stop all motors
 */
void Motor_EmergencyStop(void)
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_states[i].target_speed = 0;
        motor_states[i].current_speed = 0;
        motor_states[i].is_moving = 0;
        position_mode[i] = 0;

        /* Disable motor drivers if configured */
        if (motor_configured[i] &&
            motor_configs[i].pins.enable_port != NULL &&
            motor_configs[i].pins.enable_pin != 0) {
            HAL_GPIO_WritePin(motor_configs[i].pins.enable_port,
                            motor_configs[i].pins.enable_pin, GPIO_PIN_SET);
        }
    }
}

/**
 * @brief Start homing sequence
 */
void Motor_Home(void)
{
    /* TODO: Implement homing sequence with limit switches */
}

/**
 * @brief Configure a single motor with custom parameters
 */
int32_t Motor_Configure(uint8_t motor_id, const MotorConfig_t* config)
{
    if (motor_id >= NUM_MOTORS || config == NULL) {
        return -1;
    }

    /* Validate configuration */
    if (config->step_angle <= 0.0f || config->step_angle > 360.0f) {
        return -1;
    }

    if (config->microsteps == 0 ||
        (config->microsteps != 1 && config->microsteps != 2 &&
         config->microsteps != 4 && config->microsteps != 8 &&
         config->microsteps != 16 && config->microsteps != 32)) {
        return -1;
    }

    /* Copy configuration */
    motor_configs[motor_id] = *config;
    motor_configured[motor_id] = 1;

    /* Initialize GPIO pins */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure STEP pin */
    if (config->pins.step_port != NULL) {
        GPIO_InitStruct.Pin = config->pins.step_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(config->pins.step_port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(config->pins.step_port, config->pins.step_pin, GPIO_PIN_RESET);
    }

    /* Configure DIR pin */
    if (config->pins.dir_port != NULL) {
        GPIO_InitStruct.Pin = config->pins.dir_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(config->pins.dir_port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(config->pins.dir_port, config->pins.dir_pin,
                         config->invert_dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /* Configure ENABLE pin (if provided) */
    if (config->pins.enable_port != NULL && config->pins.enable_pin != 0) {
        GPIO_InitStruct.Pin = config->pins.enable_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(config->pins.enable_port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(config->pins.enable_port, config->pins.enable_pin,
                         GPIO_PIN_SET); /* Active low - disabled by default */
    }

    return 0;
}

/**
 * @brief Get motor configuration
 */
const MotorConfig_t* Motor_GetConfig(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return NULL;
    }
    return &motor_configs[motor_id];
}

/**
 * @brief Set custom step angle for a motor
 */
int32_t Motor_SetStepAngle(uint8_t motor_id, float step_angle)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return -1;
    }

    if (step_angle <= 0.0f || step_angle > 360.0f) {
        return -1;
    }

    motor_configs[motor_id].step_angle = step_angle;

    /* Recalculate steps per revolution */
    motor_configs[motor_id].steps_per_rev = (uint16_t)(360.0f / step_angle);

    return 0;
}

/**
 * @brief Set microstepping mode for a motor
 */
int32_t Motor_SetMicrostepping(uint8_t motor_id, uint8_t microsteps)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return -1;
    }

    /* Validate microstepping value */
    if (microsteps != 1 && microsteps != 2 && microsteps != 4 &&
        microsteps != 8 && microsteps != 16 && microsteps != 32) {
        return -1;
    }

    motor_configs[motor_id].microsteps = microsteps;

    return 0;
}

/**
 * @brief Set custom speed and acceleration limits for a motor
 */
int32_t Motor_SetLimits(uint8_t motor_id, int32_t max_speed, int32_t max_accel)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return -1;
    }

    if (max_speed <= 0 || max_accel <= 0) {
        return -1;
    }

    motor_configs[motor_id].max_speed = max_speed;
    motor_configs[motor_id].max_accel = max_accel;

    return 0;
}

/**
 * @brief Move motor to absolute position
 */
int32_t Motor_MoveToPosition(uint8_t motor_id, int32_t position)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return -1;
    }

    target_positions[motor_id] = position;
    position_mode[motor_id] = 1;

    return 0;
}

/**
 * @brief Move motor by relative steps
 */
int32_t Motor_MoveRelative(uint8_t motor_id, int32_t steps)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return -1;
    }

    target_positions[motor_id] = motor_states[motor_id].position + steps;
    position_mode[motor_id] = 1;

    return 0;
}

/**
 * @brief Zero/reset motor position
 */
void Motor_ZeroPosition(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) {
        return;
    }

    motor_states[motor_id].position = 0;
    target_positions[motor_id] = 0;
    position_mode[motor_id] = 0;
}

/**
 * @brief Enable motor driver
 */
int32_t Motor_Enable(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return -1;
    }

    /* Pull ENABLE pin low (most drivers are active-low) */
    if (motor_configs[motor_id].pins.enable_port != NULL &&
        motor_configs[motor_id].pins.enable_pin != 0) {
        HAL_GPIO_WritePin(motor_configs[motor_id].pins.enable_port,
                         motor_configs[motor_id].pins.enable_pin, GPIO_PIN_RESET);
    }

    return 0;
}

/**
 * @brief Disable motor driver
 */
int32_t Motor_Disable(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return -1;
    }

    /* Pull ENABLE pin high (disable) */
    if (motor_configs[motor_id].pins.enable_port != NULL &&
        motor_configs[motor_id].pins.enable_pin != 0) {
        HAL_GPIO_WritePin(motor_configs[motor_id].pins.enable_port,
                         motor_configs[motor_id].pins.enable_pin, GPIO_PIN_SET);
    }

    /* Stop motor movement */
    motor_states[motor_id].target_speed = 0;
    motor_states[motor_id].current_speed = 0;
    motor_states[motor_id].is_moving = 0;

    return 0;
}

/**
 * @brief Check if motor is configured
 */
uint8_t Motor_IsConfigured(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) {
        return 0;
    }
    return motor_configured[motor_id];
}

/**
 * @brief Get effective steps per revolution
 */
uint32_t Motor_GetEffectiveStepsPerRev(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configured[motor_id]) {
        return 0;
    }

    return (uint32_t)motor_configs[motor_id].steps_per_rev *
           (uint32_t)motor_configs[motor_id].microsteps;
}

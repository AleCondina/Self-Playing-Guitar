/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   Motor control implementation
 * @note    Stepper motor control with acceleration limiting
 *
 *          GPIO Pin Configuration:
 *          - Motor 0: STEP=PE10, DIR=PE11
 *          - Motor 1: STEP=PE12, DIR=PE13
 *          - Motors 2-5: Not configured (future expansion)
 ******************************************************************************
 */

#include "motor_control.h"
#include "stm32h7xx_hal.h"

/* Number of motors currently configured with GPIO pins */
#define NUM_CONFIGURED_MOTORS   2

/* Motor pin configuration (internal) */
typedef struct {
    GPIO_TypeDef* step_port;    /* STEP pin GPIO port */
    uint16_t step_pin;          /* STEP pin number */
    GPIO_TypeDef* dir_port;     /* DIR pin GPIO port */
    uint16_t dir_pin;           /* DIR pin number */
} MotorPinConfig_t;

/* Motor hardware configuration (internal) */
typedef struct {
    MotorPinConfig_t pins;      /* Pin configuration */
    int32_t max_speed;          /* Maximum speed in steps/sec */
    int32_t max_accel;          /* Maximum acceleration in steps/sec² */
    uint8_t configured;         /* 1 if motor is configured */
} MotorConfig_t;

/* Private variables */
static MotorState_t motor_states[NUM_MOTORS] = {0};
static MotorConfig_t motor_configs[NUM_MOTORS] = {0};

/* Target position for position control */
static int32_t target_positions[NUM_MOTORS] = {0};
static uint8_t position_mode[NUM_MOTORS] = {0}; /* 1 if in position control mode */

/**
 * @brief Initialize GPIO for motor control
 * @param port GPIO port
 * @param pin GPIO pin number
 */
static void Motor_InitGPIO(GPIO_TypeDef* port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/**
 * @brief Initialize motor control system
 */
int32_t Motor_Init(void)
{
    /* Enable GPIOE clock for PE10-PE13 */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Initialize motor states */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_states[i].target_speed = 0;
        motor_states[i].current_speed = 0;
        motor_states[i].position = 0;
        motor_states[i].is_moving = 0;
        motor_states[i].error_flags = 0;
        motor_configs[i].configured = 0;
    }

    /* Configure Motor 0: STEP=PE10, DIR=PE11 */
    motor_configs[0].pins.step_port = GPIOE;
    motor_configs[0].pins.step_pin = GPIO_PIN_10;
    motor_configs[0].pins.dir_port = GPIOE;
    motor_configs[0].pins.dir_pin = GPIO_PIN_11;
    motor_configs[0].max_speed = MOTOR_MAX_STEP_FREQ;
    motor_configs[0].max_accel = MOTOR_MAX_ACCEL;
    motor_configs[0].configured = 1;

    Motor_InitGPIO(GPIOE, GPIO_PIN_10);  /* STEP */
    Motor_InitGPIO(GPIOE, GPIO_PIN_11);  /* DIR */

    /* Configure Motor 1: STEP=PE12, DIR=PE13 */
    motor_configs[1].pins.step_port = GPIOE;
    motor_configs[1].pins.step_pin = GPIO_PIN_12;
    motor_configs[1].pins.dir_port = GPIOE;
    motor_configs[1].pins.dir_pin = GPIO_PIN_13;
    motor_configs[1].max_speed = MOTOR_MAX_STEP_FREQ;
    motor_configs[1].max_accel = MOTOR_MAX_ACCEL;
    motor_configs[1].configured = 1;

    Motor_InitGPIO(GPIOE, GPIO_PIN_12);  /* STEP */
    Motor_InitGPIO(GPIOE, GPIO_PIN_13);  /* DIR */

    /* Motors 2-5 not configured yet (no GPIO pins assigned) */
    /* Future: Add more GPIO pins for additional motors */

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
        if (!motor_configs[i].configured) {
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
            GPIO_PinState dir_state = (states[i].current_speed >= 0) ?
                                      GPIO_PIN_SET : GPIO_PIN_RESET;
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
    int32_t max_speed = (motor_configs[motor_id].configured && motor_configs[motor_id].max_speed > 0) ?
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
 * @brief Set custom speed and acceleration limits for a motor
 */
int32_t Motor_SetLimits(uint8_t motor_id, int32_t max_speed, int32_t max_accel)
{
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
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
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
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
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
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
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
        return -1;
    }

    /* Note: No ENABLE pin configured currently */
    /* Future: Add enable pin support if needed */

    return 0;
}

/**
 * @brief Disable motor driver
 */
int32_t Motor_Disable(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
        return -1;
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
    return motor_configs[motor_id].configured;
}

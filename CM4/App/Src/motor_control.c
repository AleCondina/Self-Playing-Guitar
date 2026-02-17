/**
 ******************************************************************************
 * @file    motor_control.c
 * @brief   Motor control implementation for DRV8825 stepper drivers
 * @note    Stepper motor control with acceleration limiting
 *
 *          DRV8825 Driver Pin Configuration (6 pins per motor):
 *          - Motor 0: STEP=PE10, DIR=PE11, M0=PE12, M1=PE13, M2=PE14, EN=PD11
 *          - Motors 1-5: Not configured (structure ready, pins TBD)
 *
 *          Microstepping Modes (M0, M1, M2):
 *          - 0,0,0 = Full step
 *          - 1,0,0 = 1/2 step
 *          - 0,1,0 = 1/4 step
 *          - 1,1,0 = 1/8 step
 *          - 0,0,1 = 1/16 step
 *          - 1,0,1 = 1/32 step
 ******************************************************************************
 */

#include "motor_control.h"
#include "stm32h7xx_hal.h"

/* Number of motors currently configured with GPIO pins */
#define NUM_CONFIGURED_MOTORS   1

/* Step pulse width in microseconds (DRV8825 minimum: 1.9us) */
#define STEP_PULSE_US           3

/* Motor pin configuration for DRV8825 driver (internal) */
typedef struct {
    GPIO_TypeDef* step_port;    /* STEP pin GPIO port */
    uint16_t step_pin;          /* STEP pin number */
    GPIO_TypeDef* dir_port;     /* DIR pin GPIO port */
    uint16_t dir_pin;           /* DIR pin number */
    GPIO_TypeDef* en_port;      /* ENABLE pin GPIO port (active LOW) */
    uint16_t en_pin;            /* ENABLE pin number */
    GPIO_TypeDef* m0_port;      /* M0 microstep GPIO port */
    uint16_t m0_pin;            /* M0 microstep pin */
    GPIO_TypeDef* m1_port;      /* M1 microstep GPIO port */
    uint16_t m1_pin;            /* M1 microstep pin */
    GPIO_TypeDef* m2_port;      /* M2 microstep GPIO port */
    uint16_t m2_pin;            /* M2 microstep pin */
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

/* Runtime state tracking */
static uint8_t motor_enabled_state[NUM_MOTORS] = {0};
static uint8_t motor_microstep_mode[NUM_MOTORS] = {0};

/**
 * @brief Microsecond delay using DWT cycle counter
 * @param us Delay in microseconds
 * @note  Requires DWT to be enabled (done in Motor_Init)
 */
static void Motor_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (CM4_CLOCK_MHZ); /* 240 cycles per us at 240 MHz */
    while ((DWT->CYCCNT - start) < cycles);
}

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
    /* Enable DWT cycle counter for microsecond-precision step pulse timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Enable GPIO clocks for PE10-PE14 and PD11 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Initialize motor states and clear all pin configs */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        motor_states[i].target_speed = 0;
        motor_states[i].current_speed = 0;
        motor_states[i].position = 0;
        motor_states[i].is_moving = 0;
        motor_states[i].error_flags = 0;
        motor_configs[i].configured = 0;

        /* Clear all pin pointers for unconfigured motors */
        motor_configs[i].pins.step_port = NULL;
        motor_configs[i].pins.dir_port = NULL;
        motor_configs[i].pins.en_port = NULL;
        motor_configs[i].pins.m0_port = NULL;
        motor_configs[i].pins.m1_port = NULL;
        motor_configs[i].pins.m2_port = NULL;
    }

    /* ================================================================
     * Configure Motor 0: DRV8825 driver
     * STEP=PE10, DIR=PE11, M0=PE12, M1=PE13, M2=PE14, EN=PD11
     * ================================================================ */

    /* STEP pin - PE10 */
    motor_configs[0].pins.step_port = GPIOE;
    motor_configs[0].pins.step_pin = GPIO_PIN_10;
    Motor_InitGPIO(GPIOE, GPIO_PIN_10);

    /* DIR pin - PE11 */
    motor_configs[0].pins.dir_port = GPIOE;
    motor_configs[0].pins.dir_pin = GPIO_PIN_11;
    Motor_InitGPIO(GPIOE, GPIO_PIN_11);

    /* M0 microstep pin - PE12 */
    motor_configs[0].pins.m0_port = GPIOE;
    motor_configs[0].pins.m0_pin = GPIO_PIN_12;
    Motor_InitGPIO(GPIOE, GPIO_PIN_12);

    /* M1 microstep pin - PE13 */
    motor_configs[0].pins.m1_port = GPIOE;
    motor_configs[0].pins.m1_pin = GPIO_PIN_13;
    Motor_InitGPIO(GPIOE, GPIO_PIN_13);

    /* M2 microstep pin - PE14 */
    motor_configs[0].pins.m2_port = GPIOE;
    motor_configs[0].pins.m2_pin = GPIO_PIN_14;
    Motor_InitGPIO(GPIOE, GPIO_PIN_14);

    /* EN pin - PD11 (active LOW, start HIGH = disabled) */
    motor_configs[0].pins.en_port = GPIOD;
    motor_configs[0].pins.en_pin = GPIO_PIN_11;
    Motor_InitGPIO(GPIOD, GPIO_PIN_11);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  /* Start disabled */

    /* Set motor parameters */
    motor_configs[0].max_speed = MOTOR_MAX_STEP_FREQ;
    motor_configs[0].max_accel = MOTOR_MAX_ACCEL;
    motor_configs[0].configured = 1;

    /* ================================================================
     * Motors 1-5: Structure ready, pins not assigned
     * To configure, assign ports/pins and set configured = 1
     * ================================================================ */

    return 0;
}

/**
 * @brief Update motor control loop (called every 1ms)
 */
void Motor_Update(MotorState_t states[NUM_MOTORS])
{
    static uint32_t step_accumulator[NUM_MOTORS] = {0};
    static int32_t accel_accumulator[NUM_MOTORS] = {0};
    const uint32_t UPDATE_FREQ_HZ = 1000; /* 1kHz update rate */

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        /* Skip unconfigured motors */
        if (!motor_configs[i].configured) {
            continue;
        }

        /* Get motor-specific limits (used by position control and accel limiter) */
        int32_t max_speed = motor_configs[i].max_speed > 0 ?
                           motor_configs[i].max_speed : MOTOR_MAX_STEP_FREQ;
        int32_t max_accel = motor_configs[i].max_accel > 0 ?
                           motor_configs[i].max_accel : MOTOR_MAX_ACCEL;

        /* Position control mode: P controller with braking distance check
         * The P controller (gain=10) sets the target speed proportional to
         * the distance remaining. A braking check prevents overshoot by
         * comparing stopping distance (v²/2a) against remaining distance.
         * When the motor can't stop in time, it decelerates to 0. */
        if (position_mode[i]) {
            int32_t position_error = target_positions[i] - states[i].position;
            int32_t abs_error = (position_error >= 0) ? position_error : -position_error;
            int32_t abs_speed = (states[i].current_speed >= 0) ?
                                states[i].current_speed : -states[i].current_speed;

            if (abs_error == 0 && abs_speed == 0) {
                /* At target AND fully stopped — exit position mode */
                states[i].target_speed = 0;
                position_mode[i] = 0;
            } else {
                /* Check stopping distance: v² / (2*a) */
                int32_t stop_dist = (int32_t)((int64_t)abs_speed * abs_speed
                                    / (2 * max_accel));

                if (abs_error == 0 || stop_dist >= abs_error) {
                    /* At target or can't stop in time — decelerate */
                    states[i].target_speed = 0;
                } else {
                    /* P controller: accelerate toward target */
                    int32_t desired_speed = position_error * 10; /* P gain = 10 */

                    /* Clamp to motor's maximum speed */
                    if (desired_speed > max_speed) {
                        desired_speed = max_speed;
                    } else if (desired_speed < -max_speed) {
                        desired_speed = -max_speed;
                    }

                    states[i].target_speed = desired_speed;
                }
            }
        }

        /* Acceleration limiting using Bresenham accumulator
         * max_accel is in steps/s². At 1kHz update rate, the per-tick
         * speed change must be max_accel / 1000. Since this is often
         * fractional, we accumulate and apply integer changes. */
        int32_t speed_error = states[i].target_speed - states[i].current_speed;

        if (speed_error != 0) {
            /* Accumulate fractional acceleration */
            accel_accumulator[i] += max_accel;

            /* Calculate integer speed change for this tick */
            int32_t max_delta = accel_accumulator[i] / (int32_t)UPDATE_FREQ_HZ;
            accel_accumulator[i] %= (int32_t)UPDATE_FREQ_HZ;

            /* Apply speed change toward target */
            if (speed_error > 0) {
                if (speed_error <= max_delta) {
                    states[i].current_speed = states[i].target_speed;
                    accel_accumulator[i] = 0;
                } else {
                    states[i].current_speed += max_delta;
                }
            } else {
                if (-speed_error <= max_delta) {
                    states[i].current_speed = states[i].target_speed;
                    accel_accumulator[i] = 0;
                } else {
                    states[i].current_speed -= max_delta;
                }
            }
        } else {
            sol [i] = 0;
        }

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

                /* DRV8825 requires minimum 1.9us HIGH pulse on STEP */
                Motor_DelayUs(STEP_PULSE_US);

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

        /* Update is_moving flag: moving if speed is non-zero OR still in position mode */
        states[i].is_moving = (states[i].current_speed != 0) || position_mode[i];
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
    /* Move all configured motors back to position 0 (the home position) */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (motor_configs[i].configured) {
            Motor_MoveToPosition(i, 0);
        }
    }
}

/**
 * @brief Set custom speed and acceleration limits for a motor
 */
int32_t Motor_SetLimits(uint8_t motor_id, int32_t max_speed, int32_t max_accel)
{
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
        return -1;
    }

    /* 0 means "keep current value" - allows updating one without the other */
    if (max_speed > 0) {
        motor_configs[motor_id].max_speed = max_speed;
    }
    if (max_accel > 0) {
        motor_configs[motor_id].max_accel = max_accel;
    }

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
    motor_states[motor_id].is_moving = 1;  /* Mark moving immediately to avoid race */

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
    motor_states[motor_id].is_moving = 1;  /* Mark moving immediately to avoid race */

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
 * @brief Enable motor driver (DRV8825 EN pin active LOW)
 */
int32_t Motor_Enable(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
        return -1;
    }

    /* Drive EN pin LOW to enable DRV8825 driver */
    if (motor_configs[motor_id].pins.en_port != NULL) {
        HAL_GPIO_WritePin(motor_configs[motor_id].pins.en_port,
                          motor_configs[motor_id].pins.en_pin, GPIO_PIN_RESET);
    }
    motor_enabled_state[motor_id] = 1;

    return 0;
}

/**
 * @brief Disable motor driver (DRV8825 EN pin active LOW)
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

    /* Drive EN pin HIGH to disable DRV8825 driver */
    if (motor_configs[motor_id].pins.en_port != NULL) {
        HAL_GPIO_WritePin(motor_configs[motor_id].pins.en_port,
                          motor_configs[motor_id].pins.en_pin, GPIO_PIN_SET);
    }
    motor_enabled_state[motor_id] = 0;

    return 0;
}

/**
 * @brief Set microstepping mode for DRV8825 driver
 * @param motor_id Motor index (0-5)
 * @param mode Microstepping mode (see MotorMicrostepMode_t enum)
 * @return 0 on success, -1 on error
 *
 * DRV8825 Microstepping Truth Table:
 * M0 | M1 | M2 | Resolution
 *  0 |  0 |  0 | Full step
 *  1 |  0 |  0 | 1/2 step
 *  0 |  1 |  0 | 1/4 step
 *  1 |  1 |  0 | 1/8 step
 *  0 |  0 |  1 | 1/16 step
 *  1 |  0 |  1 | 1/32 step
 */
int32_t Motor_SetMicrostepMode(uint8_t motor_id, uint8_t mode)
{
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) {
        return -1;
    }

    /* Check if microstep pins are configured */
    if (motor_configs[motor_id].pins.m0_port == NULL ||
        motor_configs[motor_id].pins.m1_port == NULL ||
        motor_configs[motor_id].pins.m2_port == NULL) {
        return -1;
    }

    /* Extract M0, M1, M2 bits from mode */
    GPIO_PinState m0_state = (mode & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    GPIO_PinState m1_state = (mode & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    GPIO_PinState m2_state = (mode & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    /* Set microstep pins */
    HAL_GPIO_WritePin(motor_configs[motor_id].pins.m0_port,
                      motor_configs[motor_id].pins.m0_pin, m0_state);
    HAL_GPIO_WritePin(motor_configs[motor_id].pins.m1_port,
                      motor_configs[motor_id].pins.m1_pin, m1_state);
    HAL_GPIO_WritePin(motor_configs[motor_id].pins.m2_port,
                      motor_configs[motor_id].pins.m2_pin, m2_state);

    motor_microstep_mode[motor_id] = mode;

    return 0;
}

/**
 * @brief Get motor max speed limit
 */
int32_t Motor_GetMaxSpeed(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) return 0;
    return motor_configs[motor_id].max_speed > 0 ?
           motor_configs[motor_id].max_speed : MOTOR_MAX_STEP_FREQ;
}

/**
 * @brief Get motor max acceleration limit
 */
int32_t Motor_GetMaxAccel(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS || !motor_configs[motor_id].configured) return 0;
    return motor_configs[motor_id].max_accel > 0 ?
           motor_configs[motor_id].max_accel : MOTOR_MAX_ACCEL;
}

/**
 * @brief Get motor enabled state
 */
uint8_t Motor_IsEnabled(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) return 0;
    return motor_enabled_state[motor_id];
}

/**
 * @brief Get motor microstep mode
 */
uint8_t Motor_GetMicrostepMode(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) return 0;
    return motor_microstep_mode[motor_id];
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

/**
 ******************************************************************************
 * @file    app_cm4.c
 * @brief   CM4 application implementation
 * @note    Main control loop for CM4 (motor control and solenoid timing)
 ******************************************************************************
 */

#include "app_cm4.h"
#include "inter_core_cm4.h"
#include "motor_control.h"
#include "solenoid_control.h"
#include "shared_memory.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_hsem.h"
#include "stm32h7xx_hal_rcc.h"

/* External timer handles for debug access (6 independent timers) - All on ZIO connectors */
extern TIM_HandleTypeDef htim12;  /* Solenoid 0: E2 (PB14) - ZIO CN12-28 */
extern TIM_HandleTypeDef htim13;  /* Solenoid 2: D3 (PF8)  - ZIO CN9-24 */
extern TIM_HandleTypeDef htim14;  /* Solenoid 1: A2 (PF9)  - ZIO CN9-28 */
extern TIM_HandleTypeDef htim15;  /* Solenoid 3: G3 (PE5)  - ZIO CN10-8 */
extern TIM_HandleTypeDef htim16;  /* Solenoid 4: B3 (PF6)  - ZIO CN10-7 */
extern TIM_HandleTypeDef htim17;  /* Solenoid 5: E4 (PF7)  - ZIO CN9-26 */

/* Control loop timing */
#define CONTROL_LOOP_PERIOD_MS  1   /* 1ms = 1kHz control loop */

/* Motor test parameters (fallback defaults if shared memory not set) */
#define STEPS_PER_REV               200     /* Standard stepper: 200 steps/rev (1.8° per step) */
#define DEFAULT_STEPS_FOR_180_DEG   400     /* Default steps per move */
/* Application state */
static uint32_t loop_start_time = 0;
static uint32_t loop_count = 0;
static uint32_t last_rate_calc_time = 0;

/**
 * @brief CM4 application main function
 */
void App_CM4_Main(void)
{
    uint32_t commands = 0;
    MotorState_t *motor_states = NULL;

    __HAL_RCC_HSEM_CLK_ENABLE();

    // wait at the barrier until CM7 is done initiating 
    while (HAL_HSEM_FastTake(HSEM_ID_BOOT) != HAL_OK){
    }
    // Release BOOT semaphore to continue
    HAL_HSEM_Release(HSEM_ID_BOOT, 0);

    /* Initialize motor control system */
    Motor_Init();

    /* Enable motor 0 driver (drive nEN LOW) */
    Motor_Enable(0);

    /* Zero position */
    Motor_ZeroPosition(0);

    /* Enable motor 1 driver (drive nEN LOW) */
    Motor_Enable(1);

    /* Zero position */
    Motor_ZeroPosition(1);

    /* Enable motor 2 driver - D3 string (drive nEN LOW) */
    Motor_Enable(2);

    /* Zero position */
    Motor_ZeroPosition(2);

    /* Enable motor 3 driver - G3 string (drive nEN LOW) */
    Motor_Enable(3);

    /* Zero position */
    Motor_ZeroPosition(3);

    /* Initialize solenoid control system */
    if (Solenoid_Init() != 0) {
        g_shared_memory.motor_errors |= ERROR_INVALID_COMMAND;
    }

    /* TEST: Initialize all 6 solenoids with their respective string frequencies */
    /* Each solenoid now has its own independent timer for independent frequency control */

    /* Set frequencies for all 6 strings (open position, fret 0) */
    Solenoid_SetFrequency(0, 0, 0);  // Solenoid 0: E2 string (Low E), 82.41 Hz on PB14
    Solenoid_SetFrequency(1, 1, 0);  // Solenoid 1: A2 string, 110.00 Hz on PA7
    Solenoid_SetFrequency(2, 2, 0);  // Solenoid 2: D3 string, 146.83 Hz on PA6
    Solenoid_SetFrequency(3, 3, 0);  // Solenoid 3: G3 string, 196.00 Hz on PE5
    Solenoid_SetFrequency(4, 4, 0);  // Solenoid 4: B3 string, 246.94 Hz on PF6
    Solenoid_SetFrequency(5, 5, 0);  // Solenoid 5: E4 string (High E), 329.63 Hz on PF7

    /* Start all 6 solenoids */
    // Solenoid_Start(0);
    // Solenoid_Start(1);
    // Solenoid_Start(2);
    // Solenoid_Start(3);
    // Solenoid_Start(4);
    // Solenoid_Start(5);

    /* Direct GPIO configuration for all PWM pins */
    /* Enable all required GPIO clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Enable all timer clocks */
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    /* Configure PB14 as AF2 (TIM12_CH1) - Solenoid 0: E2 - ZIO CN12-28 */
    GPIOB->MODER &= ~(0x3UL << (14 * 2));
    GPIOB->MODER |= (0x2UL << (14 * 2));
    GPIOB->AFR[1] &= ~(0xFUL << ((14-8) * 4));
    GPIOB->AFR[1] |= (0x2UL << ((14-8) * 4));
    GPIOB->OTYPER &= ~(1UL << 14);
    GPIOB->OSPEEDR |= (0x3UL << (14 * 2));

    /* Configure PF9 as AF9 (TIM14_CH1) - Solenoid 1: A2 - ZIO CN9-28 */
    GPIOF->MODER &= ~(0x3UL << (9 * 2));
    GPIOF->MODER |= (0x2UL << (9 * 2));
    GPIOF->AFR[1] &= ~(0xFUL << ((9-8) * 4));
    GPIOF->AFR[1] |= (0x9UL << ((9-8) * 4));
    GPIOF->OTYPER &= ~(1UL << 9);
    GPIOF->OSPEEDR |= (0x3UL << (9 * 2));

    /* Configure PF8 as AF9 (TIM13_CH1) - Solenoid 2: D3 - ZIO CN9-24 */
    GPIOF->MODER &= ~(0x3UL << (8 * 2));
    GPIOF->MODER |= (0x2UL << (8 * 2));
    GPIOF->AFR[1] &= ~(0xFUL << ((8-8) * 4));
    GPIOF->AFR[1] |= (0x9UL << ((8-8) * 4));
    GPIOF->OTYPER &= ~(1UL << 8);
    GPIOF->OSPEEDR |= (0x3UL << (8 * 2));

    /* Configure PE5 as AF4 (TIM15_CH1) - Solenoid 3: G3 - ZIO CN10-8 */
    GPIOE->MODER &= ~(0x3UL << (5 * 2));
    GPIOE->MODER |= (0x2UL << (5 * 2));
    GPIOE->AFR[0] &= ~(0xFUL << (5 * 4));
    GPIOE->AFR[0] |= (0x4UL << (5 * 4));
    GPIOE->OTYPER &= ~(1UL << 5);
    GPIOE->OSPEEDR |= (0x3UL << (5 * 2));

    /* Configure PF6 as AF1 (TIM16_CH1) - Solenoid 4: B3 - ZIO CN10-7 */
    GPIOF->MODER &= ~(0x3UL << (6 * 2));
    GPIOF->MODER |= (0x2UL << (6 * 2));
    GPIOF->AFR[0] &= ~(0xFUL << (6 * 4));
    GPIOF->AFR[0] |= (0x1UL << (6 * 4));
    GPIOF->OTYPER &= ~(1UL << 6);
    GPIOF->OSPEEDR |= (0x3UL << (6 * 2));

    /* Configure PF7 as AF1 (TIM17_CH1) - Solenoid 5: E4 - ZIO CN9-26 */
    GPIOF->MODER &= ~(0x3UL << (7 * 2));
    GPIOF->MODER |= (0x2UL << (7 * 2));
    GPIOF->AFR[0] &= ~(0xFUL << (7 * 4));
    GPIOF->AFR[0] |= (0x1UL << (7 * 4));
    GPIOF->OTYPER &= ~(1UL << 7);
    GPIOF->OSPEEDR |= (0x3UL << (7 * 2));

    /* DEBUG: Read back all configuration */
    volatile uint32_t debug_arr = __HAL_TIM_GET_AUTORELOAD(&htim12);
    volatile uint32_t debug_ccr = __HAL_TIM_GET_COMPARE(&htim12, TIM_CHANNEL_1);
    volatile uint32_t debug_freq = Solenoid_GetFrequency(0);
    volatile uint8_t debug_active = Solenoid_GetActiveMask();

    /* Read TIM12 control registers for verification */
    volatile uint32_t debug_tim12_cr1 = TIM12->CR1;       // Expected: 0x0081 (CEN + ARPE)
    volatile uint32_t debug_tim12_cnt = TIM12->CNT;       // Expected: counting (0 to 7764)
    volatile uint32_t debug_tim12_psc = TIM12->PSC;       // Expected: 99
    volatile uint32_t debug_tim12_arr = TIM12->ARR;       // Expected: 7764
    volatile uint32_t debug_tim12_ccr1 = TIM12->CCR1;     // Expected: 3882
    volatile uint32_t debug_tim12_ccmr1 = TIM12->CCMR1;   // Expected: 0x0068
    volatile uint32_t debug_tim12_ccer = TIM12->CCER;     // Expected: 0x0001

    /* GPIO configuration readback */
    volatile uint32_t debug_gpiob_moder = GPIOB->MODER;   // Check PB14 is in AF mode (bit 28-29 = 10)
    volatile uint32_t debug_gpiob_afrh = GPIOB->AFR[1];   // Check PB14 AF2 setting (bits 24-27 = 0010)
    volatile uint32_t debug_gpiob_otyper = GPIOB->OTYPER; // Should be 0 for PB14 (push-pull)

    /* RCC clock enable status */
    volatile uint32_t debug_rcc_apb1lenr = RCC->APB1LENR; // Bit 6 should be set (TIM12EN)

    /* Prevent compiler optimization */
    (void)debug_arr;
    (void)debug_ccr;
    (void)debug_freq;
    (void)debug_active;
    (void)debug_tim12_cr1;
    (void)debug_tim12_cnt;
    (void)debug_tim12_psc;
    (void)debug_tim12_arr;
    (void)debug_tim12_ccr1;
    (void)debug_tim12_ccmr1;
    (void)debug_tim12_ccer;
    (void)debug_gpiob_moder;
    (void)debug_gpiob_afrh;
    (void)debug_gpiob_otyper;
    (void)debug_rcc_apb1lenr;

    /* Signal ready to CM7 */
    g_shared_memory.motor_status_flags = MOTOR_STATUS_READY;
    InterCore_NotifyCM7();

    /* Initialize control loop timing */
    uint32_t next_tick = HAL_GetTick() + CONTROL_LOOP_PERIOD_MS;
    last_rate_calc_time = HAL_GetTick();

    /* Main control loop (1kHz) */
    while (1) {
        /* Wait for next tick */
        while (HAL_GetTick() < next_tick) {
            __WFI(); /* Wait for interrupt */
        }

        /* Measure loop timing */
        loop_start_time = HAL_GetTick();
        uint32_t now = loop_start_time;

        /* Check for timing overrun */
        if (now > next_tick) {
            g_shared_memory.cm4_loop_overruns++;
        }

        /* Schedule next tick */
        next_tick = now + CONTROL_LOOP_PERIOD_MS;

        /* ===== Control Loop Start ===== */

        /* Read commands from CM7 */
        if (InterCore_ReadCommands(&commands) > 0) {
            /* Process emergency stop first (highest priority) */
            if (commands & CMD_EMERGENCY_STOP) {
                Motor_EmergencyStop();
                g_shared_memory.motor_status_flags = MOTOR_STATUS_READY;
                InterCore_AckCommand(CMD_EMERGENCY_STOP);
            }

            /* Process motor speed update */
            if (commands & CMD_UPDATE_MOTOR_SPEED) {
                /* Update motor target speeds from shared memory */
                for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                    Motor_SetSpeed(i, g_shared_memory.motor_speeds[i]);
                }
                InterCore_AckCommand(CMD_UPDATE_MOTOR_SPEED);
            }

            /* Process homing command */
            if (commands & CMD_HOME_MOTORS) {
                Motor_Home();
                InterCore_AckCommand(CMD_HOME_MOTORS);
            }

            /* Process solenoid trigger */
            if (commands & CMD_TRIGGER_SOLENOID) {
                uint8_t sol_id = g_shared_memory.solenoid_id;
                uint8_t string = g_shared_memory.solenoid_string;
                uint8_t fret = g_shared_memory.solenoid_fret;
                uint8_t duty = g_shared_memory.solenoid_duty_cycle;
                uint8_t mode = g_shared_memory.solenoid_mode;
                uint16_t time_ms = g_shared_memory.solenoid_activation_time_ms;
                uint16_t burst = g_shared_memory.solenoid_burst_cycles;

                /* Validate solenoid ID and string/fret */
                if (sol_id < 6 && string < 6 && fret < 8) {
                    /* Set frequency for target solenoid */
                    if (Solenoid_SetFrequency(sol_id, string, fret) == 0) {
                        /* Set duty cycle if non-zero */
                        if (duty > 0 && duty < 100) {
                            Solenoid_SetDutyCycle(sol_id, duty);
                        }

                        /* Start with timing parameters */
                        Solenoid_StartTimed(sol_id, mode, time_ms, burst);

                        /* Update active mask in shared memory */
                        g_shared_memory.solenoid_active_mask = Solenoid_GetActiveMask();
                    } else {
                        g_shared_memory.motor_errors |= ERROR_INVALID_COMMAND;
                    }
                } else {
                    g_shared_memory.motor_errors |= ERROR_INVALID_COMMAND;
                }

                InterCore_AckCommand(CMD_TRIGGER_SOLENOID);
            }

            /* Process solenoid stop */
            if (commands & CMD_STOP_SOLENOID) {
                uint8_t sol_id = g_shared_memory.solenoid_id;

                if (sol_id < 6) {
                    Solenoid_Stop(sol_id);
                    g_shared_memory.solenoid_active_mask = Solenoid_GetActiveMask();
                } else if (sol_id == 0xFF) {
                    /* 0xFF = stop all */
                    Solenoid_StopAll();
                    g_shared_memory.solenoid_active_mask = 0;
                }

                InterCore_AckCommand(CMD_STOP_SOLENOID);
            }

            /* Process motor acceleration change */
            if (commands & CMD_SET_MOTOR_ACCEL) {
                uint8_t mid = g_shared_memory.motor_cmd_id;
                if (mid < NUM_MOTORS) {
                    Motor_SetLimits(mid, 0, g_shared_memory.motor_cmd_accel);
                }
                InterCore_AckCommand(CMD_SET_MOTOR_ACCEL);
            }

            /* Process motor max speed change */
            if (commands & CMD_SET_MOTOR_MAXSPEED) {
                uint8_t mid = g_shared_memory.motor_cmd_id;
                if (mid < NUM_MOTORS) {
                    Motor_SetLimits(mid, g_shared_memory.motor_cmd_max_speed, 0);
                }
                InterCore_AckCommand(CMD_SET_MOTOR_MAXSPEED);
            }

            /* Process relative move */
            if (commands & CMD_MOTOR_MOVE_REL) {
                uint8_t mid = g_shared_memory.motor_cmd_id;
                if (mid < NUM_MOTORS) {
                    Motor_MoveRelative(mid, g_shared_memory.motor_cmd_position);
                }
                InterCore_AckCommand(CMD_MOTOR_MOVE_REL);
            }

            /* Process absolute move */
            if (commands & CMD_MOTOR_MOVE_ABS) {
                uint8_t mid = g_shared_memory.motor_cmd_id;
                if (mid < NUM_MOTORS) {
                    Motor_MoveToPosition(mid, g_shared_memory.motor_cmd_position);
                }
                InterCore_AckCommand(CMD_MOTOR_MOVE_ABS);
            }

            /* Process microstep mode change */
            if (commands & CMD_SET_MICROSTEP) {
                uint8_t mid = g_shared_memory.motor_cmd_id;
                if (mid < NUM_MOTORS) {
                    Motor_SetMicrostepMode(mid, g_shared_memory.motor_cmd_microstep_mode);
                }
                InterCore_AckCommand(CMD_SET_MICROSTEP);
            }

            /* Process motor enable/disable */
            if (commands & CMD_MOTOR_ENABLE) {
                uint8_t mid = g_shared_memory.motor_cmd_id;
                if (mid < NUM_MOTORS) {
                    if (g_shared_memory.motor_cmd_enable) {
                        Motor_Enable(mid);
                    } else {
                        Motor_Disable(mid);
                    }
                }
                InterCore_AckCommand(CMD_MOTOR_ENABLE);
            }

            /* Process position zero */
            if (commands & CMD_MOTOR_ZERO) {
                uint8_t mid = g_shared_memory.motor_cmd_id;
                if (mid < NUM_MOTORS) {
                    Motor_ZeroPosition(mid);
                }
                InterCore_AckCommand(CMD_MOTOR_ZERO);
            }

            /* Process solenoid direct-Hz trigger (tuning controller actuation) */
            if (commands & CMD_TRIGGER_SOLENOID_HZ) {
                uint8_t sol_id  = g_shared_memory.solenoid_id;
                uint16_t freq   = g_shared_memory.solenoid_freq_hz;
                uint8_t  duty   = g_shared_memory.solenoid_duty_cycle;
                uint8_t  mode   = g_shared_memory.solenoid_mode;
                uint16_t time_ms = g_shared_memory.solenoid_activation_time_ms;
                uint16_t burst   = g_shared_memory.solenoid_burst_cycles;

                if (sol_id < 6u && freq > 0u) {
                    if (Solenoid_SetFrequencyHz(sol_id, (uint32_t)freq) == 0) {
                        if (duty > 0u && duty < 100u) {
                            Solenoid_SetDutyCycle(sol_id, duty);
                        }
                        Solenoid_StartTimed(sol_id, mode, time_ms, burst);
                        g_shared_memory.solenoid_active_mask = Solenoid_GetActiveMask();
                    } else {
                        g_shared_memory.motor_errors |= ERROR_INVALID_COMMAND;
                    }
                }
                InterCore_AckCommand(CMD_TRIGGER_SOLENOID_HZ);
            }
        }

        /* Update solenoid auto-stop timing (1ms loop) */
        Solenoid_UpdateAutoStop();

        /* Update motor control (acceleration limiting, step generation) */
        /* Pass internal motor states array - Motor_Update handles all motors */
        Motor_Update(Motor_GetState(0));

        /* Update shared memory with current motor states */
        uint8_t any_moving = 0;
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            motor_states = Motor_GetState(i);
            if (motor_states != NULL) {
                g_shared_memory.current_speeds[i] = motor_states->current_speed;
                g_shared_memory.current_positions[i] = motor_states->position;
                g_shared_memory.target_speeds_status[i] = motor_states->target_speed;
                g_shared_memory.motor_is_moving[i] = motor_states->is_moving;
                if (motor_states->is_moving) any_moving = 1;
            }
            g_shared_memory.max_speeds_status[i] = Motor_GetMaxSpeed(i);
            g_shared_memory.max_accels_status[i] = Motor_GetMaxAccel(i);
            g_shared_memory.motor_enabled[i] = Motor_IsEnabled(i);
            g_shared_memory.motor_microstep_status[i] = Motor_GetMicrostepMode(i);
        }

        uint32_t status = MOTOR_STATUS_READY;
        if (any_moving) {
            status |= MOTOR_STATUS_MOVING;
        }
        g_shared_memory.motor_status_flags = status;

        /* Update heartbeat */
        InterCore_UpdateHeartbeat();

       
        /* ===== Control Loop End ===== */

        /* Measure loop execution time */
        uint32_t loop_time_us = (HAL_GetTick() - loop_start_time) * 1000;
        if (loop_time_us > g_shared_memory.cm4_max_loop_time_us) {
            g_shared_memory.cm4_max_loop_time_us = loop_time_us;
        }

        /* Calculate actual loop rate every second */
        loop_count++;
        if ((now - last_rate_calc_time) >= 1000) {
            g_shared_memory.cm4_loop_hz = loop_count;
            loop_count = 0;
            last_rate_calc_time = now;
        }
    }
}

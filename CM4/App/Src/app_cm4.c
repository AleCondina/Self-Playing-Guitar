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

/* External timer handles for debug access */
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;

/* Control loop timing */
#define CONTROL_LOOP_PERIOD_MS  1   /* 1ms = 1kHz control loop */

/* Motor test state machine */
typedef enum {
    MOTOR_TEST_FORWARD,     /* Rotating 360° forward */
    MOTOR_TEST_PAUSE_1,     /* Brief pause between directions */
    MOTOR_TEST_REVERSE,     /* Rotating 360° back */
    MOTOR_TEST_PAUSE_2,     /* Brief pause before repeating */
    MOTOR_TEST_DONE         /* Test complete */
} MotorTestState_t;

static MotorTestState_t motor_test_state = MOTOR_TEST_FORWARD;
static uint32_t motor_test_timer = 0;

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

    /* Initialize motor control system */
    Motor_Init();

    /* ================================================================
     * Motor Test Setup: 360° forward, then 360° back
     *
     * MOTOR_STEPS_PER_REV = 200 (full step, 1.8° per step)
     * Speed: 400 steps/sec = 2 revolutions/sec = 120 RPM
     * Microstepping: Full step
     * ================================================================ */
    Motor_SetMicrostepMode(0, MOTOR_MICROSTEP_FULL);   /* Full step mode */
    Motor_Enable(0);                                    /* Enable DRV8825 (EN LOW) */
    Motor_SetLimits(0, 400, 500);                       /* 400 steps/sec max, 500 accel */
    Motor_MoveRelative(0, MOTOR_STEPS_PER_REV);         /* Move 360° forward (200 steps) */

    /* Initialize solenoid control system */
    if (Solenoid_Init() != 0) {
        g_shared_memory.motor_errors |= ERROR_INVALID_COMMAND;
    }

    /* TEST: Direct timer register manipulation for debugging */
    /* Bypass HAL entirely to verify hardware is working */

    /* Method 1: HAL-based approach - Test High E string (string 5) */
    Solenoid_SetFrequency(0, 0, 0);  // Solenoid 0: E2 string (Low E), open (82.41Hz)
    Solenoid_SetFrequency(1, 1, 0);  // Solenoid 1: A2 string , open (110Hz)
    Solenoid_SetFrequency(2, 2, 0);  // Solenoid 2: D3 string , open (146.83 )
    Solenoid_Start(0);                // This actually starts the PWM!
    Solenoid_Start(1);                // This actually starts the PWM!
    Solenoid_Start(2);                // This actually starts the PWM!

    /* Method 2: Direct GPIO configuration (needed for PWM output) */
    /* Ensure GPIOB and TIM12 clocks are enabled */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();

    /* Configure PB14 as AF2 (TIM12_CH1) */
    GPIOB->MODER &= ~(0x3UL << (14 * 2));  // Clear mode bits for PB14
    GPIOB->MODER |= (0x2UL << (14 * 2));   // Set alternate function mode
    GPIOB->AFR[1] &= ~(0xFUL << ((14-8) * 4));  // Clear AF bits for PB14
    GPIOB->AFR[1] |= (0x2UL << ((14-8) * 4));   // Set AF2 (TIM12)
    GPIOB->OTYPER &= ~(1UL << 14);         // Push-pull output
    GPIOB->OSPEEDR |= (0x3UL << (14 * 2)); // High speed

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

                /* Validate solenoid ID and string/fret */
                if (sol_id < 3 && string < 6 && fret < 8) {
                    /* Set frequency for target solenoid */
                    if (Solenoid_SetFrequency(sol_id, string, fret) == 0) {
                        /* Set duty cycle if non-zero */
                        if (duty > 0 && duty < 100) {
                            Solenoid_SetDutyCycle(sol_id, duty);
                        }
                        /* Start PWM output */
                        Solenoid_Start(sol_id);

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

                if (sol_id < 3) {
                    Solenoid_Stop(sol_id);
                    g_shared_memory.solenoid_active_mask = Solenoid_GetActiveMask();
                } else if (sol_id == 0xFF) {
                    /* 0xFF = stop all */
                    Solenoid_StopAll();
                    g_shared_memory.solenoid_active_mask = 0;
                }

                InterCore_AckCommand(CMD_STOP_SOLENOID);
            }
        }

        /* ===== Motor Test State Machine ===== */
        if (motor_test_state != MOTOR_TEST_DONE) {
            MotorState_t *m0 = Motor_GetState(0);

            switch (motor_test_state) {
                case MOTOR_TEST_FORWARD:
                    /* Wait for forward move to complete */
                    if (m0 != NULL && !m0->is_moving) {
                        motor_test_timer = now;
                        motor_test_state = MOTOR_TEST_PAUSE_1;
                    }
                    break;

                case MOTOR_TEST_PAUSE_1:
                    /* 500ms pause before reversing */
                    if ((now - motor_test_timer) >= 500) {
                        Motor_MoveRelative(0, -MOTOR_STEPS_PER_REV);  /* 360° back */
                        motor_test_state = MOTOR_TEST_REVERSE;
                    }
                    break;

                case MOTOR_TEST_REVERSE:
                    /* Wait for reverse move to complete */
                    if (m0 != NULL && !m0->is_moving) {
                        motor_test_timer = now;
                        motor_test_state = MOTOR_TEST_PAUSE_2;
                    }
                    break;

                case MOTOR_TEST_PAUSE_2:
                    /* 500ms pause, then loop back to forward (line 265 = infinite loop) */
                    if ((now - motor_test_timer) >= 500) {
                        Motor_MoveRelative(0, MOTOR_STEPS_PER_REV);  /* 360° forward again */
                        motor_test_state = MOTOR_TEST_FORWARD;       /* LINE 265: change to MOTOR_TEST_DONE to stop */
                    }
                    break;

                default:
                    break;
            }
        }

        /* Update motor control (acceleration limiting, step generation) */
        /* Pass internal motor states array - Motor_Update handles all motors */
        Motor_Update(Motor_GetState(0));

        /* Update shared memory with current motor states */
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            motor_states = Motor_GetState(i);
            if (motor_states != NULL) {
                /* Update shared memory with current speed and position */
                g_shared_memory.current_speeds[i] = motor_states->current_speed;
                g_shared_memory.current_positions[i] = motor_states->position;
            }
        }

        /* Update motor state flags */
        uint8_t any_moving = 0;
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            motor_states = Motor_GetState(i);
            if (motor_states != NULL && motor_states->is_moving) {
                any_moving = 1;
                break;
            }
        }

        uint32_t status = MOTOR_STATUS_READY;
        if (any_moving) {
            status |= MOTOR_STATUS_MOVING;
        }
        g_shared_memory.motor_status_flags = status;

        /* Update heartbeat */
        InterCore_UpdateHeartbeat();

        /* TEST MODE: Cycle through all strings and frets every 2 seconds */
        // if ((now - last_test_change) >= 2000) {
        //     /* Update test parameters */
        //     test_fret++;
        //     if (test_fret >= 8) {
        //         test_fret = 0;
        //         test_string++;
        //         if (test_string >= 6) {
        //             test_string = 0;  /* Loop back to E2 (low E) */
        //         }
        //     }

        //     /* Update solenoid 0 with new frequency */
        //     Solenoid_SetFrequency(0, test_string, test_fret);

        //     /* Store current test state in shared memory for debugging */
        //     g_shared_memory.solenoid_string = test_string;
        //     g_shared_memory.solenoid_fret = test_fret;

        //     last_test_change = now;
        // }

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

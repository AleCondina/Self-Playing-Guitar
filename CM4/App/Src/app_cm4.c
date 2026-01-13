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
#include "shared_memory.h"
#include "stm32h7xx_hal.h"

/* Control loop timing */
#define CONTROL_LOOP_PERIOD_MS  1   /* 1ms = 1kHz control loop */

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
                /* TODO: Implement solenoid control */
                InterCore_AckCommand(CMD_TRIGGER_SOLENOID);
            }
        }

        /* Update motor control (acceleration limiting, position tracking) */
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

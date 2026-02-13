/**
 ******************************************************************************
 * @file    app_cm7.c
 * @brief   CM7 application implementation
 * @note    Main control loop for CM7 (pitch detection and command generation)
 ******************************************************************************
 */

#include "app_cm7.h"
#include "inter_core_cm7.h"
#include "debug_uart.h"
#include "audio_input.h"
#include "shared_memory.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>

/* Application state */
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_ERROR
} AppState_t;

static AppState_t app_state = STATE_INIT;

/**
 * @brief CM7 application main function
 */
void App_CM7_Main(void)
{
    uint32_t last_debug_time = 0;
    uint32_t test_counter = 0;

    printf("\r\n\r\n");
    printf("=========================================\r\n");
    printf("  STM32H755ZI Guitar Controller - CM7   \r\n");
    printf("  Firmware v%d.%d.%d                    \r\n",
           FIRMWARE_VERSION_MAJOR,
           FIRMWARE_VERSION_MINOR,
           FIRMWARE_VERSION_PATCH);
    printf("=========================================\r\n\r\n");

    /* Wait for CM4 to signal ready */
    printf("[CM7] Waiting for CM4 to initialize...\r\n");
    uint32_t timeout = HAL_GetTick() + 5000;
    while (!(g_shared_memory.motor_status_flags & MOTOR_STATUS_READY)) {
        if (HAL_GetTick() > timeout) {
            printf("[CM7] ERROR: CM4 not responding!\r\n");
            app_state = STATE_ERROR;
            break;
        }
        HAL_Delay(10);
    }

    if (app_state != STATE_ERROR) {
        printf("[CM7] CM4 ready! Starting application.\r\n\r\n");
        app_state = STATE_IDLE;

        /* Initialize motor tuning defaults and show help prompt */
        Debug_InitMotorDefaults();
    }

    /* Main loop */
    while (1) {
        /* Update heartbeat */
        g_shared_memory.cm7_heartbeat++;

        /* Process serial commands (non-blocking) */
        Debug_ProcessInput();

        /* State machine */
        switch (app_state) {
            case STATE_IDLE:
                /* Wait for activation */
                /* TODO: Add button press or UART command to start */
                HAL_Delay(10);

                /* For now, automatically transition to running after 2 seconds */
                if (HAL_GetTick() > 2000) {
                    printf("[CM7] Transitioning to RUNNING state\r\n");
                    app_state = STATE_RUNNING;
                }
                break;

            case STATE_RUNNING:
                /* Main application logic */

                /* DISABLED: CM7 motor speed test - conflicts with CM4 position control
                 * This code sends alternating speed commands (500/1000 steps/s) every 5 seconds
                 * to motor 0 via InterCore_SendMotorSpeed(). When enabled, it overrides any
                 * position-based movement (Motor_MoveRelative) running on CM4 because
                 * Motor_SetSpeed() exits position control mode.
                 *
                 * Uncomment to test CM7->CM4 motor speed commands:
                 */
                // if ((HAL_GetTick() % 5000) < 10) {
                //     test_counter++;
                //
                //     /* Alternate motor speeds for testing */
                //     int32_t speed = (test_counter % 2 == 0) ? 1000 : 500;
                //
                //     printf("[CM7] Test: Setting motor 0 to %ld steps/s\r\n", speed);
                //     InterCore_SendMotorSpeed(0, speed);
                // }

                /* Check CM4 health */
                if (!InterCore_IsCM4Alive()) {
                    printf("[CM7] WARNING: CM4 appears stalled!\r\n");
                }

                HAL_Delay(10);
                break;

            case STATE_ERROR:
                /* Error state - blink LED or send debug output */
                printf("[CM7] Error state - halted\r\n");
                HAL_Delay(1000);
                break;

            default:
                app_state = STATE_ERROR;
                break;
        }

        /* Print diagnostics every 5 seconds (disabled - use 'status' command instead) */
        // if (HAL_GetTick() - last_debug_time >= 5000) {
        //     last_debug_time = HAL_GetTick();
        //     Debug_PrintSharedMemory();
        // }
    }
}

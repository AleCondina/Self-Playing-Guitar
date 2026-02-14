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
#include "serial_cmd.h"
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
        printf("[CM7] CM4 ready! Homing motors...\r\n");
        InterCore_HomeMotors();
        HAL_Delay(100);  /* Let CM4 process home command */
        printf("[CM7] Home position established (all positions zeroed).\r\n\r\n");
        Debug_UART_StartRx();
        SerialCmd_Init();
        app_state = STATE_IDLE;
    }

    /* Main loop */
    while (1) {
        /* Update heartbeat */
        g_shared_memory.cm7_heartbeat++;

        /* State machine */
        switch (app_state) {
            case STATE_IDLE:
                printf("[CM7] System ready for commands.\r\n");
                app_state = STATE_RUNNING;
                break;

            case STATE_RUNNING:
                /* Process serial commands */
                SerialCmd_Process();

                /* Check CM4 health */
                if (!InterCore_IsCM4Alive()) {
                    printf("[CM7] WARNING: CM4 appears stalled!\r\n");
                }

                HAL_Delay(1);
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

        /* Print diagnostics every 5 seconds */
        if (HAL_GetTick() - last_debug_time >= 5000) {
            last_debug_time = HAL_GetTick();
            Debug_PrintSharedMemory();
        }
    }
}

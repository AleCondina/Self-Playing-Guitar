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
#include "song_scheduler_tuning.h"
#include "midi_parser.h"
#include "midi_data.h"
#include "song_event.h"

#include <stdio.h>



extern const uint8_t g_midi_data[];
extern const uint32_t g_midi_len;

static SongEvent_t g_events[256];
static uint32_t g_event_count;
static SongScheduler_t g_sched;








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
    }


       

    MIDI_Parse(g_midi_data,g_midi_len,g_events,256,&g_event_count);

printf("Parsed %lu events\n", g_event_count);

if (g_event_count > 0)
{
    uint32_t offset = g_events[0].start_time_ms;

    for (uint32_t i = 0; i < g_event_count; i++)
    {
        g_events[i].start_time_ms -= offset;
    }
}
        
        

        for (uint32_t i = 0; i < g_event_count; i++)
        {
printf("%lu: start=%lu dur=%lu end=%lu freq=%d string=%u motor=%u vel=%u\n",
       i,
       g_events[i].start_time_ms,
       g_events[i].duration_ms,
       g_events[i].start_time_ms + g_events[i].duration_ms,
       (int)g_events[i].freq,
       g_events[i].string_id,
       g_events[i].motor_position,
       g_events[i].velocity);
        }

        SongScheduler_Init(&g_sched,g_events,g_event_count,HAL_GetTick());
    /* Main loop */
    while (1) {
        /* Update heartbeat */
        g_shared_memory.cm7_heartbeat++;

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

              //  SongScheduler_Update(&g_sched, HAL_GetTick());
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

         SongScheduler_Update(&g_sched, HAL_GetTick());
    }
}

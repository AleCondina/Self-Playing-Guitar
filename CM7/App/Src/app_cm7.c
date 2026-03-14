/**
 ******************************************************************************
 * @file    app_cm7.c
 * @brief   CM7 application implementation
 * @note    Main control loop for CM7 (pitch detection and command generation)
 *
 * Control-hop flow (every ~170 ms, driven by audio DMA half-complete):
 *   1. Audio_HasNewSamples() fires
 *   2. FFT_ProcessBlock()  – polyphonic peak detection to active_frequencies[] / active_magnitudes[]
 *   3. SongSched_Tick()    – advance note schedule
 *   4. TuningCtrl_RunHop() – FF + PI control → InterCore_SendMotorGoto()
 *                             + solenoid coordination
 ******************************************************************************
 */

#include "app_cm7.h"
#include "application_config.h"
#include "inter_core_cm7.h"
#include "debug_uart.h"
#include "serial_cmd.h"
#include "audio_input.h"
#include "fft_processing.h"
#include "tuning_controller.h"
#include "calibration_controller.h"
#include "song_scheduler.h"
#include "shared_memory.h"
#include "stm32h755xx.h"
#include "stm32h7xx_hal.h"
#include "song_scheduler_tuning.h"
#include "midi_parser.h"
#include "midi_data.h"
#include "song_event.h"

#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_tim.h"
#include <stdint.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim6;



extern const uint8_t g_midi_data[];
extern const uint32_t g_midi_len;

static SongEvent_t g_events[256];
static uint32_t g_event_count;
static SongScheduler_t g_sched;








/* Application state */
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_RUNNING,
    STATE_ERROR
} AppState_t;

static AppState_t app_state = STATE_INIT;

/* Set by serial commands to request mode changes on the next audio hop */
static volatile uint8_t s_request_calibrate      = 0u;
static volatile uint8_t s_request_calibrate_mask = 0x3Fu;
static volatile uint8_t s_request_play            = 0u;

/**
 * @brief CM7 application main function
 */
void App_CM7_Main(void) {
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
        printf("[CM7] Home position established (all positions zeroed).\r\n");

        /* Set all configured motors to 1/16 microstepping (mode 4) */
        printf("[CM7] Setting motors to 1/16 microstepping...\r\n");
        for (uint8_t i = 0u; i < 4u; i++) {
            InterCore_SendMotorMicrostep(i, 4u);
        }
        HAL_Delay(50);  /* Let CM4 apply the new mode */
        printf("[CM7] Microstep mode set.\r\n\r\n");

        Debug_UART_StartRx();
        SerialCmd_Init();
        printf("\r\nType 'help' for commands.\r\n> ");

        /* Initialize FFT Processing */
        printf("[CM7] Initializing FFT engine...\r\n");
        FFT_Init();

        /* Initialize tuning controller (depends on home being established) */
        printf("[CM7] Initializing tuning controller...\r\n");
        TuningCtrl_Init();

        /* Register song scheduler note count (not started yet – use 'smoke' command) */
        printf("[CM7] Song scheduler ready (%lu notes loaded).\r\n",
               (unsigned long)SongSched_GetNoteCount());

        /* Print CSV log header for offline analysis */
        printf("[TC] timestamp_ms,string_id,target_hz,measured_hz,error_hz,"
               "ff_steps,trim_steps,cmd_steps,cur_steps,"
               "lock,pi_en,delay_ctr,confidence,faults\r\n");

        /* Initialize audio input (ADC + DMA) */
        printf("[CM7] Initializing audio input...\r\n");

        /* CRITICAL: Calibrate ADC before doing anything else */
        if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
            printf("[CM7] ERROR: ADC3 Calibration failed!\r\n");
            app_state = STATE_ERROR;
        } else if (HAL_ADC_GetState(&hadc3) != HAL_ADC_STATE_READY) {
            printf("[CM7] ERROR: Failed to initialize audio input!\r\n");
            app_state = STATE_ERROR;
        } else {
                printf("[CM7] Audio input initialized and calibrated.\r\n");

                /* Start audio acquisition */
                if (Audio_Start() != 0) {
                    printf("[CM7] ERROR: Failed to start audio acquisition!\r\n");
                    app_state = STATE_ERROR;
                } else {
                    printf("[CM7] Audio acquisition started.\r\n");
                    HAL_TIM_Base_Start(&htim6);
                    app_state = STATE_IDLE;
                }
            }
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

        /* Process serial commands (non-blocking) */
        SerialCmd_Process();

        /* State machine */
        switch (app_state) {
            case STATE_IDLE:
                printf("[CM7] System ready. Type 'calibrate' or 'smoke'.\r\n");
                app_state = STATE_RUNNING;
                break;

            case STATE_CALIBRATING:
                /* Process serial commands while calibrating */
                SerialCmd_Process();

                if (Audio_HasNewSamples()) {
                    uint16_t *audio_buf = Audio_GetBuffer();

                    /* FFT must run first – Cal_Hop() reads active_frequencies[] */
                    FFT_ProcessBlock(audio_buf);
                    Audio_MarkProcessed();

                    Cal_Hop();
                }

                /* Return to idle when calibration finishes */
                if (Cal_GetStatus() == CAL_STATUS_DONE ||
                    Cal_GetStatus() == CAL_STATUS_ERROR) {
                    app_state = STATE_RUNNING;
                }
                break;

            case STATE_RUNNING:
                /* Process serial commands */
                SerialCmd_Process();

                /* ============================================================
                 * Audio processing and tuning control hop
                 * Fires every ~170 ms (1024 samples / 6000 Hz sample rate)
                 * ============================================================ */
                if (Audio_HasNewSamples()) {
                    uint16_t *audio_buf = Audio_GetBuffer();

                    /* --- DSP pipeline --- */
                    /* FFT: polyphonic peak detection (retained for diagnostics) */
                    FFT_ProcessBlock(audio_buf);

                    /* Mark buffer consumed before any blocking work */
                    Audio_MarkProcessed();

                    /* --- Handle requests from serial commands --- */
                    if (s_request_calibrate) {
                        uint8_t mask = s_request_calibrate_mask;
                        s_request_calibrate = 0u;
                        Cal_Start(mask);
                        app_state = STATE_CALIBRATING;
                        Audio_MarkProcessed();
                        break;
                    }
                    if (s_request_play) {
                        s_request_play = 0u;
                        SongSched_Init();
                    }

                    /* --- Song scheduler tick --- */
                    SongSched_Tick(HAL_GetTick());

                    /* --- Tuning control hop ---
                     * Reads FFT results and song scheduler state,
                     * computes PI + FF commands, pushes to CM4 via
                     * InterCore_SendMotorGoto() / solenoid commands.
                     */
                    TuningCtrl_RunHop();

                    /* Diagnostic: print FFT polyphonic results (only during playback) */
                    if (SongSched_IsActive() && active_string_count > 0) {
                        printf("[CM7] FFT %d string(s): ", active_string_count);
                        for (uint8_t i = 0; i < active_string_count; i++) {
                            printf("%.2f Hz  ", (double)active_frequencies[i]);
                        }
                        printf("\r\n");
                    }
                }

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
        if (HAL_GetTick() - last_debug_time >= 5000u) {
            last_debug_time = HAL_GetTick();
            if (app_state == STATE_RUNNING && SongSched_IsActive()) {
                printf("[CM7] Song note %lu/%lu | CM4 loop %lu Hz\r\n",
                       (unsigned long)SongSched_GetNoteIndex(),
                       (unsigned long)SongSched_GetNoteCount(),
                       (unsigned long)g_shared_memory.cm4_loop_hz);
            }
        }

         SongScheduler_Update(&g_sched, HAL_GetTick());
    }
}

/* =========================================================================
 * Serial command request API
 * Called from serial_cmd.c handlers; safe to call from the main loop.
 * ========================================================================= */

void App_RequestCalibration(uint8_t string_mask)
{
    if (app_state != STATE_CALIBRATING) {
        s_request_calibrate_mask = string_mask;
        s_request_calibrate      = 1u;
    }
}

void App_RequestPlay(void)
{
    s_request_play = 1u;
}

/**
 ******************************************************************************
 * @file    app_cm7.c
 * @brief   CM7 application implementation
 * @note    Main control loop for CM7 (pitch detection and command generation)
 *
 * Control-hop flow (every ~170 ms, driven by audio DMA half-complete):
 *   1. Audio_HasNewSamples() fires
 *   2. FFT_ProcessBlock()  – polyphonic peak detection (legacy / diagnostics)
 *   3. Goertzel_ProcessBlock() – per-string frequency + magnitude estimation
 *   4. SongSched_Tick()    – advance note schedule
 *   5. TuningCtrl_RunHop() – FF + PI control → InterCore_SendMotorGoto()
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
#include "goertzel_processing.h"
#include "tuning_controller.h"
#include "calibration_controller.h"
#include "song_scheduler.h"
#include "shared_memory.h"
#include "stm32h755xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_tim.h"
#include <stdint.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim6;

/* Application state */
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_RUNNING,
    STATE_ERROR
} AppState_t;

static AppState_t app_state = STATE_INIT;

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
        printf("[CM7] Home position established (all positions zeroed).\r\n\r\n");
        Debug_UART_StartRx();
        SerialCmd_Init();
        printf("\r\nType 'help' for commands.\r\n> ");

        /* Initialize FFT Processing */
        printf("[CM7] Initializing FFT engine...\r\n");
        FFT_Init();

        /* Initialize Goertzel per-string pitch estimator */
        printf("[CM7] Initializing Goertzel estimator...\r\n");
        Goertzel_Init();

        /* Initialize tuning controller (depends on home being established) */
        printf("[CM7] Initializing tuning controller...\r\n");
        TuningCtrl_Init();

        /* Initialize song scheduler */
        printf("[CM7] Initializing song scheduler (%lu notes)...\r\n",
               (unsigned long)SongSched_GetNoteCount());
        SongSched_Init();

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


    /* Main loop */
    while (1) {
        /* Update heartbeat */
        g_shared_memory.cm7_heartbeat++;

        /* Process serial commands (non-blocking) */
        SerialCmd_Process();

        /* State machine */
        switch (app_state) {
            case STATE_IDLE:
                printf("[CM7] System ready. Starting calibration sweep...\r\n");
                Cal_Start();
                app_state = STATE_CALIBRATING;
                break;

            case STATE_CALIBRATING:
                /* Process serial commands while calibrating */
                SerialCmd_Process();

                if (Audio_HasNewSamples()) {
                    uint16_t *audio_buf = Audio_GetBuffer();

                    /* FFT must run first – Cal_Hop() reads active_frequencies[] */
                    FFT_ProcessBlock(audio_buf);
                    Goertzel_ProcessBlock(audio_buf);
                    Audio_MarkProcessed();

                    Cal_Hop();
                }

                /* Transition to normal play when calibration finishes */
                if (Cal_GetStatus() == CAL_STATUS_DONE ||
                    Cal_GetStatus() == CAL_STATUS_ERROR) {
                    SongSched_Init();
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

                    /* Goertzel: per-string exact frequency + magnitude */
                    Goertzel_ProcessBlock(audio_buf);

                    /* Mark buffer consumed before any blocking work */
                    Audio_MarkProcessed();

                    /* --- Song scheduler tick --- */
                    SongSched_Tick(HAL_GetTick());

                    /* --- Tuning control hop ---
                     * Reads Goertzel results and song scheduler state,
                     * computes PI + FF commands, pushes to CM4 via
                     * InterCore_SendMotorGoto() / solenoid commands.
                     */
                    TuningCtrl_RunHop();

                    /* Diagnostic: print FFT polyphonic results */
                    if (active_string_count > 0) {
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
            if (app_state == STATE_RUNNING) {
                printf("[CM7] Song note %lu/%lu | CM4 loop %lu Hz\r\n",
                       (unsigned long)SongSched_GetNoteIndex(),
                       (unsigned long)SongSched_GetNoteCount(),
                       (unsigned long)g_shared_memory.cm4_loop_hz);
            }
        }
    }
}

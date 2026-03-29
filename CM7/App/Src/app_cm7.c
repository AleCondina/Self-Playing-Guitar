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
#include "lut_builder.h"
#include "song_scheduler.h"
#include "double_string.h"
#include "shared_memory.h"
#include "stm32h755xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_tim.h"
#include <stdint.h>
#include <stdio.h>

extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;

/* Application state */
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_LUT_BUILDING,
    STATE_TUNING,
    STATE_RUNNING,
    STATE_ERROR
} AppState_t;

static AppState_t app_state = STATE_INIT;
uint8_t app_debug_flags = 0;

/* Set by serial commands to request mode changes on the next audio hop */
static volatile uint8_t s_request_calibrate      = 0u;
static volatile uint8_t s_request_calibrate_mask = 0x3Fu;
static volatile uint8_t s_request_play            = 0u;
static volatile uint8_t s_request_play_song_id    = 0u;
static volatile uint8_t s_request_lutbuild       = 0u;
static volatile uint8_t s_request_lutbuild_mask  = 0x3Fu;
static volatile uint8_t s_request_tune           = 0u;
static volatile uint8_t s_request_tune_mask      = 0x0Fu;
static volatile uint8_t s_request_play_dual      = 0u;

/**
 * @brief CM7 application main function
 */
void App_CM7_Main(void) {
    uint32_t last_debug_time = 0;

    /* DWT cycle counter for microsecond-precision timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    static uint32_t hop_last_cyc       = 0;
    static uint32_t hop_interval_us    = 0;
    static uint32_t proc_time_us       = 0;
    static uint32_t proc_max_us        = 0;

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

        /* Restore calibration LUT + polynomial from backup SRAM (VBAT-backed) */
        if (Cal_LoadFromSRAM()) {
            printf("[CM7] Calibration restored from backup SRAM.\r\n");
        } else {
            Cal_FitFromBaseLUT();
            printf("[CM7] No SRAM calibration — polynomial seeded from base LUT.\r\n");
        }

        /* Restore tune offsets from backup SRAM */
        if (Cal_TuneLoadFromSRAM()) {
            printf("[CM7] Tune offsets restored from backup SRAM.\r\n");
        }

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
                    HAL_TIM_Base_Start(&htim1);
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
                    /* Reset UART RX to recover from any overrun errors during
                     * calibration's heavy printf output. */
                    Debug_UART_ResetRx();
                    printf("\r\n[CMD] Calibration finished. Ready.\r\n> ");
                }
                break;

            case STATE_LUT_BUILDING:
                SerialCmd_Process();

                if (Audio_HasNewSamples()) {
                    uint16_t *audio_buf = Audio_GetBuffer();
                    FFT_ProcessBlock(audio_buf);
                    Audio_MarkProcessed();
                    LutBuilder_Hop();
                }

                if (LutBuilder_GetStatus() == LUT_STATUS_DONE ||
                    LutBuilder_GetStatus() == LUT_STATUS_ERROR) {
                    app_state = STATE_RUNNING;
                    Debug_UART_ResetRx();
                    printf("\r\n[CMD] LUT build finished. Ready.\r\n> ");
                }
                break;

            case STATE_TUNING:
                SerialCmd_Process();

                if (Audio_HasNewSamples()) {
                    uint16_t *audio_buf = Audio_GetBuffer();
                    FFT_ProcessBlock(audio_buf);
                    Audio_MarkProcessed();
                    Cal_TuneHop();
                }

                if (Cal_TuneGetStatus() == CAL_STATUS_DONE ||
                    Cal_TuneGetStatus() == CAL_STATUS_ERROR) {
                    app_state = STATE_RUNNING;
                    printf("\r\n[CMD] Tuning finished. Ready.\r\n> ");
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
                    /* Measure hop interval (time between consecutive audio-ready events) */
                    uint32_t cyc_now = DWT->CYCCNT;
                    hop_interval_us = (cyc_now - hop_last_cyc) / (SYSTEM_CLOCK_MHZ);
                    hop_last_cyc = cyc_now;

                    /* Measure processing time */
                    uint32_t proc_start = DWT->CYCCNT;

                    uint16_t *audio_buf = Audio_GetBuffer();

                    /* --- DSP pipeline --- */
                    FFT_ProcessBlock(audio_buf);

                    /* Mark buffer consumed before any blocking work */
                    Audio_MarkProcessed();

                    /* --- Handle requests from serial commands --- */
                    if (s_request_calibrate) {
                        uint8_t mask = s_request_calibrate_mask;
                        s_request_calibrate = 0u;
                        Cal_Start(mask);
                        app_state = STATE_CALIBRATING;
                        break;
                    }
                    if (s_request_lutbuild) {
                        uint8_t mask = s_request_lutbuild_mask;
                        s_request_lutbuild = 0u;
                        LutBuilder_Start(mask);
                        app_state = STATE_LUT_BUILDING;
                        break;
                    }
                    if (s_request_tune) {
                        uint8_t mask = s_request_tune_mask;
                        s_request_tune = 0u;
                        Cal_TuneStart(mask);
                        app_state = STATE_TUNING;
                        Audio_MarkProcessed();
                        break;
                    }
                    if (s_request_play_dual) {
                        s_request_play_dual = 0u;
                        TuningCtrl_ResetAll();
                        TuningCtrl_SetDualTrackMode(1u);
                        DblString_Init(DblString_GetBpm());
                        DblString_Start(HAL_GetTick());
                    } else if (s_request_play) {
                        uint8_t sid = s_request_play_song_id;
                        s_request_play = 0u;
                        TuningCtrl_SetDualTrackMode(0u);
                        TuningCtrl_ResetAll();
                        SongSched_InitSong(sid);
                    }

                    /* --- Lock-wait mode (optional) ---
                     * When SONG_LOCK_WAIT_MODE=1 the scheduler is held on the
                     * current note until ALL active strings have achieved PI lock.
                     * This guarantees every note sounds accurately in tune before
                     * advancing, at the cost of strict rhythmic timing.
                     *
                     * Set to 0 for target BPM playback (notes advance on schedule).
                     * Set to 1 if timing can be relaxed for guaranteed accuracy.
                     * Combine with a lower SONG_BPM value for the best of both.
                     */
#define SONG_LOCK_WAIT_MODE  0u
#if SONG_LOCK_WAIT_MODE
                    if (SongSched_IsActive()) {
                        uint8_t all_locked = TuningCtrl_GetLockState(SongSched_GetStringId());
                        if (SongSched_GetStringCount() >= 2u) {
                            all_locked &= TuningCtrl_GetLockState(SongSched_GetStringId2());
                        }
                        SongSched_HoldNote(!all_locked);
                    }
#endif

                    /* --- Song scheduler tick --- */
                    if (DblString_IsActive()) {
                        DblString_Tick(HAL_GetTick());
                    } else {
                        SongSched_Tick(HAL_GetTick());
                    }

                    /* --- Tuning control hop ---
                     * Reads FFT results and song scheduler state,
                     * computes PI + FF commands, pushes to CM4 via
                     * InterCore_SendMotorGoto() / solenoid commands.
                     */
                    TuningCtrl_RunHop();

                    /* End processing timer */
                    proc_time_us = (DWT->CYCCNT - proc_start) / (SYSTEM_CLOCK_MHZ);
                    if (proc_time_us > proc_max_us) proc_max_us = proc_time_us;

                    /* Debug prints AFTER proc timer so they don't inflate measurement */
                    if (fft_print_enabled && active_string_count > 0) {
                        printf("[FFT] %d peak(s): ", active_string_count);
                        for (uint8_t i = 0; i < active_string_count; i++) {
                            printf("%.2f Hz (%.0f)  ", (double)active_frequencies[i], (double)active_magnitudes[i]);
                        }
                        printf("\r\n");
                    }
                    if (app_debug_flags & DEBUG_FLAG_NOISE_FLOOR) {
                        static uint32_t nf_decim = 0;
                        if (++nf_decim % 6 == 0) {
                            printf("[NF] bin28: mag=%.0f nf=%.0f | bin67: mag=%.0f nf=%.0f | bin112: mag=%.0f nf=%.0f | peak=%.0f\r\n",
                                   (double)fft_dbg_mag[0], (double)fft_dbg_nf[0],
                                   (double)fft_dbg_mag[1], (double)fft_dbg_nf[1],
                                   (double)fft_dbg_mag[2], (double)fft_dbg_nf[2],
                                   (double)fft_dbg_peak);
                        }
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
            if (app_debug_flags & DEBUG_FLAG_HOP_TIMER) {
                printf("[CM7] hop_interval=%lu us | proc=%lu us | proc_max=%lu us\r\n",
                       (unsigned long)hop_interval_us,
                       (unsigned long)proc_time_us,
                       (unsigned long)proc_max_us);
                proc_max_us = 0;  /* Reset peak after reporting */
            }
            if (app_state == STATE_RUNNING && DblString_IsActive()) {
                printf("[CM7] DualTrack note %lu/%lu @ %lu BPM | CM4 loop %lu Hz\r\n",
                       (unsigned long)DblString_GetNoteIndex(),
                       (unsigned long)DblString_GetNoteCount(),
                       (unsigned long)DblString_GetBpm(),
                       (unsigned long)g_shared_memory.cm4_loop_hz);
            } else if (app_state == STATE_RUNNING && SongSched_IsActive()) {
                printf("[CM7] Song note %lu/%lu | CM4 loop %lu Hz\r\n",
                       (unsigned long)SongSched_GetNoteIndex(),
                       (unsigned long)SongSched_GetNoteCount(),
                       (unsigned long)g_shared_memory.cm4_loop_hz);
            }
        }
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
    s_request_play_song_id = 0u;
    s_request_play = 1u;
}

void App_RequestPlaySong(uint8_t song_id)
{
    s_request_play_song_id = song_id;
    s_request_play = 1u;
}

void App_RequestPlayDual(void)
{
    s_request_play_dual = 1u;
}

void App_RequestLutBuild(uint8_t string_mask)
{
    if (app_state != STATE_LUT_BUILDING) {
        s_request_lutbuild_mask = string_mask;
        s_request_lutbuild      = 1u;
    }
}

void App_RequestTune(uint8_t string_mask)
{
    if (app_state != STATE_TUNING) {
        s_request_tune_mask = string_mask;
        s_request_tune      = 1u;
    }
}

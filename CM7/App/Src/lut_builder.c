/**
 ******************************************************************************
 * @file    lut_builder.c
 * @brief   Fret-by-fret feed-forward LUT measurement tool
 *
 * How it works (per string)
 * -------------------------
 *  1. Motor starts at position 0 (open string / fret 0).
 *  2. User manually plucks the string every few seconds to keep it ringing.
 *  3. System waits for FFT to detect the open-string frequency.
 *  4. Records fret 0 = 0 microsteps.
 *  5. Motor sweeps FORWARD ONLY toward a far target position.
 *     Speed adjusts dynamically based on frequency proximity to target fret:
 *       >5 Hz away  → cruise  (800 microsteps/sec)
 *       2-5 Hz away → approach (200 microsteps/sec)
 *       <2 Hz away  → creep   (75 microsteps/sec)
 *  6. When FFT frequency is within ±1.0 Hz of the target fret, the motor
 *     is stopped and the current position is recorded.
 *  7. Steps 5-6 repeat for frets 1 through 6.
 *  8. Results are written into tuning_ff_*_bp[] and printed to UART.
 *  9. Motor returns to position 0, then the next string begins.
 *
 * Why FFT?
 * --------
 * The Goertzel phase-tracker is limited to ±3 Hz of its filter centre.
 * A fret sweep spans well beyond that, so we use the FFT's parabolic-
 * interpolated peak detection across the full 60–466 Hz band.
 *
 * Units
 * -----
 * All positions and speeds are in microstep pulses (1/16 stepping mode).
 ******************************************************************************
 */

#include "lut_builder.h"
#include "tuning_controller.h"   /* LUT arrays + TUNING_FF_NUM_POINTS */
#include "fft_processing.h"      /* active_frequencies[], active_magnitudes[], active_string_count */
#include "inter_core_cm7.h"
#include "shared_memory.h"
#include "application_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Configuration — adjust if hardware behaviour differs
 * ========================================================================= */

/** Number of frets to measure (fret 0 = open, frets 1-6). */
#define LUT_NUM_FRETS           7u

/** Motor speed tiers (microsteps/sec at 1/16 stepping). */
#define LUT_SPEED_CRUISE        800
#define LUT_SPEED_APPROACH      200
#define LUT_SPEED_CREEP         75

/** Frequency thresholds for speed tier selection (Hz). */
#define LUT_FREQ_FAR_HZ        5.0f
#define LUT_FREQ_NEAR_HZ       2.0f

/** Frequency tolerance to record a fret position (Hz). */
#define LUT_FREQ_LOCK_HZ       5.0f

/** Signal detection — reject peaks below this fraction of open frequency. */
#define LUT_OPEN_FREQ_MIN_RATIO 0.80f

/** Tolerance for accepting open-string detection (Hz). */
#define LUT_OPEN_TOLERANCE_HZ   1.0f

/** Consecutive hops without signal before UART prompt. */
#define LUT_NO_SIG_PROMPT_HOPS  20u    /* ~3.4 s */

/** Consecutive hops without signal before error. */
#define LUT_NO_SIG_ERROR_HOPS   60u    /* ~10 s */

/** Hops to wait after motor reaches home before starting detection. */
#define LUT_HOME_SETTLE_HOPS    6u     /* ~1.0 s */

/** Far target position — motor sweeps toward this (microsteps). */
#define LUT_MAX_STEPS           8000

/** Hops to wait after stopping motor at a fret before resuming. */
#define LUT_LOCK_SETTLE_HOPS    2u     /* ~340 ms */

/** Bitmask of strings that have configured stepper motors.
 *  Currently motors 0-3 (E2, A2, D3, G3) are wired.
 *  Update this when B3 (bit 4) and E4 (bit 5) steppers are connected. */
#define LUT_CONFIGURED_MOTORS   0x0Fu  /* bits 0-3 = strings E2,A2,D3,G3 */

/* Open-string frequencies per tuning — must match FFT open_strings[]. */
static const float s_open_freq[NUM_TUNINGS][NUM_STRINGS] = {
    [TUNING_STANDARD] = { 82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f },
    [TUNING_DROP_D]   = { 73.42f,  82.41f, 146.83f, 196.00f, 246.94f, 329.63f },
    [TUNING_DROP_G2]  = { 82.41f,  98.00f, 146.83f, 196.00f, 246.94f, 329.63f },
};

static const char *s_string_names[NUM_TUNINGS][NUM_STRINGS] = {
    [TUNING_STANDARD] = { "E2", "A2", "D3", "G3", "B3", "E4" },
    [TUNING_DROP_D]   = { "D2", "E2", "D3", "G3", "B3", "E4" },
    [TUNING_DROP_G2]  = { "E2", "G2", "D3", "G3", "B3", "E4" },
};

/* =========================================================================
 * State machine
 * ========================================================================= */

typedef enum {
    LUT_PHASE_IDLE = 0,
    LUT_PHASE_WAIT_FOR_SIGNAL,   /**< Waiting for user to pluck open string    */
    LUT_PHASE_MEASURE_OPEN,      /**< Validate open frequency, record fret 0   */
    LUT_PHASE_SEEKING,           /**< Motor moving forward, adjusting speed    */
    LUT_PHASE_LOCK,              /**< Within tolerance — stop and record       */
    LUT_PHASE_LOCK_SETTLE,       /**< Brief pause after lock before next fret  */
    LUT_PHASE_NEXT_FRET,         /**< Advance fret counter                     */
    LUT_PHASE_BUILD,             /**< Write LUT, print results, go home        */
    LUT_PHASE_RETURN_HOME,       /**< All motors returning to position 0       */
    LUT_PHASE_DONE,
    LUT_PHASE_ERROR
} LutPhase_t;

/* =========================================================================
 * Private state
 * ========================================================================= */

static LutPhase_t        s_phase        = LUT_PHASE_IDLE;
static LutBuilderStatus_t s_status      = LUT_STATUS_IDLE;
static uint8_t            s_str         = 0u;
static uint8_t            s_string_mask = 0x3Fu;
static uint8_t            s_current_fret = 0u;
static uint16_t           s_no_sig_cnt  = 0u;
static uint16_t           s_settle_cnt  = 0u;
static int32_t            s_last_speed  = 0;
static uint8_t            s_motor_stopped = 0u;

/** Precomputed target frequencies for the current string (frets 0-6). */
static float s_target_freqs[LUT_NUM_FRETS];

/** Recorded step positions and frequencies for the current string. */
static int32_t s_fret_steps[LUT_NUM_FRETS];
static float   s_fret_freqs[LUT_NUM_FRETS];

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief Precompute equal-temperament target frequencies for frets 0-6.
 */
static void ComputeTargets(uint8_t str)
{
    float open_f = s_open_freq[TuningCtrl_GetActiveTuning()][str];
    for (uint8_t fret = 0u; fret < LUT_NUM_FRETS; fret++) {
        s_target_freqs[fret] = open_f * powf(2.0f, (float)fret / 12.0f);
    }
}

/**
 * @brief Find the best FFT peak near the expected frequency for this string.
 *
 * Scans active_frequencies[] for the peak closest to the current fret target,
 * within the valid frequency window for this string.
 *
 * @param[out] out_freq   Measured frequency (0 if no valid peak)
 * @param[out] out_mag    Magnitude of the matched peak (0 if no valid peak)
 */
static void FindBestPeak(float *out_freq, float *out_mag)
{
    float min_f = s_open_freq[TuningCtrl_GetActiveTuning()][s_str] * LUT_OPEN_FREQ_MIN_RATIO;
    float max_f = s_target_freqs[LUT_NUM_FRETS - 1u] * 1.15f;
    float target = s_target_freqs[s_current_fret];

    float best_freq = 0.0f;
    float best_mag  = 0.0f;
    float best_err  = 9999.0f;

    for (uint8_t i = 0u; i < active_string_count; i++) {
        float f = active_frequencies[i];
        float m = active_magnitudes[i];

        /* Reject out-of-range (SNR already vetted by FFT_ProcessBlock) */
        if (f < min_f || f > max_f) continue;
        (void)m;  /* magnitude no longer gate-checked here */

        float err = fabsf(f - target);
        if (err < best_err) {
            best_err  = err;
            best_freq = f;
            best_mag  = m;
        }
    }

    *out_freq = best_freq;
    *out_mag  = best_mag;
}

/**
 * @brief Set motor speed, only sending the command if the tier changed.
 */
static void SetSpeed(int32_t speed)
{
    if (speed != s_last_speed) {
        InterCore_SendMotorMaxSpeed(s_str, speed);
        s_last_speed = speed;
    }
}

/**
 * @brief Stop the motor at its current position.
 */
static void StopMotor(void)
{
    int32_t pos = g_shared_memory.current_positions[s_str];
    InterCore_SendMotorGoto(s_str, pos);
    s_motor_stopped = 1u;
}

/**
 * @brief Resume motor moving forward toward LUT_MAX_STEPS.
 */
static void ResumeMotor(void)
{
    InterCore_SendMotorGoto(s_str, LUT_MAX_STEPS);
    s_motor_stopped = 0u;
}

/**
 * @brief Write the recorded fret data into the tuning controller LUT arrays.
 */
static void WriteLUT(uint8_t str)
{
    for (uint8_t fret = 0u; fret < LUT_NUM_FRETS; fret++) {
        tuning_ff_freq_bp[str][fret]  = s_fret_freqs[fret];
        tuning_ff_steps_bp[str][fret] = (float)s_fret_steps[fret];
    }
}

/**
 * @brief Print copy-paste-ready results for one string.
 */
static void PrintResults(uint8_t str)
{
    printf("[LUT-RESULT] /* %s */ { ", s_string_names[TuningCtrl_GetActiveTuning()][str]);
    for (uint8_t fret = 0u; fret < LUT_NUM_FRETS; fret++) {
        printf("%.1f", (double)s_fret_steps[fret]);
        if (fret < LUT_NUM_FRETS - 1u) printf(", ");
    }
    printf(" },\r\n");

    printf("[LUT-FREQ]   /* %s */ { ", s_string_names[TuningCtrl_GetActiveTuning()][str]);
    for (uint8_t fret = 0u; fret < LUT_NUM_FRETS; fret++) {
        printf("%.2f", (double)s_fret_freqs[fret]);
        if (fret < LUT_NUM_FRETS - 1u) printf(", ");
    }
    printf(" },\r\n");
}

/**
 * @brief Print the final summary with all strings' results.
 */
static void PrintFinalSummary(void)
{
    printf("\r\n[LUT] ============================================\r\n");
    printf("[LUT] LUT measurement complete!\r\n");
    printf("[LUT] Copy-paste into tuning_controller.c:\r\n");
    printf("[LUT] ============================================\r\n");
    printf("[LUT] tuning_ff_steps_bp:\r\n");

    for (uint8_t str = 0u; str < NUM_STRINGS; str++) {
        if (!((s_string_mask >> str) & 1u)) continue;
        printf("    /* %s */ { ", s_string_names[TuningCtrl_GetActiveTuning()][str]);
        for (uint8_t fret = 0u; fret < LUT_NUM_FRETS; fret++) {
            printf("%.1f", (double)tuning_ff_steps_bp[str][fret]);
            if (fret < LUT_NUM_FRETS - 1u) printf(", ");
        }
        printf(" },\r\n");
    }

    printf("[LUT] tuning_ff_freq_bp:\r\n");
    for (uint8_t str = 0u; str < NUM_STRINGS; str++) {
        if (!((s_string_mask >> str) & 1u)) continue;
        printf("    /* %s */ { ", s_string_names[TuningCtrl_GetActiveTuning()][str]);
        for (uint8_t fret = 0u; fret < LUT_NUM_FRETS; fret++) {
            printf("%.2f", (double)tuning_ff_freq_bp[str][fret]);
            if (fret < LUT_NUM_FRETS - 1u) printf(", ");
        }
        printf(" },\r\n");
    }

    printf("[LUT] ============================================\r\n\r\n");
}

/**
 * @brief Advance s_str to the next string in the mask.
 * @return 1 if a next string was found, 0 if all done.
 */
static uint8_t AdvanceToNextString(void)
{
    do {
        s_str++;
    } while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u));

    return (s_str < NUM_STRINGS) ? 1u : 0u;
}

/**
 * @brief Initialize state for a new string.
 */
static void InitString(void)
{
    ComputeTargets(s_str);
    s_current_fret  = 0u;
    s_no_sig_cnt    = 0u;
    s_settle_cnt    = LUT_HOME_SETTLE_HOPS;
    s_last_speed    = 0;
    s_motor_stopped = 0u;

    memset(s_fret_steps, 0, sizeof(s_fret_steps));
    memset(s_fret_freqs, 0, sizeof(s_fret_freqs));

    /* Send motor to home */
    InterCore_SendMotorMaxSpeed(s_str, LUT_SPEED_CRUISE);
    InterCore_SendMotorGoto(s_str, 0);

    printf("\r\n[LUT] ---- String %u (%s) ----\r\n",
           (unsigned)s_str, s_string_names[TuningCtrl_GetActiveTuning()][s_str]);
    printf("[LUT] Target frequencies (frets 0-6):\r\n");
    printf("[LUT]   ");
    for (uint8_t f = 0u; f < LUT_NUM_FRETS; f++) {
        printf("%.2f  ", (double)s_target_freqs[f]);
    }
    printf("\r\n");
    printf("[LUT] Pluck the %s string and keep it ringing...\r\n",
           s_string_names[TuningCtrl_GetActiveTuning()][s_str]);

    s_phase = LUT_PHASE_WAIT_FOR_SIGNAL;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void LutBuilder_Start(uint8_t string_mask)
{
    /* Mask to only strings with configured stepper motors */
    string_mask &= LUT_CONFIGURED_MOTORS;

    if (string_mask == 0u) {
        printf("[LUT] ERROR: no configured motors in mask.\r\n");
        printf("[LUT] Configured motors: 0x%02X (E2,A2,D3,G3)\r\n",
               (unsigned)LUT_CONFIGURED_MOTORS);
        s_status = LUT_STATUS_ERROR;
        return;
    }

    s_string_mask = string_mask;

    /* Find the first set bit */
    s_str = 0u;
    while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u)) {
        s_str++;
    }

    s_status = LUT_STATUS_RUNNING;

    /* Enable all motors in the mask (DRV8825 EN pin LOW = enabled) */
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        if ((s_string_mask >> i) & 1u) {
            InterCore_SendMotorEnable(i, 1u);
        }
    }

    printf("\r\n[LUT] ============================================\r\n");
    printf("[LUT] Feed-forward LUT measurement started\r\n");
    printf("[LUT] String mask: 0x%02X\r\n", (unsigned)s_string_mask);
    printf("[LUT] Frets: 0-6 | Lock tolerance: +/-%.1f Hz\r\n",
           (double)LUT_FREQ_LOCK_HZ);
    printf("[LUT] Speeds: cruise=%d | approach=%d | creep=%d microsteps/sec\r\n",
           LUT_SPEED_CRUISE, LUT_SPEED_APPROACH, LUT_SPEED_CREEP);
    printf("[LUT] ============================================\r\n");

    InitString();
}

void LutBuilder_StartAll(void)
{
    LutBuilder_Start(LUT_CONFIGURED_MOTORS);
}

void LutBuilder_Hop(void)
{
    switch (s_phase) {

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_WAIT_FOR_SIGNAL:
    {
        /* Wait for motor to settle at home first */
        if (s_settle_cnt > 0u) {
            s_settle_cnt--;
            break;
        }

        /* Look for the open-string frequency */
        float meas_f = 0.0f;
        float meas_m = 0.0f;
        FindBestPeak(&meas_f, &meas_m);

        if (meas_f > 0.0f &&
            fabsf(meas_f - s_target_freqs[0]) <= LUT_OPEN_TOLERANCE_HZ) {
            /* Open string detected */
            printf("[LUT] Open string detected: %.2f Hz (target: %.2f Hz)\r\n",
                   (double)meas_f, (double)s_target_freqs[0]);
            s_no_sig_cnt = 0u;
            s_phase = LUT_PHASE_MEASURE_OPEN;
        } else {
            s_no_sig_cnt++;
            if (s_no_sig_cnt % LUT_NO_SIG_PROMPT_HOPS == 0u && s_no_sig_cnt > 0u) {
                printf("[LUT] No signal — pluck the %s string...\r\n",
                       s_string_names[TuningCtrl_GetActiveTuning()][s_str]);
            }
            if (s_no_sig_cnt >= LUT_NO_SIG_ERROR_HOPS) {
                printf("[LUT] ERROR: no signal for string %u (%s) after %u hops.\r\n",
                       (unsigned)s_str, s_string_names[TuningCtrl_GetActiveTuning()][s_str],
                       (unsigned)s_no_sig_cnt);
                s_phase  = LUT_PHASE_ERROR;
                s_status = LUT_STATUS_ERROR;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_MEASURE_OPEN:
    {
        /* Record fret 0 = position 0 (home) */
        s_fret_steps[0] = 0;
        s_fret_freqs[0] = s_target_freqs[0];

        printf("[LUT] Fret 0 (open): 0 microsteps @ %.2f Hz\r\n",
               (double)s_target_freqs[0]);

        /* Set up for fret 1 */
        s_current_fret = 1u;
        s_no_sig_cnt   = 0u;

        /* Start motor moving forward */
        SetSpeed(LUT_SPEED_CRUISE);
        ResumeMotor();

        printf("[LUT] Seeking fret 1 (%.2f Hz)...\r\n",
               (double)s_target_freqs[1]);

        s_phase = LUT_PHASE_SEEKING;
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_SEEKING:
    {
        float meas_f = 0.0f;
        float meas_m = 0.0f;
        FindBestPeak(&meas_f, &meas_m);

        /* No signal handling */
        if (meas_f <= 0.0f) {
            s_no_sig_cnt++;

            /* Stop motor while string is silent */
            if (!s_motor_stopped) {
                StopMotor();
            }

            if (s_no_sig_cnt % LUT_NO_SIG_PROMPT_HOPS == 0u && s_no_sig_cnt > 0u) {
                printf("[LUT] No signal — pluck the %s string again...\r\n",
                       s_string_names[TuningCtrl_GetActiveTuning()][s_str]);
            }
            if (s_no_sig_cnt >= LUT_NO_SIG_ERROR_HOPS) {
                printf("[LUT] ERROR: lost signal on string %u (%s) at fret %u.\r\n",
                       (unsigned)s_str, s_string_names[TuningCtrl_GetActiveTuning()][s_str],
                       (unsigned)s_current_fret);
                InterCore_SendMotorGoto(s_str, 0);
                s_phase  = LUT_PHASE_ERROR;
                s_status = LUT_STATUS_ERROR;
            }
            break;
        }

        /* Signal restored — resume motor if it was stopped */
        s_no_sig_cnt = 0u;
        if (s_motor_stopped) {
            ResumeMotor();
        }

        /* Compute frequency error to current fret target */
        float target_f  = s_target_freqs[s_current_fret];
        float freq_error = fabsf(meas_f - target_f);

        /* Check for lock */
        if (freq_error <= LUT_FREQ_LOCK_HZ) {
            s_phase = LUT_PHASE_LOCK;
            break;
        }

        /* Adjust speed based on proximity */
        if (freq_error > LUT_FREQ_FAR_HZ) {
            SetSpeed(LUT_SPEED_CRUISE);
        } else if (freq_error > LUT_FREQ_NEAR_HZ) {
            SetSpeed(LUT_SPEED_APPROACH);
        } else {
            SetSpeed(LUT_SPEED_CREEP);
        }

        /* Safety: check if we've exceeded max travel */
        int32_t cur_pos = g_shared_memory.current_positions[s_str];
        if (cur_pos >= LUT_MAX_STEPS) {
            printf("[LUT] ERROR: exceeded max travel (%d microsteps) on string %u.\r\n",
                   LUT_MAX_STEPS, (unsigned)s_str);
            InterCore_SendMotorGoto(s_str, 0);
            s_phase  = LUT_PHASE_ERROR;
            s_status = LUT_STATUS_ERROR;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_LOCK:
    {
        /* Stop motor and record position */
        StopMotor();

        int32_t pos = g_shared_memory.current_positions[s_str];

        /* Re-read FFT for the recorded frequency */
        float meas_f = 0.0f;
        float meas_m = 0.0f;
        FindBestPeak(&meas_f, &meas_m);

        s_fret_steps[s_current_fret] = pos;
        s_fret_freqs[s_current_fret] = (meas_f > 0.0f) ? meas_f : s_target_freqs[s_current_fret];

        printf("[LUT] Fret %u: %ld microsteps @ %.2f Hz (target: %.2f Hz)\r\n",
               (unsigned)s_current_fret,
               (long)pos,
               (double)s_fret_freqs[s_current_fret],
               (double)s_target_freqs[s_current_fret]);

        /* Brief settle before moving to next fret */
        s_settle_cnt = LUT_LOCK_SETTLE_HOPS;
        s_phase = LUT_PHASE_LOCK_SETTLE;
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_LOCK_SETTLE:
    {
        if (s_settle_cnt > 0u) {
            s_settle_cnt--;
            break;
        }
        s_phase = LUT_PHASE_NEXT_FRET;
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_NEXT_FRET:
    {
        s_current_fret++;

        if (s_current_fret >= LUT_NUM_FRETS) {
            /* All frets measured for this string */
            s_phase = LUT_PHASE_BUILD;
        } else {
            /* Resume forward motion toward next fret */
            s_no_sig_cnt = 0u;
            SetSpeed(LUT_SPEED_CRUISE);
            ResumeMotor();

            printf("[LUT] Seeking fret %u (%.2f Hz)...\r\n",
                   (unsigned)s_current_fret,
                   (double)s_target_freqs[s_current_fret]);

            s_phase = LUT_PHASE_SEEKING;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_BUILD:
    {
        /* Write measured data into tuning LUT */
        WriteLUT(s_str);

        /* Print results for this string */
        PrintResults(s_str);

        /* Return motor to home */
        InterCore_SendMotorMaxSpeed(s_str, LUT_SPEED_CRUISE);
        InterCore_SendMotorGoto(s_str, 0);

        /* Advance to next string in mask */
        if (AdvanceToNextString()) {
            /* Brief wait for motor to return home before next string */
            InitString();
        } else {
            /* All strings done — return all motors to home */
            printf("\r\n[LUT] Returning all motors to home...\r\n");
            for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
                if ((s_string_mask >> i) & 1u) {
                    InterCore_SendMotorGoto(i, 0);
                }
            }
            s_settle_cnt = LUT_HOME_SETTLE_HOPS;
            s_phase = LUT_PHASE_RETURN_HOME;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_RETURN_HOME:
    {
        if (s_settle_cnt > 0u) {
            s_settle_cnt--;
            break;
        }

        PrintFinalSummary();

        s_phase  = LUT_PHASE_DONE;
        s_status = LUT_STATUS_DONE;
        break;
    }

    /* ------------------------------------------------------------------ */
    case LUT_PHASE_IDLE:
    case LUT_PHASE_DONE:
    case LUT_PHASE_ERROR:
    default:
        break;
    }
}

LutBuilderStatus_t LutBuilder_GetStatus(void)
{
    return s_status;
}

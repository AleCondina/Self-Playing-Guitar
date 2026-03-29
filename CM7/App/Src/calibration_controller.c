/**
 ******************************************************************************
 * @file    calibration_controller.c
 * @brief   Feed-forward fret calibration + open-string tune-offset workflow
 *
 * Calibration flow
 * ----------------
 *  1. Start from motor position 0 (home / open-string reference).
 *  2. For each fret breakpoint:
 *     a. Use the currently loaded LUT as the feed-forward estimate.
 *     b. Move to that estimate and let the motor settle.
 *     c. Re-excite the string at the tracked FFT frequency when available,
 *        otherwise at the target fret frequency.
 *     d. Run the same PI-style lock logic used by the tune workflow until the
 *        fret is confirmed. There is no retry limit or timeout; the state
 *        machine keeps reacquiring until it locks.
 *     e. Record the actual motor steps for that fret.
 *  3. Write the measured fret points directly into tuning_ff_*_bp[].
 *  4. Fit the quadratic feed-forward polynomial and save to backup SRAM.
 *
 * Open-string tuning
 * ------------------
 * The separate tune workflow is kept intact. It still computes per-string
 * offsets that the tuning controller adds on top of the calibrated base LUT.
 ******************************************************************************
 */

#include "calibration_controller.h"
#include "tuning_controller.h"
#include "fft_processing.h"
#include "inter_core_cm7.h"
#include "shared_memory.h"
#include "application_config.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Calibration parameters
 * ========================================================================= */

/** Strings with commissioned stepper hardware. */
#define CAL_CONFIGURED_STRING_MASK   0x0Fu

/** Direct fret points written back into tuning_ff_*_bp[]. */
#define CAL_NUM_FRETS                TUNING_FF_NUM_POINTS

/** Motor speed used for fret-to-fret calibration moves. */
#define CAL_MOTOR_SPEED              3000

/** Hops to let the motor settle after reaching the feed-forward estimate. */
#define CAL_MOVE_SETTLE_HOPS         2u

/** Timed solenoid burst used for initial excitation and re-excitation. */
#define CAL_REEXCITE_DURATION_MS     300u

/** Hops to wait for the solenoid burst to clear the FFT window. */
#define CAL_REEXCITE_WAIT_HOPS       4u

/** Silent hops before calibration reacquires the signal. */
#define CAL_REEXCITE_TRIGGER_HOPS    3u

/** Consecutive in-band hops required before a fret is accepted. */
#define CAL_LOCK_CONFIRM_HOPS        4u

/** Reminder cadence while waiting for a manual pluck. */
#define CAL_PLUCK_PROMPT_HOPS        20u

/** Calibration solenoid duty cycle. */
#define CAL_SOLENOID_DUTY            50u

/* Backup SRAM base address and magic word (STM32H7, VBAT-backed, 4 KB). */
#define CAL_BKPSRAM_BASE             ((uint32_t)0x38800000UL)
#define CAL_PERSIST_MAGIC            0xCA115701UL

/* =========================================================================
 * Shared tune-style lock parameters
 * ========================================================================= */

#define TUNE_PI_KP                     16.0f
#define TUNE_PI_KI                     4.0f
#define TUNE_PI_MAX_TRIM               600.0f
#define TUNE_MAX_STEP_DELTA            800.0f
#define TUNE_MIN_MOVE_STEPS            4.0f
#define TUNE_LOCK_BAND_HZ              0.2f
#define TUNE_INTEGRAL_ENABLE_BAND_HZ   1.0f
#define TUNE_LOCK_CONSEC_HOPS          2u
#define TUNE_PI_LOCK_TIMEOUT           40u
#define TUNE_DEAD_HOPS                 3u
#define TUNE_MAX_RETRIES               3u
#define TUNE_REEXCITE_DURATION_MS      300u
#define TUNE_REEXCITE_WAIT_HOPS        4u
#define TUNE_MOTOR_SPEED               3000
#define TUNE_PLUCK_TIMEOUT             25u
#define TUNE_MOTOR_WAIT_HOPS           20u
#define TUNE_SUPPORTED_STRING_MASK     0x0Fu
#define TUNE_TRACK_MIN_RATIO           0.60f
#define TUNE_TRACK_MAX_RATIO           1.50f
#define TUNE_SOLENOID_DUTY             50u

/* Open-string frequencies (Hz) per tuning. */
static const float s_open_freq[NUM_TUNINGS][NUM_STRINGS] = {
    [TUNING_STANDARD] = { E2_HZ, A2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ },
    [TUNING_DROP_D]   = { D2_HZ, E2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ },
    [TUNING_DROP_G2]  = { E2_HZ, G2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ },
};

static const char *s_string_names[NUM_TUNINGS][NUM_STRINGS] = {
    [TUNING_STANDARD] = { "E2", "A2", "D3", "G3", "B3", "E4" },
    [TUNING_DROP_D]   = { "D2", "E2", "D3", "G3", "B3", "E4" },
    [TUNING_DROP_G2]  = { "E2", "G2", "D3", "G3", "B3", "E4" },
};

/**
 * Hard upper limit on motor position during calibration and tune, per string.
 * Strings 4-5 keep conservative placeholders until that hardware is wired.
 */
static const int32_t s_cal_steps_max[NUM_STRINGS] = {
    5000,  /* E2 */
    7500,  /* A2 / G2 */
    9000,  /* D3 */
    8200,  /* G3 */
    2200,  /* B3 placeholder */
    2200,  /* E4 placeholder */
};

/* Forward declaration: calibration invalidates tune offsets for reworked strings. */
static uint8_t Tune_ClearOffsetsMask(uint8_t string_mask);

/* =========================================================================
 * Persistent storage layout
 * ========================================================================= */

typedef struct {
    uint32_t magic;
    float    ff_freq [NUM_STRINGS][TUNING_FF_NUM_POINTS];
    float    ff_steps[NUM_STRINGS][TUNING_FF_NUM_POINTS];
    float    poly_a  [NUM_STRINGS];
    float    poly_b  [NUM_STRINGS];
    float    poly_c  [NUM_STRINGS];
    /* Semitone-domain coefficients (added for v2 controller) */
    float    semi_ff  [NUM_STRINGS][3];   /**< tuning_ff_semi_coeff[str][a,b,c] */
    float    semi_plant[NUM_STRINGS][3];  /**< tuning_plant_coeff[str][a,b,c]   */
    uint32_t checksum;
} __attribute__((packed)) CalPersist_t;

/* =========================================================================
 * Public polynomial coefficients
 * ========================================================================= */

float cal_poly_coeff[NUM_STRINGS][3] = { {0} };

/* =========================================================================
 * Calibration private types / state
 * ========================================================================= */

typedef enum {
    CAL_PHASE_IDLE = 0,
    CAL_PHASE_STR_INIT,
    CAL_PHASE_NEXT_FRET,
    CAL_PHASE_WAIT_MOVE,
    CAL_PHASE_WAIT,
    CAL_PHASE_EXCITE,
    CAL_PHASE_BURST_REENABLE,
    CAL_PHASE_WAIT_PLUCK,
    CAL_PHASE_LOCK,
    CAL_PHASE_RECORD,
    CAL_PHASE_BUILD,
    CAL_PHASE_NEXT_STRING,
    CAL_PHASE_DONE,
    CAL_PHASE_ERROR
} CalPhase_t;

typedef struct {
    float steps;         /**< Recorded motor steps for this fret          */
    float freq;          /**< Target frequency used for polynomial / LUT  */
    float measured_freq; /**< Last locked FFT frequency (diagnostic only) */
} CalPoint_t;

static CalPhase_t  s_phase         = CAL_PHASE_IDLE;
static CalStatus_t s_status        = CAL_STATUS_IDLE;
static uint8_t     s_string_mask   = 0u;
static uint8_t     s_str           = 0u;
static uint8_t     s_fret          = 0u;
static float       s_target_freq   = 0.0f;
static int32_t     s_steps_expected = 0;
static float       s_last_valid_freq = 0.0f;
static float       s_pi_integral   = 0.0f;
static int32_t     s_pi_cmd_steps  = 0;
static uint8_t     s_lock_consec   = 0u;
static uint8_t     s_no_sig_cnt    = 0u;
static uint16_t    s_wait_cnt      = 0u;
static CalPhase_t  s_after_wait    = CAL_PHASE_IDLE;
static CalPhase_t  s_after_move    = CAL_PHASE_IDLE;
static uint16_t    s_manual_prompt_hops = 0u;
static CalPoint_t  s_buf[CAL_NUM_FRETS];
static uint16_t    s_buf_n         = 0u;

/* =========================================================================
 * Shared helpers
 * ========================================================================= */

static inline float CalClampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void WaitThen(uint16_t hops, CalPhase_t next)
{
    s_wait_cnt   = hops;
    s_after_wait = next;
    s_phase      = CAL_PHASE_WAIT;
}

static void StopSolenoid(uint8_t str_id)
{
    g_shared_memory.solenoid_id          = str_id;
    g_shared_memory.motor_command_flags |= CMD_STOP_SOLENOID;
    InterCore_NotifyCM4();
}

static void TrackClosestFreq(float center_freq,
                             float min_freq,
                             float max_freq,
                             float *out_freq)
{
    float best_freq = 0.0f;
    float best_mag  = 0.0f;
    float best_err  = 1.0e9f;

    for (uint8_t i = 0u; i < active_string_count; i++) {
        float freq = active_frequencies[i];
        float mag  = active_magnitudes[i];

        if (freq < min_freq || freq > max_freq || mag <= 50.0f) {
            continue;
        }

        float err = fabsf(freq - center_freq);
        if (best_freq == 0.0f || err < best_err
            || (fabsf(err - best_err) < 0.01f && mag > best_mag)) {
            best_freq = freq;
            best_mag  = mag;
            best_err  = err;
        }
    }

    *out_freq = best_freq;
}

static float StepCapForError(float abs_error_hz)
{
    if (abs_error_hz > 40.0f) return TUNE_MAX_STEP_DELTA;
    if (abs_error_hz > 15.0f) return 400.0f;
    if (abs_error_hz > 5.0f)  return 160.0f;
    return 80.0f;
}

/* =========================================================================
 * Calibration helpers
 * ========================================================================= */

static uint32_t CalChecksum(const CalPersist_t *p)
{
    const uint32_t *w          = (const uint32_t *)p;
    uint32_t        size_words = (sizeof(CalPersist_t) / sizeof(uint32_t)) - 2u;
    uint32_t        csum       = 0u;

    for (uint32_t i = 1u; i <= size_words; i++) {
        csum ^= w[i];
    }

    return csum;
}

static void Cal_EnableBkpSRAM(void)
{
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPRAM_CLK_ENABLE();

    if (HAL_PWREx_EnableBkUpReg() != HAL_OK) {
        printf("[CAL] ERROR: backup regulator timeout\r\n");
    }
}

static uint8_t CalStringHasSolenoid(uint8_t str)
{
    return (str <= 2u) ? 1u : 0u;
}

static float CalTargetFreq(uint8_t str, uint8_t fret)
{
    float open_f = s_open_freq[TuningCtrl_GetActiveTuning()][str];
    return open_f * powf(2.0f, (float)fret / 12.0f);
}

static int32_t CalEstimateSteps(uint8_t str, uint8_t fret)
{
    float estimate = tuning_ff_steps_bp[str][fret];

    if (fret > 0u && s_buf_n >= (uint16_t)fret) {
        float prev_measured = s_buf[fret - 1u].steps;
        float prev_seed     = tuning_ff_steps_bp[str][fret - 1u];
        float curr_seed     = tuning_ff_steps_bp[str][fret];
        estimate = prev_measured + (curr_seed - prev_seed);
    }

    return (int32_t)CalClampf(estimate, 0.0f, (float)s_cal_steps_max[str]);
}

static void CalResetLockState(float seed_freq)
{
    s_pi_integral    = 0.0f;
    s_pi_cmd_steps   = g_shared_memory.current_positions[s_str];
    s_lock_consec    = 0u;
    s_no_sig_cnt     = 0u;
    s_last_valid_freq = seed_freq;
}

static void CalGotoThen(int32_t position, CalPhase_t next)
{
    InterCore_SendMotorGoto(s_str, position);
    s_after_move = next;
    s_phase      = CAL_PHASE_WAIT_MOVE;
}

static uint8_t CalAdvanceToNextString(void)
{
    do {
        s_str++;
    } while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u));

    return (s_str < NUM_STRINGS) ? 1u : 0u;
}

static void CalStartExcitation(void)
{
    float excite_freq = (s_last_valid_freq > 0.0f) ? s_last_valid_freq : s_target_freq;

    CalResetLockState(excite_freq);

    if (CalStringHasSolenoid(s_str) && excite_freq >= 40.0f && excite_freq <= 800.0f) {
        printf("[CAL] str%u fret%u: excite %.2f Hz near %ld steps\r\n",
               (unsigned)s_str,
               (unsigned)s_fret,
               (double)excite_freq,
               (long)g_shared_memory.current_positions[s_str]);

        StopSolenoid(s_str);
        fft_sensing_enabled = 0u;

        InterCore_TriggerSolenoidHz(s_str,
                                    excite_freq,
                                    CAL_SOLENOID_DUTY,
                                    SOLENOID_MODE_TIMED,
                                    CAL_REEXCITE_DURATION_MS,
                                    0u);

        WaitThen(CAL_REEXCITE_WAIT_HOPS, CAL_PHASE_BURST_REENABLE);
    } else {
        printf("[CAL] str%u fret%u: pluck %s now (target %.2f Hz)\r\n",
               (unsigned)s_str,
               (unsigned)s_fret,
               s_string_names[TuningCtrl_GetActiveTuning()][s_str],
               (double)s_target_freq);
        s_manual_prompt_hops = 0u;
        s_phase = CAL_PHASE_WAIT_PLUCK;
    }
}

/* =========================================================================
 * LUT and polynomial builders
 * ========================================================================= */

static void Cal_BuildTable(uint8_t str)
{
    if (s_buf_n < CAL_NUM_FRETS) {
        printf("[CAL] str%u WARN: only %u/%u fret points recorded — LUT unchanged\r\n",
               (unsigned)str,
               (unsigned)s_buf_n,
               (unsigned)CAL_NUM_FRETS);
        return;
    }

    for (uint8_t bp = 0u; bp < CAL_NUM_FRETS; bp++) {
        tuning_ff_freq_bp[str][bp]  = s_buf[bp].freq;
        tuning_ff_steps_bp[str][bp] = s_buf[bp].steps;

        printf("[CAL-LUT] %u, %u, %.2f, %.0f\r\n",
               (unsigned)str,
               (unsigned)bp,
               (double)s_buf[bp].freq,
               (double)s_buf[bp].steps);
    }
}

static void Cal_FitPolynomial(uint8_t str)
{
    if (s_buf_n < 3u) {
        cal_poly_coeff[str][0] = 0.0f;
        cal_poly_coeff[str][1] = 10.0f;
        cal_poly_coeff[str][2] = 0.0f;
        printf("[CAL-POLY] str%u: too few samples, linear fallback\r\n",
               (unsigned)str);
        return;
    }

    double S0 = 0.0;
    double S1 = 0.0;
    double S2 = 0.0;
    double S3 = 0.0;
    double S4 = 0.0;
    double T0 = 0.0;
    double T1 = 0.0;
    double T2 = 0.0;

    for (uint16_t i = 0u; i < s_buf_n; i++) {
        double x  = (double)s_buf[i].freq;
        double y  = (double)s_buf[i].steps;
        double x2 = x * x;

        S0 += 1.0;
        S1 += x;
        S2 += x2;
        S3 += x2 * x;
        S4 += x2 * x2;
        T0 += y;
        T1 += x * y;
        T2 += x2 * y;
    }

    double A[3][4] = {
        { S0, S1, S2, T0 },
        { S1, S2, S3, T1 },
        { S2, S3, S4, T2 }
    };

    for (int piv = 0; piv < 3; piv++) {
        int max_row = piv;

        for (int r = piv + 1; r < 3; r++) {
            if (fabs(A[r][piv]) > fabs(A[max_row][piv])) {
                max_row = r;
            }
        }

        for (int c = 0; c < 4; c++) {
            double tmp      = A[piv][c];
            A[piv][c]       = A[max_row][c];
            A[max_row][c]   = tmp;
        }

        if (fabs(A[piv][piv]) < 1e-12) {
            cal_poly_coeff[str][0] = 0.0f;
            cal_poly_coeff[str][1] = 10.0f;
            cal_poly_coeff[str][2] = 0.0f;
            printf("[CAL-POLY] str%u: singular matrix, linear fallback\r\n",
                   (unsigned)str);
            return;
        }

        for (int r = piv + 1; r < 3; r++) {
            double factor = A[r][piv] / A[piv][piv];
            for (int c = piv; c < 4; c++) {
                A[r][c] -= factor * A[piv][c];
            }
        }
    }

    double coeff[3];
    for (int i = 2; i >= 0; i--) {
        coeff[i] = A[i][3];
        for (int j = i + 1; j < 3; j++) {
            coeff[i] -= A[i][j] * coeff[j];
        }
        coeff[i] /= A[i][i];
    }

    cal_poly_coeff[str][0] = (float)coeff[0];
    cal_poly_coeff[str][1] = (float)coeff[1];
    cal_poly_coeff[str][2] = (float)coeff[2];

    printf("[CAL-POLY] str%u: a=%.3f b=%.4f c=%.6f\r\n",
           (unsigned)str,
           (double)coeff[0],
           (double)coeff[1],
           (double)coeff[2]);
}

static void Cal_FitSemiPolynomial(uint8_t str)
{
    if (s_buf_n < 3u) {
        printf("[CAL-SEMI] str%u: too few samples for semitone fit\r\n",
               (unsigned)str);
        return;
    }

    {
        float open_f = s_open_freq[TuningCtrl_GetActiveTuning()][str];

        /* Feedforward: steps = a*s^2 + b*s + c */
        {
            double S0 = 0.0;
            double S1 = 0.0;
            double S2 = 0.0;
            double S3 = 0.0;
            double S4 = 0.0;
            double T0 = 0.0;
            double T1 = 0.0;
            double T2 = 0.0;
            double A[3][4];
            uint8_t ok = 1u;

            for (uint16_t i = 0u; i < s_buf_n; i++) {
                double semi = 12.0 * log2((double)s_buf[i].freq / (double)open_f);
                double y = (double)s_buf[i].steps;
                double x2 = semi * semi;

                S0 += 1.0;
                S1 += semi;
                S2 += x2;
                S3 += x2 * semi;
                S4 += x2 * x2;
                T0 += y;
                T1 += semi * y;
                T2 += x2 * y;
            }

            A[0][0] = S0; A[0][1] = S1; A[0][2] = S2; A[0][3] = T0;
            A[1][0] = S1; A[1][1] = S2; A[1][2] = S3; A[1][3] = T1;
            A[2][0] = S2; A[2][1] = S3; A[2][2] = S4; A[2][3] = T2;

            for (int piv = 0; piv < 3; piv++) {
                int max_row = piv;
                for (int r = piv + 1; r < 3; r++) {
                    if (fabs(A[r][piv]) > fabs(A[max_row][piv])) {
                        max_row = r;
                    }
                }
                for (int c = 0; c < 4; c++) {
                    double tmp = A[piv][c];
                    A[piv][c] = A[max_row][c];
                    A[max_row][c] = tmp;
                }
                if (fabs(A[piv][piv]) < 1e-12) {
                    ok = 0u;
                    break;
                }
                for (int r = piv + 1; r < 3; r++) {
                    double factor = A[r][piv] / A[piv][piv];
                    for (int c = piv; c < 4; c++) {
                        A[r][c] -= factor * A[piv][c];
                    }
                }
            }

            if (ok) {
                double coeff[3];
                for (int i = 2; i >= 0; i--) {
                    coeff[i] = A[i][3];
                    for (int j = i + 1; j < 3; j++) {
                        coeff[i] -= A[i][j] * coeff[j];
                    }
                    coeff[i] /= A[i][i];
                }

                tuning_ff_semi_coeff[str][0] = (float)coeff[2];
                tuning_ff_semi_coeff[str][1] = (float)coeff[1];
                tuning_ff_semi_coeff[str][2] = (float)coeff[0];

                printf("[CAL-SEMI-FF] str%u: steps = %.4f·s² + %.3f·s + %.3f\r\n",
                       (unsigned)str,
                       coeff[2],
                       coeff[1],
                       coeff[0]);
            } else {
                printf("[CAL-SEMI-FF] str%u: singular — keeping existing\r\n",
                       (unsigned)str);
            }
        }

        /* Plant: semi = a*steps^2 + b*steps + c */
        {
            double S0 = 0.0;
            double S1 = 0.0;
            double S2 = 0.0;
            double S3 = 0.0;
            double S4 = 0.0;
            double T0 = 0.0;
            double T1 = 0.0;
            double T2 = 0.0;
            double A[3][4];
            uint8_t ok = 1u;

            for (uint16_t i = 0u; i < s_buf_n; i++) {
                double x = (double)s_buf[i].steps;
                double semi = 12.0 * log2((double)s_buf[i].freq / (double)open_f);
                double x2 = x * x;

                S0 += 1.0;
                S1 += x;
                S2 += x2;
                S3 += x2 * x;
                S4 += x2 * x2;
                T0 += semi;
                T1 += x * semi;
                T2 += x2 * semi;
            }

            A[0][0] = S0; A[0][1] = S1; A[0][2] = S2; A[0][3] = T0;
            A[1][0] = S1; A[1][1] = S2; A[1][2] = S3; A[1][3] = T1;
            A[2][0] = S2; A[2][1] = S3; A[2][2] = S4; A[2][3] = T2;

            for (int piv = 0; piv < 3; piv++) {
                int max_row = piv;
                for (int r = piv + 1; r < 3; r++) {
                    if (fabs(A[r][piv]) > fabs(A[max_row][piv])) {
                        max_row = r;
                    }
                }
                for (int c = 0; c < 4; c++) {
                    double tmp = A[piv][c];
                    A[piv][c] = A[max_row][c];
                    A[max_row][c] = tmp;
                }
                if (fabs(A[piv][piv]) < 1e-12) {
                    ok = 0u;
                    break;
                }
                for (int r = piv + 1; r < 3; r++) {
                    double factor = A[r][piv] / A[piv][piv];
                    for (int c = piv; c < 4; c++) {
                        A[r][c] -= factor * A[piv][c];
                    }
                }
            }

            if (ok) {
                double coeff[3];
                for (int i = 2; i >= 0; i--) {
                    coeff[i] = A[i][3];
                    for (int j = i + 1; j < 3; j++) {
                        coeff[i] -= A[i][j] * coeff[j];
                    }
                    coeff[i] /= A[i][i];
                }

                tuning_plant_coeff[str][0] = (float)coeff[2];
                tuning_plant_coeff[str][1] = (float)coeff[1];
                tuning_plant_coeff[str][2] = (float)coeff[0];

                printf("[CAL-SEMI-PLANT] str%u: semi = %.4e·s² + %.4e·s + %.4f\r\n",
                       (unsigned)str,
                       coeff[2],
                       coeff[1],
                       coeff[0]);
            } else {
                printf("[CAL-SEMI-PLANT] str%u: singular — keeping existing\r\n",
                       (unsigned)str);
            }
        }
    }
}

/* =========================================================================
 * Persistent storage
 * ========================================================================= */

static void Cal_SaveToSRAM(void)
{
    Cal_EnableBkpSRAM();

    CalPersist_t *p = (CalPersist_t *)CAL_BKPSRAM_BASE;

    p->magic = CAL_PERSIST_MAGIC;
    memcpy(p->ff_freq,  tuning_ff_freq_bp,  sizeof(tuning_ff_freq_bp));
    memcpy(p->ff_steps, tuning_ff_steps_bp, sizeof(tuning_ff_steps_bp));

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        p->poly_a[i] = cal_poly_coeff[i][0];
        p->poly_b[i] = cal_poly_coeff[i][1];
        p->poly_c[i] = cal_poly_coeff[i][2];
    }

    memcpy(p->semi_ff, tuning_ff_semi_coeff, sizeof(tuning_ff_semi_coeff));
    memcpy(p->semi_plant, tuning_plant_coeff, sizeof(tuning_plant_coeff));

    __DSB();
    p->checksum = CalChecksum(p);
    __DSB();

    printf("[CAL] Saved to backup SRAM (0x%08lX, %u bytes)\r\n",
           (unsigned long)CAL_BKPSRAM_BASE,
           (unsigned)sizeof(CalPersist_t));
}

uint8_t Cal_LoadFromSRAM(void)
{
    Cal_EnableBkpSRAM();

    const CalPersist_t *p = (const CalPersist_t *)CAL_BKPSRAM_BASE;

    if (p->magic != CAL_PERSIST_MAGIC) {
        printf("[CAL] No stored calibration (magic 0x%08lX)\r\n",
               (unsigned long)p->magic);
        return 0u;
    }

    {
        uint32_t expected = CalChecksum(p);
        if (p->checksum != expected) {
            printf("[CAL] Checksum mismatch (got 0x%08lX expected 0x%08lX) — ignored\r\n",
                   (unsigned long)p->checksum,
                   (unsigned long)expected);
            return 0u;
        }
    }

    memcpy(tuning_ff_freq_bp,  p->ff_freq,  sizeof(tuning_ff_freq_bp));
    memcpy(tuning_ff_steps_bp, p->ff_steps, sizeof(tuning_ff_steps_bp));

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        cal_poly_coeff[i][0] = p->poly_a[i];
        cal_poly_coeff[i][1] = p->poly_b[i];
        cal_poly_coeff[i][2] = p->poly_c[i];
    }

    /* Semitone-domain coefficients — check for non-zero to detect old SRAM
     * layouts that don't have these fields (they'd read as all-zero). */
    {
        uint8_t has_semi = 0u;
        for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
            if (p->semi_ff[i][0] != 0.0f || p->semi_ff[i][1] != 0.0f ||
                p->semi_ff[i][2] != 0.0f) {
                has_semi = 1u;
                break;
            }
        }
        if (has_semi) {
            memcpy(tuning_ff_semi_coeff, p->semi_ff,    sizeof(tuning_ff_semi_coeff));
            memcpy(tuning_plant_coeff,   p->semi_plant, sizeof(tuning_plant_coeff));
            printf("[CAL] Semitone-domain coefficients restored.\r\n");
        } else {
            printf("[CAL] No semitone coefficients in SRAM — using defaults.\r\n");
        }
    }

    printf("[CAL] Calibration restored from backup SRAM.\r\n");
    return 1u;
}

/* =========================================================================
 * Public API — calibration
 * ========================================================================= */

void Cal_FitFromBaseLUT(void)
{
    for (uint8_t str = 0u; str < NUM_STRINGS; str++) {
        s_buf_n = TUNING_FF_NUM_POINTS;
        for (uint8_t i = 0u; i < TUNING_FF_NUM_POINTS; i++) {
            s_buf[i].freq          = tuning_ff_freq_bp[str][i];
            s_buf[i].steps         = tuning_ff_steps_bp[str][i];
            s_buf[i].measured_freq = tuning_ff_freq_bp[str][i];
        }
        Cal_FitPolynomial(str);
        Cal_FitSemiPolynomial(str);
    }

    s_buf_n = 0u;
}

void Cal_Start(uint8_t string_mask)
{
    uint8_t requested_mask = string_mask & 0x3Fu;
    uint8_t supported_mask = requested_mask & CAL_CONFIGURED_STRING_MASK;

    if (requested_mask == 0u) {
        printf("[CAL] ERROR: empty string mask.\r\n");
        s_status = CAL_STATUS_ERROR;
        s_phase  = CAL_PHASE_ERROR;
        return;
    }

    if (supported_mask == 0u) {
        printf("[CAL] ERROR: no configured calibration motors in mask 0x%02X\r\n",
               (unsigned)requested_mask);
        printf("[CAL] Configured calibration motors: 0x%02X (strings 0-3)\r\n",
               (unsigned)CAL_CONFIGURED_STRING_MASK);
        s_status = CAL_STATUS_ERROR;
        s_phase  = CAL_PHASE_ERROR;
        return;
    }

    if (supported_mask != requested_mask) {
        printf("[CAL] WARNING: ignoring unconfigured strings, using mask 0x%02X\r\n",
               (unsigned)supported_mask);
    }

    s_string_mask = supported_mask;
    s_str = 0u;
    while (s_str < NUM_STRINGS && !((s_string_mask >> s_str) & 1u)) {
        s_str++;
    }

    if (Tune_ClearOffsetsMask(s_string_mask)) {
        printf("[CAL] Cleared stale tune offsets for mask 0x%02X\r\n",
               (unsigned)s_string_mask);
    }

    fft_sensing_enabled = 1u;
    s_status = CAL_STATUS_RUNNING;
    s_phase  = CAL_PHASE_STR_INIT;

    printf("[CAL] ================================================\r\n");
    printf("[CAL] Feed-forward calibration started — mask 0x%02X\r\n",
           (unsigned)s_string_mask);
    printf("[CAL] Seed source: active tuning_ff tables (SRAM if loaded, otherwise hardcoded)\r\n");
    printf("[CAL] Per-fret flow: LUT seed -> settle -> excite/pluck -> tune-style lock -> record\r\n");
    printf("[CAL] Output: str, fret, steps, measured_hz, target_hz\r\n");
    printf("[CAL] ================================================\r\n");
}

void Cal_StartStr(uint8_t str_id)
{
    if (str_id >= NUM_STRINGS) {
        printf("[CAL] ERROR: invalid string ID %u\r\n", (unsigned)str_id);
        return;
    }

    Cal_Start((uint8_t)(1u << str_id));
}

void Cal_StartAll(void)
{
    Cal_Start(0x3Fu);
}

CalStatus_t Cal_GetStatus(void)
{
    return s_status;
}

void Cal_Hop(void)
{
    switch (s_phase) {

    case CAL_PHASE_STR_INIT:
    {
        float open_f = CalTargetFreq(s_str, 0u);

        InterCore_SendMotorEnableMask((uint8_t)(1u << s_str));
        InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED);
        StopSolenoid(s_str);

        memset(s_buf, 0, sizeof(s_buf));
        s_buf[0].freq          = open_f;
        s_buf[0].measured_freq = open_f;
        s_buf[0].steps         = 0.0f;
        s_buf_n                = 1u;

        s_fret           = 1u;
        s_target_freq    = 0.0f;
        s_steps_expected = 0;
        s_manual_prompt_hops = 0u;
        CalResetLockState(0.0f);

        printf("\r\n[CAL] ---- String %u (%s) ----\r\n",
               (unsigned)s_str,
               s_string_names[TuningCtrl_GetActiveTuning()][s_str]);
        printf("[CAL] Fret 0 anchor: 0 steps @ %.2f Hz\r\n", (double)open_f);
        printf("[CAL] Re-excitation: %s\r\n",
               CalStringHasSolenoid(s_str) ? "solenoid" : "manual pluck");

        CalGotoThen(0, (CAL_NUM_FRETS > 1u) ? CAL_PHASE_NEXT_FRET : CAL_PHASE_BUILD);
        break;
    }

    case CAL_PHASE_NEXT_FRET:
    {
        if (s_fret >= CAL_NUM_FRETS) {
            s_phase = CAL_PHASE_BUILD;
            break;
        }

        s_target_freq    = CalTargetFreq(s_str, s_fret);
        s_steps_expected = CalEstimateSteps(s_str, s_fret);

        printf("[CAL] str%u fret%u: FF seed %ld steps -> %.2f Hz\r\n",
               (unsigned)s_str,
               (unsigned)s_fret,
               (long)s_steps_expected,
               (double)s_target_freq);

        CalGotoThen(s_steps_expected, CAL_PHASE_EXCITE);
        break;
    }

    case CAL_PHASE_WAIT_MOVE:
    {
        if (!g_shared_memory.motor_is_moving[s_str]) {
            CalResetLockState(0.0f);
            WaitThen(CAL_MOVE_SETTLE_HOPS, s_after_move);
        }
        break;
    }

    case CAL_PHASE_WAIT:
    {
        if (s_wait_cnt > 0u) {
            s_wait_cnt--;
        } else {
            s_phase = s_after_wait;
        }
        break;
    }

    case CAL_PHASE_EXCITE:
    {
        CalStartExcitation();
        break;
    }

    case CAL_PHASE_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_phase = CAL_PHASE_LOCK;
        break;
    }

    case CAL_PHASE_WAIT_PLUCK:
    {
        float center_freq = (s_last_valid_freq > 0.0f) ? s_last_valid_freq : s_target_freq;
        float min_freq    = s_target_freq * TUNE_TRACK_MIN_RATIO;
        float max_freq    = s_target_freq * TUNE_TRACK_MAX_RATIO;
        float meas_f      = 0.0f;

        TrackClosestFreq(center_freq, min_freq, max_freq, &meas_f);

        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;
            s_no_sig_cnt      = 0u;
            s_lock_consec     = 0u;
            printf("[CAL] str%u fret%u: detected %.2f Hz — locking\r\n",
                   (unsigned)s_str,
                   (unsigned)s_fret,
                   (double)meas_f);
            s_phase = CAL_PHASE_LOCK;
        } else {
            s_manual_prompt_hops++;
            if ((s_manual_prompt_hops % CAL_PLUCK_PROMPT_HOPS) == 0u) {
                printf("[CAL] str%u fret%u: still waiting for pluck near %.2f Hz\r\n",
                       (unsigned)s_str,
                       (unsigned)s_fret,
                       (double)s_target_freq);
            }
        }
        break;
    }

    case CAL_PHASE_LOCK:
    {
        float center_freq = (s_last_valid_freq > 0.0f) ? s_last_valid_freq : s_target_freq;
        float min_freq    = s_target_freq * TUNE_TRACK_MIN_RATIO;
        float max_freq    = s_target_freq * TUNE_TRACK_MAX_RATIO;
        float meas_f      = 0.0f;
        uint8_t motor_busy = g_shared_memory.motor_is_moving[s_str];

        TrackClosestFreq(center_freq, min_freq, max_freq, &meas_f);

        if (meas_f > 0.0f) {
            float error = s_target_freq - meas_f;

            s_last_valid_freq = meas_f;
            s_no_sig_cnt      = 0u;

            if (fabsf(error) > TUNE_LOCK_BAND_HZ) {
                if (fabsf(error) <= TUNE_INTEGRAL_ENABLE_BAND_HZ) {
                    s_pi_integral = CalClampf(
                        s_pi_integral + TUNE_PI_KI * error * TUNING_SAMPLE_TIME_S,
                        -TUNE_PI_MAX_TRIM,
                        TUNE_PI_MAX_TRIM);
                } else {
                    s_pi_integral = 0.0f;
                }

                if (!motor_busy) {
                    float step_cap = StepCapForError(fabsf(error));
                    float delta_cmd = CalClampf(TUNE_PI_KP * error + s_pi_integral,
                                                -step_cap,
                                                step_cap);
                    int32_t cmd = (int32_t)CalClampf(
                        (float)s_pi_cmd_steps + delta_cmd,
                        0.0f,
                        (float)s_cal_steps_max[s_str]);

                    int32_t delta = cmd - s_pi_cmd_steps;
                    if ((delta < 0 ? -delta : delta) >= (int32_t)TUNE_MIN_MOVE_STEPS) {
                        s_pi_cmd_steps = cmd;
                        InterCore_SendMotorGoto(s_str, s_pi_cmd_steps);
                    }
                }

                s_lock_consec = 0u;
            } else if (!motor_busy) {
                s_lock_consec++;
            }

            if (s_lock_consec >= CAL_LOCK_CONFIRM_HOPS) {
                printf("[CAL] str%u fret%u: confirmed at %.2f Hz (target %.2f Hz) pos=%ld\r\n",
                       (unsigned)s_str,
                       (unsigned)s_fret,
                       (double)meas_f,
                       (double)s_target_freq,
                       (long)g_shared_memory.current_positions[s_str]);
                s_phase = CAL_PHASE_RECORD;
            }
        } else if (!motor_busy) {
            s_no_sig_cnt++;

            if (s_no_sig_cnt >= CAL_REEXCITE_TRIGGER_HOPS) {
                printf("[CAL] str%u fret%u: signal lost — reacquiring\r\n",
                       (unsigned)s_str,
                       (unsigned)s_fret);
                s_phase = CAL_PHASE_EXCITE;
            }
        }
        break;
    }

    case CAL_PHASE_RECORD:
    {
        int32_t actual_steps = g_shared_memory.current_positions[s_str];
        float measured_freq  = (s_last_valid_freq > 0.0f) ? s_last_valid_freq : s_target_freq;

        s_buf[s_fret].freq          = s_target_freq;
        s_buf[s_fret].measured_freq = measured_freq;
        s_buf[s_fret].steps         = (float)actual_steps;

        if (s_buf_n < (uint16_t)(s_fret + 1u)) {
            s_buf_n = (uint16_t)(s_fret + 1u);
        }

        printf("[CAL] %u, %u, %ld, %.2f, %.2f\r\n",
               (unsigned)s_str,
               (unsigned)s_fret,
               (long)actual_steps,
               (double)measured_freq,
               (double)s_target_freq);

        s_fret++;
        s_phase = (s_fret >= CAL_NUM_FRETS) ? CAL_PHASE_BUILD : CAL_PHASE_NEXT_FRET;
        break;
    }

    case CAL_PHASE_BUILD:
    {
        Cal_BuildTable(s_str);
        Cal_FitPolynomial(s_str);
        Cal_FitSemiPolynomial(s_str);
        Cal_SaveToSRAM();

        StopSolenoid(s_str);
        CalGotoThen(0, CAL_PHASE_NEXT_STRING);
        break;
    }

    case CAL_PHASE_NEXT_STRING:
    {
        StopSolenoid(s_str);

        if (CalAdvanceToNextString()) {
            s_phase = CAL_PHASE_STR_INIT;
        } else {
            fft_sensing_enabled = 1u;
            printf("[CAL] ================================================\r\n");
            printf("[CAL] Calibration complete — LUT + polynomial saved.\r\n");
            printf("[CAL] ================================================\r\n");
            s_status = CAL_STATUS_DONE;
            s_phase  = CAL_PHASE_DONE;
        }
        break;
    }

    case CAL_PHASE_IDLE:
    case CAL_PHASE_DONE:
    case CAL_PHASE_ERROR:
    default:
        break;
    }
}

/* =========================================================================
 * Tune controller — open-string pitch correction (bidirectional)
 * ========================================================================= */

typedef enum {
    TUNE_PHASE_IDLE = 0,
    TUNE_PHASE_STR_INIT,
    TUNE_PHASE_WAIT_HOME,
    TUNE_PHASE_WAIT_PLUCK,
    TUNE_PHASE_MOTOR_MOVE,
    TUNE_PHASE_WAIT_MOTOR,
    TUNE_PHASE_WAIT,
    TUNE_PHASE_SOL_BURST,
    TUNE_PHASE_BURST_REENABLE,
    TUNE_PHASE_PI_LOCK,
    TUNE_PHASE_RECORD,
    TUNE_PHASE_DONE,
    TUNE_PHASE_ERROR
} TunePhase_t;

static TunePhase_t s_tune_phase       = TUNE_PHASE_IDLE;
static CalStatus_t s_tune_status      = CAL_STATUS_IDLE;
static uint8_t     s_tune_str         = 0u;
static uint8_t     s_tune_mask        = 0x0Fu;

static int32_t     s_tune_offset[NUM_STRINGS] = {0};

static uint8_t     s_tune_pluck_cnt   = 0u;
static float       s_tune_detected_f  = 0.0f;

static int32_t     s_tune_target_steps = 0;
static uint8_t     s_tune_motor_cnt   = 0u;

static float       s_tune_pi_integral    = 0.0f;
static int32_t     s_tune_pi_cmd_steps   = 0;
static uint8_t     s_tune_pi_hop_cnt     = 0u;
static uint8_t     s_tune_pi_lock_consec = 0u;
static float       s_tune_last_valid_f   = 0.0f;
static uint8_t     s_tune_no_sig_cnt     = 0u;
static uint8_t     s_tune_retry_cnt      = 0u;
static uint8_t     s_tune_locked         = 0u;

static uint16_t    s_tune_wait_cnt       = 0u;
static TunePhase_t s_tune_after_wait     = TUNE_PHASE_IDLE;

static void TuneWaitThen(uint16_t hops, TunePhase_t next)
{
    s_tune_wait_cnt   = hops;
    s_tune_after_wait = next;
    s_tune_phase      = TUNE_PHASE_WAIT;
}

static uint8_t Tune_ClearOffsetsMask(uint8_t string_mask)
{
    uint8_t changed = 0u;

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        if (((string_mask >> i) & 1u) && s_tune_offset[i] != 0) {
            s_tune_offset[i] = 0;
            changed = 1u;
        }
    }

    return changed;
}

void Cal_TuneStart(uint8_t string_mask)
{
    uint8_t requested_mask = string_mask & 0x3Fu;
    uint8_t supported_mask = requested_mask & TUNE_SUPPORTED_STRING_MASK;

    if (requested_mask == 0u) {
        printf("[TUNE] ERROR: empty string mask.\r\n");
        s_tune_status = CAL_STATUS_ERROR;
        return;
    }

    if (supported_mask == 0u) {
        printf("[TUNE] ERROR: tune currently supports only strings 0-3.\r\n");
        s_tune_status = CAL_STATUS_ERROR;
        return;
    }

    if (supported_mask != requested_mask) {
        printf("[TUNE] WARNING: strings 4-5 are not commissioned; using mask 0x%02X\r\n",
               (unsigned)supported_mask);
    }

    s_tune_mask = supported_mask;
    s_tune_str  = 0u;
    while (s_tune_str < NUM_STRINGS && !((s_tune_mask >> s_tune_str) & 1u)) {
        s_tune_str++;
    }

    s_tune_phase  = TUNE_PHASE_STR_INIT;
    s_tune_status = CAL_STATUS_RUNNING;

    printf("[TUNE] ================================================\r\n");
    printf("[TUNE] Open-string tuning started — mask 0x%02X\r\n",
           (unsigned)s_tune_mask);
    printf("[TUNE] ================================================\r\n");
}

CalStatus_t Cal_TuneGetStatus(void)
{
    return s_tune_status;
}

int32_t Cal_TuneGetOffset(uint8_t str)
{
    if (str >= NUM_STRINGS) {
        return 0;
    }
    return s_tune_offset[str];
}

void Cal_TuneClearOffsets(void)
{
    memset(s_tune_offset, 0, sizeof(s_tune_offset));
    printf("[TUNE] Offsets cleared.\r\n");
}

uint8_t Cal_TuneLoadFromSRAM(void)
{
    memset(s_tune_offset, 0, sizeof(s_tune_offset));
    printf("[TUNE] Tune-offset persistence disabled — starting with zero offsets.\r\n");
    return 0u;
}

void Cal_TuneHop(void)
{
    switch (s_tune_phase) {

    case TUNE_PHASE_STR_INIT:
    {
        int32_t cur = g_shared_memory.current_positions[s_tune_str];

        InterCore_SendMotorEnableMask((uint8_t)(1u << s_tune_str));
        InterCore_SendMotorMaxSpeed(s_tune_str, TUNE_MOTOR_SPEED);

        s_tune_pluck_cnt      = 0u;
        s_tune_detected_f     = 0.0f;
        s_tune_motor_cnt      = 0u;
        s_tune_pi_integral    = 0.0f;
        s_tune_pi_cmd_steps   = cur;
        s_tune_pi_hop_cnt     = 0u;
        s_tune_pi_lock_consec = 0u;
        s_tune_last_valid_f   = 0.0f;
        s_tune_no_sig_cnt     = 0u;
        s_tune_retry_cnt      = 0u;
        s_tune_target_steps   = cur;
        s_tune_locked         = 0u;

        printf("[TUNE] -- String %u (target %.2f Hz) starting at pos=%ld\r\n",
               (unsigned)s_tune_str,
               (double)s_open_freq[TuningCtrl_GetActiveTuning()][s_tune_str],
               (long)cur);
        printf("[TUNE] >>> Pluck string %u now!\r\n",
               (unsigned)s_tune_str);

        s_tune_phase = TUNE_PHASE_WAIT_PLUCK;
        break;
    }

    case TUNE_PHASE_WAIT:
    {
        if (s_tune_wait_cnt > 0u) {
            s_tune_wait_cnt--;
        } else {
            s_tune_phase = s_tune_after_wait;
        }
        break;
    }

    case TUNE_PHASE_WAIT_PLUCK:
    {
        float open_f    = s_open_freq[TuningCtrl_GetActiveTuning()][s_tune_str];
        float best_freq = 0.0f;

        TrackClosestFreq(open_f, open_f * 0.70f, open_f * 1.80f, &best_freq);

        if (best_freq > 0.0f) {
            s_tune_detected_f = best_freq;
            printf("[TUNE] str%u: pluck detected at %.2f Hz\r\n",
                   (unsigned)s_tune_str,
                   (double)s_tune_detected_f);
            s_tune_phase = TUNE_PHASE_MOTOR_MOVE;
        }

        if (s_tune_phase == TUNE_PHASE_WAIT_PLUCK) {
            s_tune_pluck_cnt++;
            if (s_tune_pluck_cnt >= TUNE_PLUCK_TIMEOUT) {
                printf("[TUNE] str%u: no pluck detected — skipping\r\n",
                       (unsigned)s_tune_str);
                s_tune_phase = TUNE_PHASE_RECORD;
            }
        }
        break;
    }

    case TUNE_PHASE_MOTOR_MOVE:
    {
        float target      = s_open_freq[TuningCtrl_GetActiveTuning()][s_tune_str];
        float measured    = s_tune_detected_f;
        float delta_f     = target - measured;
        int32_t current_pos = g_shared_memory.current_positions[s_tune_str];

        printf("[TUNE] str%u: measured=%.2f Hz target=%.2f Hz delta=%.2f Hz "
               "-> closed-loop engage at %ld steps\r\n",
               (unsigned)s_tune_str,
               (double)measured,
               (double)target,
               (double)delta_f,
               (long)current_pos);

        if (fabsf(delta_f) < TUNE_LOCK_BAND_HZ) {
            s_tune_last_valid_f = measured;
            s_tune_locked       = 1u;
            printf("[TUNE] str%u: already in tune!\r\n",
                   (unsigned)s_tune_str);
            s_tune_phase = TUNE_PHASE_RECORD;
            break;
        }

        s_tune_last_valid_f = measured;
        s_tune_pi_cmd_steps = current_pos;
        s_tune_target_steps = current_pos;
        s_tune_pi_integral  = 0.0f;
        s_tune_pi_hop_cnt   = 0u;
        s_tune_no_sig_cnt   = 0u;
        s_tune_retry_cnt    = 0u;

        fft_sensing_enabled = 0u;
        TuneWaitThen(TUNE_REEXCITE_WAIT_HOPS, TUNE_PHASE_BURST_REENABLE);
        break;
    }

    case TUNE_PHASE_WAIT_MOTOR:
    {
        uint8_t arrived = !g_shared_memory.motor_is_moving[s_tune_str];
        s_tune_motor_cnt++;

        if (arrived || s_tune_motor_cnt >= TUNE_MOTOR_WAIT_HOPS) {
            if (s_tune_str <= 2u) {
                s_tune_phase = TUNE_PHASE_SOL_BURST;
            } else {
                TuneWaitThen(2u, TUNE_PHASE_PI_LOCK);
            }
        }
        break;
    }

    case TUNE_PHASE_SOL_BURST:
    {
        float burst_freq = (s_tune_last_valid_f > 0.0f)
                         ? s_tune_last_valid_f
                         : ((s_tune_detected_f > 0.0f)
                              ? s_tune_detected_f
                              : s_open_freq[TuningCtrl_GetActiveTuning()][s_tune_str]);

        fft_sensing_enabled = 0u;

        if (burst_freq >= 40.0f && burst_freq <= 800.0f) {
            InterCore_TriggerSolenoidHz(s_tune_str,
                                        burst_freq,
                                        TUNE_SOLENOID_DUTY,
                                        SOLENOID_MODE_TIMED,
                                        TUNE_REEXCITE_DURATION_MS,
                                        0u);
        }

        s_tune_pi_integral    = 0.0f;
        s_tune_pi_cmd_steps   = g_shared_memory.current_positions[s_tune_str];
        s_tune_pi_hop_cnt     = 0u;
        s_tune_pi_lock_consec = 0u;
        s_tune_no_sig_cnt     = 0u;

        TuneWaitThen(TUNE_REEXCITE_WAIT_HOPS, TUNE_PHASE_BURST_REENABLE);
        break;
    }

    case TUNE_PHASE_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_tune_phase = TUNE_PHASE_PI_LOCK;
        break;
    }

    case TUNE_PHASE_PI_LOCK:
    {
        float target_freq = s_open_freq[TuningCtrl_GetActiveTuning()][s_tune_str];
        float center_freq = (s_tune_last_valid_f > 0.0f)
                          ? s_tune_last_valid_f
                          : ((s_tune_detected_f > 0.0f)
                               ? s_tune_detected_f
                               : target_freq);
        float min_freq = target_freq * TUNE_TRACK_MIN_RATIO;
        float max_freq = target_freq * TUNE_TRACK_MAX_RATIO;
        float meas_f   = 0.0f;
        uint8_t motor_busy = g_shared_memory.motor_is_moving[s_tune_str];

        TrackClosestFreq(center_freq, min_freq, max_freq, &meas_f);

        if (meas_f > 0.0f) {
            float error = target_freq - meas_f;

            s_tune_last_valid_f = meas_f;
            s_tune_no_sig_cnt   = 0u;

            if (fabsf(error) > TUNE_LOCK_BAND_HZ) {
                if (fabsf(error) <= TUNE_INTEGRAL_ENABLE_BAND_HZ) {
                    s_tune_pi_integral = CalClampf(
                        s_tune_pi_integral + TUNE_PI_KI * error * TUNING_SAMPLE_TIME_S,
                        -TUNE_PI_MAX_TRIM,
                        TUNE_PI_MAX_TRIM);
                } else {
                    s_tune_pi_integral = 0.0f;
                }

                if (!motor_busy) {
                    float step_cap = StepCapForError(fabsf(error));
                    float delta_cmd = CalClampf(TUNE_PI_KP * error + s_tune_pi_integral,
                                                -step_cap,
                                                step_cap);
                    int32_t cmd = (int32_t)CalClampf(
                        (float)s_tune_pi_cmd_steps + delta_cmd,
                        (float)(-s_cal_steps_max[s_tune_str]),
                        (float)s_cal_steps_max[s_tune_str]);

                    int32_t delta = cmd - s_tune_pi_cmd_steps;
                    if ((delta < 0 ? -delta : delta) >= (int32_t)TUNE_MIN_MOVE_STEPS) {
                        s_tune_pi_cmd_steps = cmd;
                        s_tune_target_steps = cmd;
                        InterCore_SendMotorGoto(s_tune_str, s_tune_pi_cmd_steps);
                    }
                }

                s_tune_pi_lock_consec = 0u;
            } else {
                s_tune_pi_lock_consec++;
            }

            if (s_tune_pi_lock_consec >= TUNE_LOCK_CONSEC_HOPS) {
                s_tune_locked = 1u;
                printf("[TUNE] str%u: LOCKED at %.2f Hz (err %.3f Hz) pos=%ld\r\n",
                       (unsigned)s_tune_str,
                       (double)meas_f,
                       (double)(target_freq - meas_f),
                       (long)g_shared_memory.current_positions[s_tune_str]);
                s_tune_phase = TUNE_PHASE_RECORD;
                break;
            }
        } else {
            if (!motor_busy) {
                s_tune_no_sig_cnt++;
            }

            if (s_tune_no_sig_cnt >= TUNE_DEAD_HOPS) {
                s_tune_retry_cnt++;

                if (s_tune_retry_cnt < TUNE_MAX_RETRIES) {
                    if (s_tune_str <= 2u) {
                        printf("[TUNE] str%u: signal lost — re-excite (retry %u/%u)\r\n",
                               (unsigned)s_tune_str,
                               (unsigned)s_tune_retry_cnt,
                               (unsigned)TUNE_MAX_RETRIES);
                        s_tune_phase = TUNE_PHASE_SOL_BURST;
                    } else {
                        printf("[TUNE] str%u: signal lost — pluck again! (retry %u/%u)\r\n",
                               (unsigned)s_tune_str,
                               (unsigned)s_tune_retry_cnt,
                               (unsigned)TUNE_MAX_RETRIES);
                        s_tune_pluck_cnt = 0u;
                        s_tune_phase = TUNE_PHASE_WAIT_PLUCK;
                    }
                } else {
                    printf("[TUNE] str%u: max retries reached — leaving offset unchanged\r\n",
                           (unsigned)s_tune_str);
                    s_tune_phase = TUNE_PHASE_RECORD;
                }
                break;
            }
        }

        if (!motor_busy) {
            s_tune_pi_hop_cnt++;
        }
        if (s_tune_pi_hop_cnt >= TUNE_PI_LOCK_TIMEOUT
            && s_tune_pi_lock_consec == 0u) {
            s_tune_retry_cnt++;

            if (s_tune_retry_cnt < TUNE_MAX_RETRIES) {
                if (s_tune_str <= 2u) {
                    printf("[TUNE] str%u: lock timeout — re-excite (retry %u/%u)\r\n",
                           (unsigned)s_tune_str,
                           (unsigned)s_tune_retry_cnt,
                           (unsigned)TUNE_MAX_RETRIES);
                    s_tune_phase = TUNE_PHASE_SOL_BURST;
                } else {
                    printf("[TUNE] str%u: lock timeout — pluck again! (retry %u/%u)\r\n",
                           (unsigned)s_tune_str,
                           (unsigned)s_tune_retry_cnt,
                           (unsigned)TUNE_MAX_RETRIES);
                    s_tune_pluck_cnt = 0u;
                    s_tune_phase = TUNE_PHASE_WAIT_PLUCK;
                }
            } else {
                printf("[TUNE] str%u: timeout after %u retries — leaving offset unchanged\r\n",
                       (unsigned)s_tune_str,
                       (unsigned)TUNE_MAX_RETRIES);
                s_tune_phase = TUNE_PHASE_RECORD;
            }
        }
        break;
    }

    case TUNE_PHASE_RECORD:
    {
        int32_t actual_pos = g_shared_memory.current_positions[s_tune_str];

        if (s_tune_locked) {
            s_tune_offset[s_tune_str] = actual_pos;
            printf("[TUNE] str%u: offset = %ld steps (pos=%ld)\r\n",
                   (unsigned)s_tune_str,
                   (long)s_tune_offset[s_tune_str],
                   (long)actual_pos);
        } else {
            printf("[TUNE] str%u: no confirmed lock — offset unchanged (%ld)\r\n",
                   (unsigned)s_tune_str,
                   (long)s_tune_offset[s_tune_str]);
        }

        do {
            s_tune_str++;
        } while (s_tune_str < NUM_STRINGS && !((s_tune_mask >> s_tune_str) & 1u));

        if (s_tune_str >= NUM_STRINGS) {
            fft_sensing_enabled = 1u;

            /* Zero motor positions (we're at open fret) and disable all */
            for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
                if ((s_tune_mask >> i) & 1u) {
                    InterCore_SendMotorZero(i);
                }
            }
            InterCore_SendMotorEnableMask(0u);

            printf("[TUNE] ================================================\r\n");
            printf("[TUNE] Tuning complete. Offsets:");
            for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
                if ((s_tune_mask >> i) & 1u) {
                    printf(" str%u=%ld", (unsigned)i, (long)s_tune_offset[i]);
                }
            }
            printf("\r\n");
            printf("[TUNE] Motors zeroed and disabled.\r\n");
            printf("[TUNE] ================================================\r\n");

            s_tune_status = CAL_STATUS_DONE;
            s_tune_phase  = TUNE_PHASE_IDLE;
        } else {
            s_tune_phase = TUNE_PHASE_STR_INIT;
        }
        break;
    }

    case TUNE_PHASE_DONE:
    {
        fft_sensing_enabled = 1u;

        /* Zero motor positions (we're at open fret) and disable all */
        for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
            if ((s_tune_mask >> i) & 1u) {
                InterCore_SendMotorZero(i);
            }
        }
        InterCore_SendMotorEnableMask(0u);

        printf("[TUNE] ================================================\r\n");
        printf("[TUNE] Tuning complete (via DONE phase). Offsets:");
        for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
            if ((s_tune_mask >> i) & 1u) {
                printf(" str%u=%ld", (unsigned)i, (long)s_tune_offset[i]);
            }
        }
        printf("\r\n");
        printf("[TUNE] Motors zeroed and disabled.\r\n");
        printf("[TUNE] ================================================\r\n");

        s_tune_status = CAL_STATUS_DONE;
        s_tune_phase  = TUNE_PHASE_IDLE;
        break;
    }

    case TUNE_PHASE_IDLE:
    case TUNE_PHASE_ERROR:
    case TUNE_PHASE_WAIT_HOME:
    default:
        break;
    }
}

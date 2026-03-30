/**
 ******************************************************************************
 * @file    calibration_controller.c
 * @brief   Feed-forward fret calibration + open-string tune-offset workflow
 *
 * Calibration flow
 * ----------------
 *  1. Start in the current logical home area and lock the open string first.
 *  2. Zero the motor at that locked open-string position.
 *  3. For each fret breakpoint:
 *     a. Use the currently loaded LUT as the feed-forward estimate.
 *     b. Move to that estimate and let the motor settle.
 *     c. Re-excite the string at the tracked FFT frequency when available,
 *        otherwise at the target fret frequency.
 *     d. Run the same PI-style lock logic used by the tune workflow until the
 *        fret is confirmed. There is no retry limit or timeout; the state
 *        machine keeps reacquiring until it locks.
 *     e. Record the actual locked motor steps and locked FFT frequency.
 *  4. Write the measured fret points directly into tuning_ff_*_bp[].
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

/** Set to 0 to skip per-fret calibration (use existing LUT). */
#define CAL_ENABLE_PER_FRET          1u

/** Strings with commissioned stepper hardware. */
#define CAL_CONFIGURED_STRING_MASK   0x0Fu

/** Direct fret points written back into tuning_ff_*_bp[]. */
#define CAL_NUM_FRETS                TUNING_FF_NUM_POINTS

/** Motor speed used for fret-to-fret calibration moves. */
#define CAL_MOTOR_SPEED             20000

/** Hops to let the motor settle after reaching the feed-forward estimate. */
#define CAL_MOVE_SETTLE_HOPS         2u

/** Timed solenoid burst used for initial excitation and re-excitation. */
#define CAL_REEXCITE_DURATION_MS     300u

/** Hops to wait for the solenoid burst to clear the FFT window. */
#define CAL_REEXCITE_WAIT_HOPS       4u

/** Silent / weak hops before calibration reacquires the signal. */
#define CAL_REEXCITE_TRIGGER_HOPS    2u

/** Minimum FFT magnitude accepted as a valid calibration peak candidate. */
#define CAL_TRACK_MIN_MAG           60.0f

/** If the tracked peak decays below this, calibration will re-excite. */
#define CAL_REEXCITE_MIN_MAG       110.0f

/** Consecutive in-band hops required before a fret is accepted. */
#define CAL_LOCK_CONFIRM_HOPS        4u

/** Hops to wait after zeroing the open-string reference. */
#define CAL_ZERO_SYNC_HOPS           2u

/** Reminder cadence while waiting for a manual pluck. */
#define CAL_PLUCK_PROMPT_HOPS        20u

/** Calibration solenoid duty cycle. */
#define CAL_SOLENOID_DUTY            50u

/* Backup SRAM base address and magic word (STM32H7, VBAT-backed, 4 KB). */
#define CAL_BKPSRAM_BASE             ((uint32_t)0x38800000UL)
#define CAL_PERSIST_MAGIC            0xCA115702UL

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
    float    transition_steps[NUM_STRINGS][TUNING_FF_NUM_POINTS][TUNING_FF_NUM_POINTS];
    uint8_t  transition_valid[NUM_STRINGS][TUNING_FF_NUM_POINTS][TUNING_FF_NUM_POINTS];
    float    poly_a  [NUM_STRINGS];
    float    poly_b  [NUM_STRINGS];
    float    poly_c  [NUM_STRINGS];
    uint32_t checksum;
} CalPersist_t;

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
    float freq;          /**< Locked FFT frequency used for polynomial/LUT */
    float measured_freq; /**< Last locked FFT frequency (diagnostic only)  */
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
static uint8_t     s_weak_sig_cnt  = 0u;
static uint16_t    s_wait_cnt      = 0u;
static CalPhase_t  s_after_wait    = CAL_PHASE_IDLE;
static CalPhase_t  s_after_move    = CAL_PHASE_IDLE;
static uint16_t    s_manual_prompt_hops = 0u;
static CalPoint_t  s_buf[CAL_NUM_FRETS];
static uint16_t    s_buf_n         = 0u;
static float       s_transition_delta[NUM_STRINGS][CAL_NUM_FRETS][CAL_NUM_FRETS] = {{{0}}};
static uint8_t     s_transition_valid[NUM_STRINGS][CAL_NUM_FRETS][CAL_NUM_FRETS] = {{{0}}};
static uint8_t     s_transition_samples[NUM_STRINGS][CAL_NUM_FRETS][CAL_NUM_FRETS] = {{{0}}};
static uint8_t     s_open_manual_seed_pending = 0u;

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
                             float *out_freq,
                             float *out_mag)
{
    float best_freq = 0.0f;
    float best_mag  = 0.0f;
    float best_err  = 1.0e9f;

    for (uint8_t i = 0u; i < active_string_count; i++) {
        float freq = active_frequencies[i];
        float mag  = active_magnitudes[i];

        if (freq < min_freq || freq > max_freq || mag <= CAL_TRACK_MIN_MAG) {
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
    if (out_mag != NULL) {
        *out_mag = best_mag;
    }
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
    const uint8_t *bytes = (const uint8_t *)p;
    size_t end = (size_t)((const uint8_t *)&p->checksum - bytes);
    uint32_t csum = 2166136261u; /* FNV-1a seed */

    for (size_t i = sizeof(p->magic); i < end; i++) {
        csum ^= (uint32_t)bytes[i];
        csum *= 16777619u;
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

static void CalClearTransitionTable(uint8_t str)
{
    memset(s_transition_delta[str], 0, sizeof(s_transition_delta[str]));
    memset(s_transition_valid[str], 0, sizeof(s_transition_valid[str]));
    memset(s_transition_samples[str], 0, sizeof(s_transition_samples[str]));
}

static float CalEstimateFreqFromSteps(uint8_t str, float steps)
{
    const float *freq_bp  = tuning_ff_freq_bp[str];
    const float *steps_bp = tuning_ff_steps_bp[str];

    if (steps <= steps_bp[0]) {
        return freq_bp[0];
    }
    if (steps >= steps_bp[CAL_NUM_FRETS - 1u]) {
        return freq_bp[CAL_NUM_FRETS - 1u];
    }

    for (uint32_t i = 0u; i < (CAL_NUM_FRETS - 1u); i++) {
        float x0 = steps_bp[i];
        float x1 = steps_bp[i + 1u];
        float y0 = freq_bp[i];
        float y1 = freq_bp[i + 1u];

        if (steps <= x1) {
            float span = x1 - x0;
            if (fabsf(span) <= 1e-6f) {
                return y0;
            }

            return y0 + ((steps - x0) / span) * (y1 - y0);
        }
    }

    return freq_bp[CAL_NUM_FRETS - 1u];
}

static void CalStoreTransitionDelta(uint8_t str,
                                    uint8_t from_fret,
                                    uint8_t to_fret,
                                    float delta_steps)
{
    uint8_t n = s_transition_samples[str][from_fret][to_fret];
    float prev = s_transition_delta[str][from_fret][to_fret];

    s_transition_delta[str][from_fret][to_fret] =
        (n == 0u) ? delta_steps : ((prev * (float)n) + delta_steps) / (float)(n + 1u);

    s_transition_samples[str][from_fret][to_fret] = (uint8_t)(n + 1u);
    s_transition_valid[str][from_fret][to_fret]   = 1u;
}

static void CalResetLockState(float seed_freq)
{
    s_pi_integral    = 0.0f;
    s_pi_cmd_steps   = g_shared_memory.current_positions[s_str];
    s_lock_consec    = 0u;
    s_no_sig_cnt     = 0u;
    s_weak_sig_cnt   = 0u;
    s_last_valid_freq = seed_freq;
}

static void CalGotoThen(int32_t position, CalPhase_t next)
{
    InterCore_SendMotorGoto(s_str, position);
    s_after_move = next;
    s_phase      = CAL_PHASE_WAIT_MOVE;
}

static uint8_t CalPrimeNextFretFromLockedNote(uint8_t from_fret, float locked_freq)
{
    uint8_t next_fret = (uint8_t)(from_fret + 1u);

    if (next_fret >= CAL_NUM_FRETS || !CalStringHasSolenoid(s_str)) {
        return 0u;
    }

    if (locked_freq < 40.0f || locked_freq > 800.0f) {
        return 0u;
    }

    s_fret           = next_fret;
    s_target_freq    = CalTargetFreq(s_str, next_fret);
    s_steps_expected = CalEstimateSteps(s_str, next_fret);

    printf("[CAL] str%u fret%u->%u: prime %.2f Hz, FF seed %ld steps -> %.2f Hz\r\n",
           (unsigned)s_str,
           (unsigned)from_fret,
           (unsigned)next_fret,
           (double)locked_freq,
           (long)s_steps_expected,
           (double)s_target_freq);

    StopSolenoid(s_str);
    InterCore_TriggerSolenoidHz(s_str,
                                locked_freq,
                                CAL_SOLENOID_DUTY,
                                SOLENOID_MODE_TIMED,
                                CAL_REEXCITE_DURATION_MS,
                                0u);

    CalGotoThen(s_steps_expected, CAL_PHASE_LOCK);
    return 1u;
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
    if (s_open_manual_seed_pending) {
        s_open_manual_seed_pending = 0u;
        CalResetLockState(0.0f);

        printf("[CAL] str%u fret%u: pluck %s now (open reference %.2f Hz)\r\n",
               (unsigned)s_str,
               (unsigned)s_fret,
               s_string_names[TuningCtrl_GetActiveTuning()][s_str],
               (double)s_target_freq);

        s_manual_prompt_hops = 0u;
        s_phase = CAL_PHASE_WAIT_PLUCK;
        return;
    }

    float position_freq = CalEstimateFreqFromSteps(s_str,
                                                   (float)g_shared_memory.current_positions[s_str]);
    float excite_freq = (s_last_valid_freq > 0.0f) ? s_last_valid_freq : position_freq;

    if (excite_freq < 40.0f || excite_freq > 800.0f) {
        excite_freq = s_target_freq;
    }

    CalResetLockState(excite_freq);

    if (CalStringHasSolenoid(s_str) && excite_freq >= 40.0f && excite_freq <= 800.0f) {
        printf("[CAL] str%u fret%u: excite %.2f Hz near %ld steps (target %.2f)\r\n",
               (unsigned)s_str,
               (unsigned)s_fret,
               (double)excite_freq,
               (long)g_shared_memory.current_positions[s_str],
               (double)s_target_freq);

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

static void CalRequestReacquire(uint8_t weak_signal, float signal_value)
{
    int32_t hold_steps = g_shared_memory.current_positions[s_str];

    s_pi_cmd_steps = hold_steps;

    if (g_shared_memory.motor_is_moving[s_str]) {
        InterCore_SendMotorGoto(s_str, hold_steps);
        if (weak_signal) {
            printf("[CAL] str%u fret%u: weak signal %.1f -> stop + reacquire\r\n",
                   (unsigned)s_str,
                   (unsigned)s_fret,
                   (double)signal_value);
        } else {
            printf("[CAL] str%u fret%u: signal lost -> stop + reacquire\r\n",
                   (unsigned)s_str,
                   (unsigned)s_fret);
        }
        WaitThen(CAL_MOVE_SETTLE_HOPS, CAL_PHASE_EXCITE);
    } else {
        if (weak_signal) {
            printf("[CAL] str%u fret%u: weak signal %.1f -> reacquiring\r\n",
                   (unsigned)s_str,
                   (unsigned)s_fret,
                   (double)signal_value);
        } else {
            printf("[CAL] str%u fret%u: signal lost -> reacquiring\r\n",
                   (unsigned)s_str,
                   (unsigned)s_fret);
        }
        s_phase = CAL_PHASE_EXCITE;
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

/* =========================================================================
 * Persistent storage
 * ========================================================================= */

static void Cal_SaveToSRAM(void)
{
    Cal_EnableBkpSRAM();

    CalPersist_t *p = (CalPersist_t *)CAL_BKPSRAM_BASE;

    memset(p, 0, sizeof(*p));
    p->magic = CAL_PERSIST_MAGIC;
    memcpy(p->ff_freq,  tuning_ff_freq_bp,  sizeof(tuning_ff_freq_bp));
    memcpy(p->ff_steps, tuning_ff_steps_bp, sizeof(tuning_ff_steps_bp));
    memcpy(p->transition_steps, s_transition_delta, sizeof(s_transition_delta));
    memcpy(p->transition_valid, s_transition_valid, sizeof(s_transition_valid));

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        p->poly_a[i] = cal_poly_coeff[i][0];
        p->poly_b[i] = cal_poly_coeff[i][1];
        p->poly_c[i] = cal_poly_coeff[i][2];
    }

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
    memcpy(s_transition_delta, p->transition_steps, sizeof(s_transition_delta));
    memcpy(s_transition_valid, p->transition_valid, sizeof(s_transition_valid));
    memset(s_transition_samples, 0, sizeof(s_transition_samples));

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        cal_poly_coeff[i][0] = p->poly_a[i];
        cal_poly_coeff[i][1] = p->poly_b[i];
        cal_poly_coeff[i][2] = p->poly_c[i];
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

#if CAL_ENABLE_PER_FRET
    s_status = CAL_STATUS_RUNNING;
    s_phase  = CAL_PHASE_STR_INIT;

    printf("[CAL] ================================================\r\n");
    printf("[CAL] Feed-forward calibration started — mask 0x%02X\r\n",
           (unsigned)s_string_mask);
    printf("[CAL] Seed source: active tuning_ff tables (SRAM if loaded, otherwise hardcoded)\r\n");
    printf("[CAL] Per-string flow: open lock -> zero -> LUT seed -> excite/pluck -> tune-style lock -> record\r\n");
    printf("[CAL] Output: str, fret, steps, measured_hz, target_hz\r\n");
    printf("[CAL] ================================================\r\n");
#else
    printf("[CAL] Per-fret calibration disabled (CAL_ENABLE_PER_FRET=0).\r\n");
    printf("[CAL] Use 'transcal' for iterative transition calibration.\r\n");
    s_status = CAL_STATUS_DONE;
    s_phase  = CAL_PHASE_DONE;
#endif
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

uint8_t Cal_GetTransitionDelta(uint8_t str_id,
                               uint8_t from_fret,
                               uint8_t to_fret,
                               float  *out_delta_steps)
{
    if (str_id >= NUM_STRINGS || from_fret >= CAL_NUM_FRETS || to_fret >= CAL_NUM_FRETS) {
        return 0u;
    }

    if (!s_transition_valid[str_id][from_fret][to_fret]) {
        return 0u;
    }

    if (out_delta_steps != NULL) {
        *out_delta_steps = s_transition_delta[str_id][from_fret][to_fret];
    }

    return 1u;
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
        s_buf_n                = 0u;

        s_fret           = 0u;
        s_target_freq    = open_f;
        s_steps_expected = 0;
        s_open_manual_seed_pending = 1u;
        s_manual_prompt_hops = 0u;
        CalResetLockState(0.0f);

        printf("\r\n[CAL] ---- String %u (%s) ----\r\n",
               (unsigned)s_str,
               s_string_names[TuningCtrl_GetActiveTuning()][s_str]);
        printf("[CAL] Open-string reference: manual pluck, lock %.2f Hz, then zero motor there\r\n",
               (double)open_f);
        printf("[CAL] Re-excitation: %s\r\n",
               CalStringHasSolenoid(s_str) ? "solenoid" : "manual pluck");

        CalGotoThen(0, CAL_PHASE_EXCITE);
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
            float seed_freq = CalEstimateFreqFromSteps(s_str,
                                                       (float)g_shared_memory.current_positions[s_str]);
            CalResetLockState(seed_freq);
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
        float meas_mag    = 0.0f;

        TrackClosestFreq(center_freq, min_freq, max_freq, &meas_f, &meas_mag);

        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;
            s_no_sig_cnt      = 0u;
            s_weak_sig_cnt    = 0u;
            s_lock_consec     = 0u;
            printf("[CAL] str%u fret%u: detected %.2f Hz (mag=%.1f) — locking\r\n",
                   (unsigned)s_str,
                   (unsigned)s_fret,
                   (double)meas_f,
                   (double)meas_mag);
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
        float meas_mag    = 0.0f;
        uint8_t motor_busy = g_shared_memory.motor_is_moving[s_str];

        TrackClosestFreq(center_freq, min_freq, max_freq, &meas_f, &meas_mag);

        if (meas_f > 0.0f) {
            s_last_valid_freq = meas_f;
            if (meas_mag < CAL_REEXCITE_MIN_MAG) {
                s_lock_consec = 0u;
                s_no_sig_cnt  = 0u;
                if (s_weak_sig_cnt < 0xFFu) {
                    s_weak_sig_cnt++;
                }
                if (s_weak_sig_cnt >= CAL_REEXCITE_TRIGGER_HOPS) {
                    CalRequestReacquire(1u, meas_mag);
                }
            } else {
                float error = s_target_freq - meas_f;

                s_no_sig_cnt   = 0u;
                s_weak_sig_cnt = 0u;

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
                        float cmd_lo = (s_fret == 0u)
                                     ? (float)(-s_cal_steps_max[s_str])
                                     : 0.0f;
                        int32_t cmd = (int32_t)CalClampf(
                            (float)s_pi_cmd_steps + delta_cmd,
                            cmd_lo,
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
                    if (s_fret == 0u) {
                        printf("[CAL] str%u open reference locked — zeroing at %.2f Hz\r\n",
                               (unsigned)s_str,
                               (double)meas_f);
                        InterCore_SendMotorZero(s_str);
                        WaitThen(CAL_ZERO_SYNC_HOPS, CAL_PHASE_RECORD);
                    } else {
                        s_phase = CAL_PHASE_RECORD;
                    }
                }
            }
        } else {
            s_no_sig_cnt++;
            s_weak_sig_cnt = 0u;

            if (s_no_sig_cnt >= CAL_REEXCITE_TRIGGER_HOPS) {
                CalRequestReacquire(0u, 0.0f);
            }
        }
        break;
    }

    case CAL_PHASE_RECORD:
    {
        uint8_t recorded_fret = s_fret;
        int32_t actual_steps = (s_fret == 0u) ? 0 : g_shared_memory.current_positions[s_str];
        float measured_freq  = (s_last_valid_freq > 0.0f) ? s_last_valid_freq : s_target_freq;

        s_buf[s_fret].freq          = measured_freq;
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

        if (CalPrimeNextFretFromLockedNote(recorded_fret, measured_freq)) {
            break;
        }

        s_fret = (uint8_t)(recorded_fret + 1u);
        s_phase = (s_fret >= CAL_NUM_FRETS) ? CAL_PHASE_BUILD : CAL_PHASE_NEXT_FRET;
        break;
    }

    case CAL_PHASE_BUILD:
    {
        Cal_BuildTable(s_str);
        Cal_FitPolynomial(s_str);
        StopSolenoid(s_str);
        Cal_SaveToSRAM();
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

        TrackClosestFreq(open_f, open_f * 0.70f, open_f * 1.80f, &best_freq, NULL);

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

        TrackClosestFreq(center_freq, min_freq, max_freq, &meas_f, NULL);

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

            printf("[TUNE] ================================================\r\n");
            printf("[TUNE] Tuning complete. Offsets:");
            for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
                if ((s_tune_mask >> i) & 1u) {
                    printf(" str%u=%ld", (unsigned)i, (long)s_tune_offset[i]);
                }
            }
            printf("\r\n");
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

        printf("[TUNE] ================================================\r\n");
        printf("[TUNE] Tuning complete (via DONE phase). Offsets:");
        for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
            if ((s_tune_mask >> i) & 1u) {
                printf(" str%u=%ld", (unsigned)i, (long)s_tune_offset[i]);
            }
        }
        printf("\r\n");
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

/* =========================================================================
 * Iterative transition calibration
 *
 * Runs fret-to-fret jumps from the Smoke on Water song repeatedly until
 * the measured delta converges (last 2 agree within ±TRANSCAL_CONVERGE_STEPS).
 * After convergence a confirmation pass verifies the delta from a clean
 * from-fret lock.
 * ========================================================================= */

/* Transition calibration parameters */
#define TRANSCAL_MAX_ITERS       10u    /* Maximum passes per transition       */
#define TRANSCAL_CONVERGE_STEPS  40.0f  /* Stop if last 2 agree within this   */
#define TRANSCAL_MOTOR_SPEED     20000  /* Same speed as song playback        */
#define TRANSCAL_SETTLE_HOPS     3u     /* Hops to wait after motor move      */
#define TRANSCAL_LOCK_TIMEOUT    30u    /* Max hops for PI to lock            */

/* Magnitude-based hit confirmation (replaces PI lock at destination) */
#define TRANSCAL_HIT_MIN_MAG     1000000.0f /* Min FFT magnitude for "hit"   */
#define TRANSCAL_HIT_FREQ_TOL    5.0f   /* Hz tolerance for frequency match   */
#define TRANSCAL_HIT_CHECK_HOPS  5u     /* Hops to check after burst clears   */

/* Smoke on Water unique transitions (both directions) */
typedef struct {
    uint8_t from_fret;
    uint8_t to_fret;
} TransitionPair_t;

static const TransitionPair_t s_smoke_transitions[] = {
    { 0, 3 },   /* open -> fret 3 */
    { 3, 5 },   /* fret 3 -> fret 5 */
    { 3, 6 },   /* fret 3 -> fret 6 */
    { 6, 5 },   /* fret 6 -> fret 5 */
    { 5, 0 },   /* fret 5 -> open */
    { 5, 3 },   /* fret 5 -> fret 3 */
    { 3, 0 },   /* fret 3 -> open */
};
#define SMOKE_TRANSITION_COUNT \
    ((uint8_t)(sizeof(s_smoke_transitions) / sizeof(s_smoke_transitions[0])))

/* Transition calibration state machine phases */
typedef enum {
    TC_PHASE_IDLE = 0,
    /* Iteration phases */
    TC_PHASE_INIT,              /* Setup for a transition pair          */
    TC_PHASE_GOTO_FROM,         /* Move to from_fret position           */
    TC_PHASE_WAIT_MOVE,         /* Wait for motor to arrive             */
    TC_PHASE_EXCITE_FROM,       /* Re-excite string at from_fret        */
    TC_PHASE_BURST_REENABLE,    /* Re-enable FFT after burst            */
    TC_PHASE_LOCK_FROM,         /* PI lock at from_fret, record start   */
    TC_PHASE_JUMP,              /* Relative delta move to to_fret       */
    TC_PHASE_WAIT_JUMP,         /* Wait for jump motor move             */
    TC_PHASE_EXCITE_TO,         /* Re-excite string at to_fret          */
    TC_PHASE_BURST_REENABLE_TO, /* Re-enable FFT after to_fret burst    */
    TC_PHASE_LOCK_TO,           /* PI lock at to_fret, measure delta    */
    TC_PHASE_RECORD,            /* Record delta, check convergence      */
    /* Confirmation phases */
    TC_PHASE_CONFIRM_GOTO,      /* Go back to true from_fret            */
    TC_PHASE_CONFIRM_WAIT_MOVE, /* Wait for confirm motor move          */
    TC_PHASE_CONFIRM_EXCITE,    /* Excite at from_fret                  */
    TC_PHASE_CONFIRM_BURST_RE,  /* Re-enable FFT after confirm burst    */
    TC_PHASE_CONFIRM_LOCK_FROM, /* PI lock at from_fret                 */
    TC_PHASE_CONFIRM_JUMP,      /* Execute converged delta              */
    TC_PHASE_CONFIRM_WAIT_JUMP, /* Wait for confirm jump                */
    TC_PHASE_CONFIRM_EXCITE_TO, /* Excite at to_fret                    */
    TC_PHASE_CONFIRM_BURST_RE2, /* Re-enable FFT                        */
    TC_PHASE_CONFIRM_LOCK_TO,   /* PI lock at to_fret, verify           */
    /* Routing */
    TC_PHASE_NEXT,              /* Advance to next iter/transition      */
    TC_PHASE_DONE,
    TC_PHASE_ERROR
} TcPhase_t;

/* Transition calibration state */
static TcPhase_t   s_tc_phase          = TC_PHASE_IDLE;
static CalStatus_t s_tc_status         = CAL_STATUS_IDLE;
static uint8_t     s_tc_string_mask    = 0u;
static uint8_t     s_tc_str            = 0u;
static uint8_t     s_tc_transition_idx = 0u;
static uint8_t     s_tc_iter           = 0u;
static int32_t     s_tc_start_pos      = 0;
static float       s_tc_prev_delta     = 0.0f;
static uint8_t     s_tc_converged      = 0u;
static float       s_tc_converged_avg  = 0.0f;
static int32_t     s_tc_confirm_start  = 0;
static uint8_t     s_tc_hit_check_cnt  = 0u;

/* PI lock state for transition cal (reuses same approach as Cal_Hop LOCK) */
static float       s_tc_pi_integral    = 0.0f;
static int32_t     s_tc_pi_cmd_steps   = 0;
static uint8_t     s_tc_lock_consec    = 0u;
static uint8_t     s_tc_no_sig_cnt     = 0u;
static uint8_t     s_tc_weak_sig_cnt   = 0u;
static float       s_tc_last_valid_f   = 0.0f;
static float       s_tc_target_freq    = 0.0f;
static uint16_t    s_tc_wait_cnt       = 0u;
static TcPhase_t   s_tc_after_wait     = TC_PHASE_IDLE;
static uint8_t     s_tc_lock_hop_cnt   = 0u;

static void TcWaitThen(uint16_t hops, TcPhase_t next)
{
    s_tc_wait_cnt   = hops;
    s_tc_after_wait = next;
    s_tc_phase      = TC_PHASE_IDLE;  /* reuse idle as wait */
}

static void TcResetLockState(float seed_freq)
{
    s_tc_pi_integral  = 0.0f;
    s_tc_pi_cmd_steps = g_shared_memory.current_positions[s_tc_str];
    s_tc_lock_consec  = 0u;
    s_tc_no_sig_cnt   = 0u;
    s_tc_weak_sig_cnt = 0u;
    s_tc_last_valid_f = seed_freq;
    s_tc_lock_hop_cnt = 0u;
}

/**
 * @brief Run one PI lock hop for the transition calibration.
 *
 * Mirrors the CAL_PHASE_LOCK logic but returns 1 when locked (lock_consec
 * reaches CAL_LOCK_CONFIRM_HOPS), 0 otherwise.  On signal loss it
 * re-excites via solenoid burst.
 */
static uint8_t TcRunLockHop(TcPhase_t excite_phase)
{
    float center_freq = (s_tc_last_valid_f > 0.0f) ? s_tc_last_valid_f : s_tc_target_freq;
    float min_freq    = s_tc_target_freq * TUNE_TRACK_MIN_RATIO;
    float max_freq    = s_tc_target_freq * TUNE_TRACK_MAX_RATIO;
    float meas_f      = 0.0f;
    float meas_mag    = 0.0f;
    uint8_t motor_busy = g_shared_memory.motor_is_moving[s_tc_str];

    TrackClosestFreq(center_freq, min_freq, max_freq, &meas_f, &meas_mag);

    if (meas_f > 0.0f) {
        s_tc_last_valid_f = meas_f;

        if (meas_mag < CAL_REEXCITE_MIN_MAG) {
            s_tc_lock_consec = 0u;
            s_tc_no_sig_cnt  = 0u;
            if (s_tc_weak_sig_cnt < 0xFFu) s_tc_weak_sig_cnt++;
            if (s_tc_weak_sig_cnt >= CAL_REEXCITE_TRIGGER_HOPS) {
                s_tc_phase = excite_phase;
            }
            return 0u;
        }

        float error = s_tc_target_freq - meas_f;
        s_tc_no_sig_cnt   = 0u;
        s_tc_weak_sig_cnt = 0u;

        if (fabsf(error) > TUNE_LOCK_BAND_HZ) {
            if (fabsf(error) <= TUNE_INTEGRAL_ENABLE_BAND_HZ) {
                s_tc_pi_integral = CalClampf(
                    s_tc_pi_integral + TUNE_PI_KI * error * TUNING_SAMPLE_TIME_S,
                    -TUNE_PI_MAX_TRIM, TUNE_PI_MAX_TRIM);
            } else {
                s_tc_pi_integral = 0.0f;
            }

            if (!motor_busy) {
                float step_cap = StepCapForError(fabsf(error));
                float delta_cmd = CalClampf(TUNE_PI_KP * error + s_tc_pi_integral,
                                            -step_cap, step_cap);
                int32_t cmd = (int32_t)CalClampf(
                    (float)s_tc_pi_cmd_steps + delta_cmd,
                    0.0f, (float)s_cal_steps_max[s_tc_str]);
                int32_t delta = cmd - s_tc_pi_cmd_steps;
                if ((delta < 0 ? -delta : delta) >= (int32_t)TUNE_MIN_MOVE_STEPS) {
                    s_tc_pi_cmd_steps = cmd;
                    InterCore_SendMotorGoto(s_tc_str, s_tc_pi_cmd_steps);
                }
            }

            s_tc_lock_consec = 0u;
        } else if (!motor_busy) {
            s_tc_lock_consec++;
        }

        if (s_tc_lock_consec >= CAL_LOCK_CONFIRM_HOPS) {
            return 1u;  /* Locked! */
        }
    } else {
        s_tc_no_sig_cnt++;
        s_tc_weak_sig_cnt = 0u;

        if (s_tc_no_sig_cnt >= CAL_REEXCITE_TRIGGER_HOPS) {
            s_tc_phase = excite_phase;
        }
    }

    /* Timeout check */
    if (!motor_busy) s_tc_lock_hop_cnt++;
    if (s_tc_lock_hop_cnt >= TRANSCAL_LOCK_TIMEOUT) {
        printf("[TRANSCAL] str%u: lock timeout at %.2f Hz (target %.2f Hz)\r\n",
               (unsigned)s_tc_str,
               (double)s_tc_last_valid_f,
               (double)s_tc_target_freq);
        s_tc_phase = excite_phase;  /* Re-excite and retry */
        s_tc_lock_hop_cnt = 0u;
    }

    return 0u;
}

static void TcExciteBurst(TcPhase_t reenable_phase)
{
    float burst_freq = (s_tc_last_valid_f > 0.0f) ? s_tc_last_valid_f : s_tc_target_freq;

    if (CalStringHasSolenoid(s_tc_str) && burst_freq >= 40.0f && burst_freq <= 800.0f) {
        StopSolenoid(s_tc_str);
        fft_sensing_enabled = 0u;
        InterCore_TriggerSolenoidHz(s_tc_str, burst_freq,
                                    CAL_SOLENOID_DUTY, SOLENOID_MODE_TIMED,
                                    CAL_REEXCITE_DURATION_MS, 0u);
        s_tc_wait_cnt   = CAL_REEXCITE_WAIT_HOPS;
        s_tc_after_wait = reenable_phase;
        s_tc_phase      = TC_PHASE_IDLE;
    } else {
        /* No solenoid — just continue (manual pluck needed) */
        printf("[TRANSCAL] str%u: pluck string now (target %.2f Hz)\r\n",
               (unsigned)s_tc_str, (double)s_tc_target_freq);
        s_tc_phase = reenable_phase;
    }
}

static uint8_t TcAdvanceToNextString(void)
{
    do {
        s_tc_str++;
    } while (s_tc_str < NUM_STRINGS && !((s_tc_string_mask >> s_tc_str) & 1u));
    return (s_tc_str < NUM_STRINGS) ? 1u : 0u;
}

/* ---- Public API ---- */

void Cal_StartTransitionCal(uint8_t string_mask)
{
    uint8_t requested = string_mask & 0x3Fu;
    uint8_t supported = requested & CAL_CONFIGURED_STRING_MASK;

    if (supported == 0u) {
        printf("[TRANSCAL] ERROR: no configured motors in mask 0x%02X\r\n",
               (unsigned)requested);
        s_tc_status = CAL_STATUS_ERROR;
        return;
    }

    s_tc_string_mask = supported;
    s_tc_str = 0u;
    while (s_tc_str < NUM_STRINGS && !((s_tc_string_mask >> s_tc_str) & 1u)) {
        s_tc_str++;
    }

    /* Clear existing transition data for selected strings */
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        if ((s_tc_string_mask >> i) & 1u) {
            CalClearTransitionTable(i);
        }
    }

    s_tc_transition_idx = 0u;
    s_tc_iter           = 0u;
    s_tc_converged      = 0u;
    fft_sensing_enabled = 1u;
    s_tc_status         = CAL_STATUS_RUNNING;
    s_tc_phase          = TC_PHASE_INIT;

    printf("[TRANSCAL] ================================================\r\n");
    printf("[TRANSCAL] Transition calibration started — mask 0x%02X\r\n",
           (unsigned)s_tc_string_mask);
    printf("[TRANSCAL] %u transitions, max %u iterations each\r\n",
           (unsigned)SMOKE_TRANSITION_COUNT,
           (unsigned)TRANSCAL_MAX_ITERS);
    printf("[TRANSCAL] Convergence threshold: +/-%.0f steps\r\n",
           (double)TRANSCAL_CONVERGE_STEPS);
    printf("[TRANSCAL] ================================================\r\n");
}

CalStatus_t Cal_TransitionCalGetStatus(void)
{
    return s_tc_status;
}

void Cal_TransitionCalHop(void)
{
    /* Handle wait states first */
    if (s_tc_phase == TC_PHASE_IDLE && s_tc_wait_cnt > 0u) {
        s_tc_wait_cnt--;
        if (s_tc_wait_cnt == 0u) {
            s_tc_phase = s_tc_after_wait;
        }
        return;
    }

    switch (s_tc_phase) {

    /* ---- INIT: setup for a transition pair ---- */
    case TC_PHASE_INIT:
    {
        if (s_tc_transition_idx >= SMOKE_TRANSITION_COUNT) {
            /* All transitions done for this string */
            Cal_SaveToSRAM();
            StopSolenoid(s_tc_str);

            printf("[TRANSCAL] str%u: all transitions complete\r\n",
                   (unsigned)s_tc_str);

            if (TcAdvanceToNextString()) {
                s_tc_transition_idx = 0u;
                s_tc_phase = TC_PHASE_INIT;
            } else {
                printf("[TRANSCAL] ================================================\r\n");
                printf("[TRANSCAL] Transition calibration complete!\r\n");
                printf("[TRANSCAL] ================================================\r\n");
                s_tc_status = CAL_STATUS_DONE;
                s_tc_phase  = TC_PHASE_DONE;
            }
            break;
        }

        uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
        uint8_t to   = s_smoke_transitions[s_tc_transition_idx].to_fret;

        s_tc_iter      = 0u;
        s_tc_converged = 0u;
        s_tc_prev_delta = 0.0f;

        InterCore_SendMotorEnableMask((uint8_t)(1u << s_tc_str));
        InterCore_SendMotorMaxSpeed(s_tc_str, TRANSCAL_MOTOR_SPEED);

        printf("\r\n[TRANSCAL] str%u: calibrating fret %u -> %u (%u/%u)\r\n",
               (unsigned)s_tc_str,
               (unsigned)from,
               (unsigned)to,
               (unsigned)(s_tc_transition_idx + 1u),
               (unsigned)SMOKE_TRANSITION_COUNT);

        s_tc_phase = TC_PHASE_GOTO_FROM;
        break;
    }

    /* ---- GOTO_FROM: move to from_fret position ---- */
    case TC_PHASE_GOTO_FROM:
    {
        uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
        int32_t target_pos = (int32_t)CalClampf(
            tuning_ff_steps_bp[s_tc_str][from],
            0.0f, (float)s_cal_steps_max[s_tc_str]);

        s_tc_target_freq = CalTargetFreq(s_tc_str, from);

        printf("[TRANSCAL] str%u: moving to fret %u (pos %ld, %.2f Hz) iter %u/%u\r\n",
               (unsigned)s_tc_str,
               (unsigned)from,
               (long)target_pos,
               (double)s_tc_target_freq,
               (unsigned)(s_tc_iter + 1u),
               (unsigned)TRANSCAL_MAX_ITERS);

        InterCore_SendMotorGoto(s_tc_str, target_pos);
        s_tc_phase = TC_PHASE_WAIT_MOVE;
        break;
    }

    /* ---- WAIT_MOVE: wait for motor to arrive ---- */
    case TC_PHASE_WAIT_MOVE:
    {
        if (!g_shared_memory.motor_is_moving[s_tc_str]) {
            TcResetLockState(s_tc_target_freq);
            TcWaitThen(TRANSCAL_SETTLE_HOPS, TC_PHASE_EXCITE_FROM);
        }
        break;
    }

    /* ---- EXCITE_FROM: re-excite at from_fret ---- */
    case TC_PHASE_EXCITE_FROM:
    {
        TcResetLockState(s_tc_target_freq);
        TcExciteBurst(TC_PHASE_BURST_REENABLE);
        break;
    }

    /* ---- BURST_REENABLE: re-enable FFT after burst ---- */
    case TC_PHASE_BURST_REENABLE:
    {
        fft_sensing_enabled = 1u;
        s_tc_phase = TC_PHASE_LOCK_FROM;
        break;
    }

    /* ---- LOCK_FROM: PI lock at from_fret, record start position ---- */
    case TC_PHASE_LOCK_FROM:
    {
        if (TcRunLockHop(TC_PHASE_EXCITE_FROM)) {
            s_tc_start_pos = g_shared_memory.current_positions[s_tc_str];
            printf("[TRANSCAL] str%u: from_fret locked at pos %ld (%.2f Hz)\r\n",
                   (unsigned)s_tc_str,
                   (long)s_tc_start_pos,
                   (double)s_tc_last_valid_f);
            s_tc_phase = TC_PHASE_JUMP;
        }
        break;
    }

    /* ---- JUMP: execute relative delta move to to_fret ---- */
    case TC_PHASE_JUMP:
    {
        uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
        uint8_t to   = s_smoke_transitions[s_tc_transition_idx].to_fret;

        float delta;
        if (s_tc_iter == 0u && !s_transition_valid[s_tc_str][from][to]) {
            /* First iteration: use LUT delta */
            delta = tuning_ff_steps_bp[s_tc_str][to]
                  - tuning_ff_steps_bp[s_tc_str][from];
        } else {
            /* Subsequent: use updated calibrated delta */
            delta = s_transition_delta[s_tc_str][from][to];
        }

        s_tc_target_freq = CalTargetFreq(s_tc_str, to);

        printf("[TRANSCAL] str%u: jumping fret%u->%u delta=%.0f steps\r\n",
               (unsigned)s_tc_str,
               (unsigned)from,
               (unsigned)to,
               (double)delta);

        InterCore_SendMotorMoveRelative(s_tc_str, (int32_t)delta);
        s_tc_phase = TC_PHASE_WAIT_JUMP;
        break;
    }

    /* ---- WAIT_JUMP: wait for jump move to finish ---- */
    case TC_PHASE_WAIT_JUMP:
    {
        if (!g_shared_memory.motor_is_moving[s_tc_str]) {
            TcResetLockState(s_tc_target_freq);
            TcWaitThen(TRANSCAL_SETTLE_HOPS, TC_PHASE_EXCITE_TO);
        }
        break;
    }

    /* ---- EXCITE_TO: re-excite at to_fret ---- */
    case TC_PHASE_EXCITE_TO:
    {
        TcResetLockState(s_tc_target_freq);
        TcExciteBurst(TC_PHASE_BURST_REENABLE_TO);
        break;
    }

    /* ---- BURST_REENABLE_TO ---- */
    case TC_PHASE_BURST_REENABLE_TO:
    {
        fft_sensing_enabled = 1u;
        s_tc_phase = TC_PHASE_LOCK_TO;
        break;
    }

    /* ---- LOCK_TO: PI lock at to_fret, measure actual delta ---- */
    case TC_PHASE_LOCK_TO:
    {
        if (TcRunLockHop(TC_PHASE_EXCITE_TO)) {
            printf("[TRANSCAL] str%u: to_fret locked at pos %ld (%.2f Hz)\r\n",
                   (unsigned)s_tc_str,
                   (long)g_shared_memory.current_positions[s_tc_str],
                   (double)s_tc_last_valid_f);
            s_tc_phase = TC_PHASE_RECORD;
        }
        break;
    }

    /* ---- RECORD: record delta, check convergence ---- */
    case TC_PHASE_RECORD:
    {
        uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
        uint8_t to   = s_smoke_transitions[s_tc_transition_idx].to_fret;
        int32_t end_pos = g_shared_memory.current_positions[s_tc_str];
        float actual_delta = (float)(end_pos - s_tc_start_pos);

        /* Update running average */
        CalStoreTransitionDelta(s_tc_str, from, to, actual_delta);

        float avg = s_transition_delta[s_tc_str][from][to];
        uint8_t n = s_transition_samples[s_tc_str][from][to];

        printf("[TRANSCAL] str%u fret%u->%u iter%u: delta=%.0f avg=%.0f (n=%u)\r\n",
               (unsigned)s_tc_str,
               (unsigned)from,
               (unsigned)to,
               (unsigned)(s_tc_iter + 1u),
               (double)actual_delta,
               (double)avg,
               (unsigned)n);

        /* Check convergence: do last 2 measurements agree within threshold? */
        if (s_tc_iter > 0u &&
            fabsf(actual_delta - s_tc_prev_delta) <= TRANSCAL_CONVERGE_STEPS) {
            s_tc_converged     = 1u;
            s_tc_converged_avg = avg;
            printf("[TRANSCAL] Converged! delta=%.0f steps — running confirmation...\r\n",
                   (double)avg);
            s_tc_phase = TC_PHASE_CONFIRM_GOTO;
        } else {
            s_tc_prev_delta = actual_delta;
            s_tc_phase = TC_PHASE_NEXT;
        }
        break;
    }

    /* ---- CONFIRM_GOTO: go back to true from_fret position ---- */
    case TC_PHASE_CONFIRM_GOTO:
    {
        uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
        int32_t target_pos = (int32_t)CalClampf(
            tuning_ff_steps_bp[s_tc_str][from],
            0.0f, (float)s_cal_steps_max[s_tc_str]);

        s_tc_target_freq = CalTargetFreq(s_tc_str, from);

        printf("[TRANSCAL] Confirmation: moving to fret %u (pos %ld)\r\n",
               (unsigned)from, (long)target_pos);

        InterCore_SendMotorGoto(s_tc_str, target_pos);
        s_tc_phase = TC_PHASE_CONFIRM_WAIT_MOVE;
        break;
    }

    case TC_PHASE_CONFIRM_WAIT_MOVE:
    {
        if (!g_shared_memory.motor_is_moving[s_tc_str]) {
            TcResetLockState(s_tc_target_freq);
            TcWaitThen(TRANSCAL_SETTLE_HOPS, TC_PHASE_CONFIRM_EXCITE);
        }
        break;
    }

    case TC_PHASE_CONFIRM_EXCITE:
    {
        TcResetLockState(s_tc_target_freq);
        TcExciteBurst(TC_PHASE_CONFIRM_BURST_RE);
        break;
    }

    case TC_PHASE_CONFIRM_BURST_RE:
    {
        fft_sensing_enabled = 1u;
        s_tc_phase = TC_PHASE_CONFIRM_LOCK_FROM;
        break;
    }

    /* ---- CONFIRM_LOCK_FROM: PI lock at from_fret ---- */
    case TC_PHASE_CONFIRM_LOCK_FROM:
    {
        if (TcRunLockHop(TC_PHASE_CONFIRM_EXCITE)) {
            s_tc_confirm_start = g_shared_memory.current_positions[s_tc_str];
            printf("[TRANSCAL] Confirm: from_fret locked at pos %ld\r\n",
                   (long)s_tc_confirm_start);
            s_tc_phase = TC_PHASE_CONFIRM_JUMP;
        }
        break;
    }

    /* ---- CONFIRM_JUMP: execute converged delta one final time ---- */
    case TC_PHASE_CONFIRM_JUMP:
    {
        uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
        uint8_t to   = s_smoke_transitions[s_tc_transition_idx].to_fret;

        s_tc_target_freq = CalTargetFreq(s_tc_str, to);

        printf("[TRANSCAL] Confirm: jumping with converged delta=%.0f\r\n",
               (double)s_tc_converged_avg);

        InterCore_SendMotorMoveRelative(s_tc_str, (int32_t)s_tc_converged_avg);
        s_tc_phase = TC_PHASE_CONFIRM_WAIT_JUMP;
        (void)from;
        break;
    }

    case TC_PHASE_CONFIRM_WAIT_JUMP:
    {
        if (!g_shared_memory.motor_is_moving[s_tc_str]) {
            TcResetLockState(s_tc_target_freq);
            TcWaitThen(TRANSCAL_SETTLE_HOPS, TC_PHASE_CONFIRM_EXCITE_TO);
        }
        break;
    }

    case TC_PHASE_CONFIRM_EXCITE_TO:
    {
        TcResetLockState(s_tc_target_freq);
        TcExciteBurst(TC_PHASE_CONFIRM_BURST_RE2);
        break;
    }

    case TC_PHASE_CONFIRM_BURST_RE2:
    {
        fft_sensing_enabled = 1u;
        s_tc_hit_check_cnt  = 0u;   /* Reset hop counter for magnitude check */
        s_tc_phase = TC_PHASE_CONFIRM_LOCK_TO;
        break;
    }

    /* ---- CONFIRM_LOCK_TO: magnitude-based hit check at to_fret ---- */
    case TC_PHASE_CONFIRM_LOCK_TO:
    {
        uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
        uint8_t to   = s_smoke_transitions[s_tc_transition_idx].to_fret;

        float meas_f   = 0.0f;
        float meas_mag = 0.0f;
        TrackClosestFreq(s_tc_target_freq,
                         s_tc_target_freq * TUNE_TRACK_MIN_RATIO,
                         s_tc_target_freq * TUNE_TRACK_MAX_RATIO,
                         &meas_f, &meas_mag);

        s_tc_hit_check_cnt++;

        if (meas_f > 0.0f &&
            fabsf(meas_f - s_tc_target_freq) <= TRANSCAL_HIT_FREQ_TOL &&
            meas_mag >= TRANSCAL_HIT_MIN_MAG) {
            /* Strong peak near target frequency → confirmed hit */
            printf("[TRANSCAL] CONFIRMED fret%u->%u: mag=%.0f freq=%.2f Hz "
                   "(target %.2f Hz) delta=%.0f steps\r\n",
                   (unsigned)from, (unsigned)to,
                   (double)meas_mag, (double)meas_f,
                   (double)s_tc_target_freq,
                   (double)s_tc_converged_avg);

            /* Advance to next transition */
            s_tc_transition_idx++;
            s_tc_phase = TC_PHASE_INIT;
        } else if (s_tc_hit_check_cnt >= TRANSCAL_HIT_CHECK_HOPS) {
            /* No strong hit detected within allowed hops */
            printf("[TRANSCAL] Confirmation FAILED — no hit at fret%u "
                   "(best mag=%.0f freq=%.2f, target=%.2f Hz) "
                   "— resuming iterations\r\n",
                   (unsigned)to,
                   (double)meas_mag, (double)meas_f,
                   (double)s_tc_target_freq);
            s_tc_converged = 0u;
            s_tc_phase = TC_PHASE_NEXT;
        }
        /* else: keep waiting for more hops */
        break;
    }

    /* ---- NEXT: advance iteration or transition ---- */
    case TC_PHASE_NEXT:
    {
        s_tc_iter++;

        if (s_tc_iter >= TRANSCAL_MAX_ITERS) {
            uint8_t from = s_smoke_transitions[s_tc_transition_idx].from_fret;
            uint8_t to   = s_smoke_transitions[s_tc_transition_idx].to_fret;
            float avg = s_transition_delta[s_tc_str][from][to];

            printf("[TRANSCAL] WARNING: max iterations reached for fret%u->%u "
                   "(best avg=%.0f) — using anyway\r\n",
                   (unsigned)from, (unsigned)to, (double)avg);

            s_tc_transition_idx++;
            s_tc_phase = TC_PHASE_INIT;
        } else {
            /* Continue iterating */
            s_tc_phase = TC_PHASE_GOTO_FROM;
        }
        break;
    }

    case TC_PHASE_DONE:
    case TC_PHASE_ERROR:
    case TC_PHASE_IDLE:
    default:
        break;
    }
}

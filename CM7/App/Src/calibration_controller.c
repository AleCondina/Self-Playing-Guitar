/**
 ******************************************************************************
 * @file    calibration_controller.c
 * @brief   Guitar string feed-forward calibration sweep
 *
 * How it works (per string)
 * -------------------------
 *  1. Send motor to position 0 (home / open string).
 *  2. Start solenoid in INFINITE mode at the open-string frequency so the
 *     string rings continuously during the sweep.
 *  3. Wait CAL_HOME_HOPS audio hops for the motor to settle and the FFT
 *     estimator to lock on.
 *  4. Sample active_frequencies[0] (FFT-detected peak, parabolic interpolated).
 *  5. Record the (step-position, frequency) pair in a local buffer.
 *  6. Advance the motor by CAL_STEP_INCREMENT microsteps; wait CAL_SETTLE_HOPS.
 *  7. Repeat steps 4–6 until the measured frequency reaches the fret-7 target,
 *     the step safety limit is hit, or the buffer is full.
 *  8. Pick TUNING_FF_NUM_POINTS evenly-spaced frequency breakpoints from the
 *     buffer and write them into tuning_ff_freq_bp / tuning_ff_steps_bp.
 *  9. Stop solenoid, return motor to 0, advance to next string.
 *
 * Why FFT instead of Goertzel?
 * ----------------------------
 * The Goertzel phase-tracker can only estimate accurately within ±~3 Hz of
 * its filter centre (limited by the hop-rate Nyquist).  A fret-7 sweep spans
 * 40–165 Hz above the open frequency – far outside that window.  The FFT's
 * parabolic-interpolated peak detection works across the full 60–440 Hz band
 * with sub-Hz accuracy and is already computed before Cal_Hop() is called.
 ******************************************************************************
 */

#include "calibration_controller.h"
#include "tuning_controller.h"      /* LUT tables (non-const) + TUNING_FF_NUM_POINTS */
#include "fft_processing.h"         /* active_frequencies[], active_string_count */
#include "inter_core_cm7.h"
#include "shared_memory.h"
#include "application_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Calibration parameters  ── edit these if hardware behaviour differs
 * ========================================================================= */

/** Microsteps advanced per measurement point.
 *  20 steps gives up to 60 samples over 1200 steps.
 *  Lower = finer LUT; raises sweep time proportionally. */
#define CAL_STEP_INCREMENT   20

/** Absolute step safety limit per string.
 *  Must be below TUNING_STEPS_MAX_DEFAULT (2000) with margin. */
#define CAL_MAX_STEPS        1200

/** Maximum (freq, step) pairs stored per string (buffer size). */
#define CAL_MAX_SAMPLES      60u

/** Hops to wait after sending motor to home (position 0) before sampling.
 *  4 hops × ~170 ms ≈ 680 ms – motor settle + solenoid warm-up. */
#define CAL_HOME_HOPS        4u

/** Hops to wait after each incremental motor move before sampling.
 *  4 hops × ~170 ms ≈ 680 ms. */
#define CAL_SETTLE_HOPS      4u

/** Frequency ratio for fret 7: 2^(7/12). */
#define CAL_FRET7_RATIO      1.4983f

/** Stop sweeping when frequency reaches this fraction above the fret-7 target.
 *  1.02 = stop 2% above fret 7 to ensure fret 7 is captured in the buffer. */
#define CAL_FRET7_MARGIN     1.02f

/** Motor max speed (steps/sec) during the calibration sweep.
 *  300 steps/sec is intentionally slow for safety. */
#define CAL_MOTOR_SPEED      300

/** Solenoid duty cycle during calibration (lighter touch than normal play). */
#define CAL_SOLENOID_DUTY    30u

/** Consecutive hops with no FFT peak before declaring error on this string. */
#define CAL_NO_SIG_LIMIT     10u

/** Minimum samples needed to fill the LUT; below this the old table is kept. */
#define CAL_MIN_SAMPLES      ((uint16_t)TUNING_FF_NUM_POINTS)

/* Open-string frequencies – must match Goertzel open_strings[].
 * String order:  0=E2  1=A2  2=D3  3=G3  4=B3  5=E4 */
static const float s_open_freq[NUM_STRINGS] = {
     82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f
};

/* =========================================================================
 * Private types
 * ========================================================================= */

typedef enum {
    CAL_PHASE_IDLE     = 0,
    CAL_PHASE_STR_INIT,   /**< Send motor home, start solenoid, reset buf  */
    CAL_PHASE_WAIT,        /**< Generic hop-count wait (motor settle / home) */
    CAL_PHASE_MEASURE,     /**< Read FFT peak, record point                 */
    CAL_PHASE_ADVANCE,     /**< Move motor +increment, arm settle counter   */
    CAL_PHASE_BUILD,       /**< Compute LUT breakpoints, teardown, next str */
    CAL_PHASE_DONE,
    CAL_PHASE_ERROR
} CalPhase_t;

typedef struct {
    float steps;
    float freq;
} CalPoint_t;

/* =========================================================================
 * Private state
 * ========================================================================= */

static CalPhase_t  s_phase              = CAL_PHASE_IDLE;
static CalStatus_t s_status             = CAL_STATUS_IDLE;
static uint8_t     s_str               = 0u;
static int32_t     s_steps             = 0;
static uint16_t    s_wait_cnt          = 0u;
static CalPhase_t  s_after_wait        = CAL_PHASE_IDLE;
static uint8_t     s_no_sig_cnt        = 0u;
static CalPoint_t  s_buf[CAL_MAX_SAMPLES];
static uint16_t    s_buf_n             = 0u;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static void StopSolenoid(uint8_t str_id)
{
    g_shared_memory.solenoid_id          = str_id;
    g_shared_memory.motor_command_flags |= CMD_STOP_SOLENOID;
    InterCore_NotifyCM4();
}

static void StartSolenoid(uint8_t str_id)
{
    uint16_t freq_u16 = (uint16_t)s_open_freq[str_id];
    InterCore_TriggerSolenoidHz(str_id,
                                freq_u16,
                                CAL_SOLENOID_DUTY,
                                SOLENOID_MODE_INFINITE,
                                0u, 0u);
}

/**
 * @brief Arm the generic wait state.
 * @param hops       Number of audio hops to wait.
 * @param next_phase Phase to enter after the wait expires.
 */
static void WaitThen(uint16_t hops, CalPhase_t next_phase)
{
    s_wait_cnt   = hops;
    s_after_wait = next_phase;
    s_phase      = CAL_PHASE_WAIT;
}

/**
 * @brief Fill LUT breakpoints for string `str` from the collected sample data.
 *
 * Selects TUNING_FF_NUM_POINTS evenly-spaced frequency targets between
 * the open-string frequency and the highest measured frequency (≤ fret-7),
 * then finds the nearest measured sample for each target and writes it into
 * tuning_ff_freq_bp / tuning_ff_steps_bp.
 *
 * Breakpoint 0 is always forced to (open_freq, 0 steps) regardless of the
 * buffer, as the motor home position is the authoritative reference.
 */
static void Cal_BuildTable(uint8_t str)
{
    float open_f  = s_open_freq[str];
    float fret7_f = open_f * CAL_FRET7_RATIO;

    if (s_buf_n < CAL_MIN_SAMPLES) {
        printf("[CAL] str%u WARN: %u samples – too few, LUT unchanged\r\n",
               (unsigned)str, (unsigned)s_buf_n);
        return;
    }

    /* Clamp fret7_f to the highest frequency actually measured */
    float max_meas = s_buf[0].freq;
    for (uint16_t k = 1u; k < s_buf_n; k++) {
        if (s_buf[k].freq > max_meas) max_meas = s_buf[k].freq;
    }
    if (fret7_f > max_meas) fret7_f = max_meas;

    printf("[CAL] str%u: %u samples | open=%.2f Hz | top=%.2f Hz\r\n",
           (unsigned)str, (unsigned)s_buf_n,
           (double)open_f, (double)fret7_f);

    for (uint8_t bp = 0u; bp < TUNING_FF_NUM_POINTS; bp++) {

        /* Breakpoint 0: always the mechanical home reference */
        if (bp == 0u) {
            tuning_ff_freq_bp[str][0]  = open_f;
            tuning_ff_steps_bp[str][0] = 0.0f;
            printf("[CAL-LUT] %u, 0, %.2f, 0\r\n",
                   (unsigned)str, (double)open_f);
            continue;
        }

        /* Evenly divide the open→fret7 range across the remaining breakpoints */
        float target_f = open_f + (fret7_f - open_f)
                         * ((float)bp / (float)(TUNING_FF_NUM_POINTS - 1u));

        /* Nearest-frequency lookup in the sample buffer */
        uint16_t best      = 0u;
        float    best_diff = fabsf(s_buf[0].freq - target_f);
        for (uint16_t k = 1u; k < s_buf_n; k++) {
            float d = fabsf(s_buf[k].freq - target_f);
            if (d < best_diff) { best_diff = d; best = k; }
        }

        tuning_ff_freq_bp[str][bp]  = s_buf[best].freq;
        tuning_ff_steps_bp[str][bp] = s_buf[best].steps;

        printf("[CAL-LUT] %u, %u, %.2f, %.0f\r\n",
               (unsigned)str, (unsigned)bp,
               (double)s_buf[best].freq,
               (double)s_buf[best].steps);
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void Cal_Start(void)
{
    s_str        = 0u;
    s_steps      = 0;
    s_buf_n      = 0u;
    s_no_sig_cnt = 0u;
    s_phase      = CAL_PHASE_STR_INIT;
    s_status     = CAL_STATUS_RUNNING;

    printf("[CAL] ============================================\r\n");
    printf("[CAL] Feed-forward calibration started\r\n");
    printf("[CAL] Sweeping strings E2 (0) through E4 (5)\r\n");
    printf("[CAL] Step increment: %d microsteps | Max: %d steps\r\n",
           CAL_STEP_INCREMENT, CAL_MAX_STEPS);
    printf("[CAL] ============================================\r\n");
    printf("[CAL] str, step, freq_hz\r\n");
}

void Cal_Hop(void)
{
    switch (s_phase) {

    /* ------------------------------------------------------------------ */
    case CAL_PHASE_STR_INIT:
    {
        /* Set conservative motor speed for safe sweeping */
        InterCore_SendMotorMaxSpeed(s_str, CAL_MOTOR_SPEED);

        /* Drive motor to position 0 (open string / home) */
        s_steps = 0;
        InterCore_SendMotorGoto(s_str, 0);

        /* Start solenoid at open-string frequency (INFINITE mode) */
        StartSolenoid(s_str);

        /* Reset sample buffer and no-signal counter */
        s_buf_n      = 0u;
        s_no_sig_cnt = 0u;

        printf("[CAL] -- String %u | open=%.2f Hz | fret7=%.2f Hz\r\n",
               (unsigned)s_str,
               (double)s_open_freq[s_str],
               (double)(s_open_freq[s_str] * CAL_FRET7_RATIO));

        /* Wait for motor to reach 0 and solenoid to warm up */
        WaitThen(CAL_HOME_HOPS, CAL_PHASE_MEASURE);
        break;
    }

    /* ------------------------------------------------------------------ */
    case CAL_PHASE_WAIT:
    {
        if (s_wait_cnt > 0u) {
            s_wait_cnt--;
        } else {
            s_phase = s_after_wait;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case CAL_PHASE_MEASURE:
    {
        float open_f      = s_open_freq[s_str];
        float stop_freq   = open_f * CAL_FRET7_RATIO * CAL_FRET7_MARGIN;
        float meas_f      = (active_string_count > 0u)
                            ? active_frequencies[0] : 0.0f;

        /* Reject readings below 80% of open frequency (noise / other string) */
        uint8_t valid = (active_string_count > 0u) && (meas_f >= open_f * 0.80f);

        if (!valid) {
            s_no_sig_cnt++;
            if (s_no_sig_cnt >= CAL_NO_SIG_LIMIT) {
                printf("[CAL] str%u ERROR: no signal for %u consecutive hops\r\n",
                       (unsigned)s_str, (unsigned)CAL_NO_SIG_LIMIT);
                StopSolenoid(s_str);
                InterCore_SendMotorGoto(s_str, 0);
                s_phase  = CAL_PHASE_ERROR;
                s_status = CAL_STATUS_ERROR;
            }
            /* Else: stay in MEASURE and try next hop */
            break;
        }

        s_no_sig_cnt = 0u;

        /* Store sample */
        if (s_buf_n < CAL_MAX_SAMPLES) {
            s_buf[s_buf_n].steps = (float)s_steps;
            s_buf[s_buf_n].freq  = meas_f;
            s_buf_n++;
        }

        printf("[CAL] %u, %ld, %.2f\r\n",
               (unsigned)s_str, (long)s_steps, (double)meas_f);

        /* Check stop conditions: fret-7 reached, step limit, or buffer full */
        uint8_t sweep_done = (meas_f   >= stop_freq)       ||
                             (s_steps  >= CAL_MAX_STEPS)    ||
                             (s_buf_n  >= CAL_MAX_SAMPLES);

        s_phase = sweep_done ? CAL_PHASE_BUILD : CAL_PHASE_ADVANCE;
        break;
    }

    /* ------------------------------------------------------------------ */
    case CAL_PHASE_ADVANCE:
    {
        s_steps += CAL_STEP_INCREMENT;
        InterCore_SendMotorGoto(s_str, s_steps);
        WaitThen(CAL_SETTLE_HOPS, CAL_PHASE_MEASURE);
        break;
    }

    /* ------------------------------------------------------------------ */
    case CAL_PHASE_BUILD:
    {
        Cal_BuildTable(s_str);

        /* Tear down: stop solenoid, return motor to home */
        StopSolenoid(s_str);
        InterCore_SendMotorGoto(s_str, 0);

        s_str++;
        if (s_str >= NUM_STRINGS) {
            printf("[CAL] ============================================\r\n");
            printf("[CAL] Calibration complete. Resuming normal play.\r\n");
            printf("[CAL] ============================================\r\n");
            s_phase  = CAL_PHASE_DONE;
            s_status = CAL_STATUS_DONE;
        } else {
            /* Wait for motor to reach home before starting next string */
            WaitThen(CAL_HOME_HOPS, CAL_PHASE_STR_INIT);
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case CAL_PHASE_IDLE:
    case CAL_PHASE_DONE:
    case CAL_PHASE_ERROR:
    default:
        break;
    }
}

CalStatus_t Cal_GetStatus(void)
{
    return s_status;
}

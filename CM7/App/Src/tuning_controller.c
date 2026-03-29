/**
 ******************************************************************************
 * @file    tuning_controller.c
 * @brief   Semitone-domain FF+PI guitar string tuning controller — CM7
 *
 * See tuning_controller.h for the full architecture description.
 *
 * Key interactions:
 *  fft_processing   – active_frequencies[] / active_magnitudes[]
 *  song_scheduler   – SongSched_GetStringId() / SongSched_GetTargetFreq()
 *  inter_core_cm7   – InterCore_SendMotorGotoBatch(), InterCore_TriggerSolenoidHz()
 *  shared_memory    – g_shared_memory.current_positions[] (motor feedback)
 *
 * Feedforward uses quadratic polynomials in the semitone domain:
 *   steps = a·semi² + b·semi + c
 * Gain scheduling uses the plant model:
 *   semi  = a·steps² + b·steps + c
 ******************************************************************************
 */

#include "tuning_controller.h"
#include "calibration_controller.h"
#include "song_scheduler.h"
#include "double_string.h"
#include "fft_processing.h"
#include "inter_core_cm7.h"
#include "shared_memory.h"
#include "application_config.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* FFT search window (Hz) — accept peaks within this radius of expected/target */
#define TUNE_SEARCH_WINDOW_HZ  15.0f
#define NOTE_CHANGE_SEMI       0.02f   /* ~2 cents threshold */

/* =========================================================================
 * Feed-forward LUT data (legacy — kept for SRAM calibration compat)
 * ========================================================================= */
/* Standard tuning feed-forward breakpoints (active copy — overwritten by SetTuning) */
float tuning_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* E2 frets 0-6 */ {  82.41f,  82.69f,  89.97f,  96.05f, 100.07f, 106.37f, 112.82f },
    /* A2 frets 0-6 */ { 110.00f, 115.54f, 121.57f, 127.57f, 139.23f, 145.69f, 151.22f },
    /* D3 frets 0-6 */ { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    /* G3 frets 0-6 */ { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    /* B3 frets 0-6 */ { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    /* E4 frets 0-6 */ { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* E2 */ {   0.0f, 267.0f, 1080.0f, 1623.0f, 2029.0f, 2706.0f, 3385.0f },
    /* A2 */ {   0.0f,  751.0f, 1549.0f, 2370.0f, 4103.0f, 5013.0f, 5955.0f },
    /* D3 */ {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    /* G3 */ {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    /* B3 */ {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    /* E4 */ {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

/* Backup copies of standard tuning (so we can restore after Drop D) */
static const float s_std_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {  82.41f,  82.69f,  89.97f,  96.05f, 100.07f, 106.37f, 112.82f },
    { 110.00f, 115.54f, 121.57f, 127.57f, 139.23f, 145.69f, 151.22f },
    { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

static const float s_std_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {   0.0f, 267.0f, 1080.0f, 1623.0f, 2029.0f, 2706.0f, 3385.0f },
    {   0.0f,  751.0f, 1549.0f, 2370.0f, 4103.0f, 5013.0f, 5955.0f },
    {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

/* Drop D feed-forward breakpoints (PLACEHOLDER — calibrate with 'lutbuild' in Drop D) */
static const float s_dropd_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* D2 frets 0-6 (placeholder: equal temperament) */
    { D2_HZ, Ds2_HZ, E2_HZ, F2_HZ, Fs2_HZ, G2_HZ, Gs2_HZ },
    /* E2 frets 0-6 (placeholder: equal temperament) */
    { E2_HZ, F2_HZ, Fs2_HZ, G2_HZ, Gs2_HZ, A2_HZ, As2_HZ },
    /* D3-E4: same as standard */
    { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

static const float s_dropd_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* D2: placeholder linear estimate (~10 steps/Hz) */
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    /* E2: placeholder linear estimate */
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    /* D3-E4: same as standard */
    {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

/* Drop G2 feed-forward breakpoints (PLACEHOLDER — calibrate with 'lutbuild' in Drop G2) */
static const float s_dropg2_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* E2: same as standard (measured LUT RUN 1) */
    {  82.41f,  82.69f,  89.97f,  96.05f, 100.07f, 106.37f, 112.82f },
    /* G2: placeholder equal temperament (calibrate with 'lutbuild') */
    { G2_HZ, Gs2_HZ, A2_HZ, As2_HZ, B2_HZ, C3_HZ, Cs3_HZ },
    /* D3-E4: same as standard */
    { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

static const float s_dropg2_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* E2: same as standard (measured LUT RUN 1) */
    {   0.0f, 267.0f, 1080.0f, 1623.0f, 2029.0f, 2706.0f, 3385.0f },
    /* G2: placeholder linear estimate (~10 steps/Hz) */
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    /* D3-E4: same as standard */
    {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

/* =========================================================================
 * Private state
 * ========================================================================= */

static TuningStringState_t s_state[NUM_STRINGS];

#define DEBUG_DECIM_HOPS  5u
static uint32_t s_debug_counter = 0u;

static uint8_t s_motor_enable_mask = 0u;

/* Deferred cold-start burst: give CM4 time to process motor-enable before
 * the very first solenoid fires. */
#define COLD_START_DEFER_HOPS  2u
static uint8_t s_song_first_note    = 1u;
static uint8_t s_cold_start_pending = 0u;

/* Dual-track mode: when enabled, RunHop drives both str0 and str1 using
 * DblString targets instead of the sequential song_scheduler. */
static uint8_t s_dual_track_mode = 0u;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static inline float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t StringCanTriggerSolenoid(uint8_t string_id, float target_hz)
{
    return (string_id < NUM_SOLENOIDS &&
            target_hz >= 40.0f &&
            target_hz <= 800.0f);
}

static uint8_t IsNewNoteTarget(uint8_t string_id, float target_freq_hz)
{
    float prev_target_semi = s_state[string_id].prev_target_semi;
    float new_target_semi  = freq_to_semi(string_id, target_freq_hz);
    return (fabsf(new_target_semi - prev_target_semi) > NOTE_CHANGE_SEMI);
}

static uint8_t BurstClearWaitHops(uint16_t burst_ms)
{
    float fft_window_ms = ((float)FFT_SIZE * 1000.0f)
                        / (float)AUDIO_SAMPLE_RATE_HZ;
    float hop_ms = TUNING_SAMPLE_TIME_S * 1000.0f;
    float wait_hops = ceilf((((float)burst_ms) + fft_window_ms) / hop_ms);

    wait_hops = Clampf(wait_hops, 0.0f, 255.0f);
    return (uint8_t)wait_hops;
}

static void StartNoteTransition(uint8_t string_id,
                                float   target_freq_hz,
                                uint8_t burst_fired,
                                uint16_t burst_ms)
{
    TuningStringState_t *s = &s_state[string_id];
    float target_semi = freq_to_semi(string_id, target_freq_hz);

    s->integral_state                = 0.0f;
    s->lock_state                    = 0u;
    s->prev_lock_state               = 0u;
    s->stall_counter                 = 0u;
    s->fault_flags                   = 0u;
    s->hop_count                     = 0u;
    s->prev_target_semi              = target_semi;
    s->resonance_wait_hops_remaining = 0u;
    s->fft_clear_wait_hops_remaining = 0u;

    if (burst_fired) {
        s->state = TUNE_STATE_RESONANCE_WAIT;
        s->resonance_wait_hops_remaining = TUNING_RESONANCE_WAIT_HOPS;
        s->fft_clear_wait_hops_remaining = BurstClearWaitHops(burst_ms);
    } else {
        s->state = TUNE_STATE_FF_MOVE;
    }
}

/**
 * @brief Feedforward: semitones → absolute step position.
 *        steps = a·semi² + b·semi + c
 */
static float feedforward_steps(uint8_t str, float target_semi)
{
    const float *c = tuning_ff_semi_coeff[str];
    float steps = c[0] * target_semi * target_semi
                + c[1] * target_semi
                + c[2];

    /* Apply tune offset from 'tune' command */
    steps += (float)Cal_TuneGetOffset(str);

    return steps;
}

/**
 * @brief Inverse plant model: steps → semitones.
 *        semi = a·steps² + b·steps + c
 *
 * Used to predict what frequency the string should be producing at the
 * current motor position, for the FFT search window.
 */
static float plant_steps_to_semi(uint8_t str, float steps)
{
    const float *c = tuning_plant_coeff[str];
    return c[0] * steps * steps + c[1] * steps + c[2];
}

/**
 * @brief Gain-scheduled proportional gain.
 *
 * The local plant gain at the current operating point is:
 *   K_local = d(semi)/d(steps) = 2·a·steps + b
 * The gain-scheduled Kp keeps the loop gain constant:
 *   Kp_actual = KP_nominal · K_AVG / K_local
 */
static float gain_scheduled_kp(uint8_t str, int32_t current_steps)
{
    const float *c = tuning_plant_coeff[str];
    float K_local = 2.0f * c[0] * (float)current_steps + c[1];

    /* Safety: avoid division by zero or tiny values */
    if (fabsf(K_local) < 1e-6f)
        K_local = (K_local >= 0.0f) ? 1e-6f : -1e-6f;

    return TUNING_KP * TUNING_K_AVG / K_local;
}

/* =========================================================================
 * Open-string rejection — prevent sympathetic resonance hijacking
 * ========================================================================= */

#define OPEN_STRING_REJECT_HZ  5.0f

static uint8_t IsOpenStringInterference(float freq, uint8_t string_id)
{
    for (uint8_t s = 0u; s < NUM_STRINGS; s++) {
        if (s == string_id) continue;
        if (fabsf(freq - s_open_freq_hz[s]) <= OPEN_STRING_REJECT_HZ)
            return 1u;
    }
    return 0u;
}

/**
 * @brief Sweep FFT peaks for the best match to target string.
 *
 * Rejects peaks near other open strings, then picks the loudest peak
 * within ±TUNE_SEARCH_WINDOW_HZ of either expected or target frequency.
 */
static void GetFFTRadarLock(uint8_t string_id,
                            float expected, float target,
                            float *out_measured, float *out_confidence)
{
    float best_measured = 0.0f;
    float best_mag      = 0.0f;

    for (uint8_t i = 0; i < active_string_count; i++) {
        float freq = active_frequencies[i];
        float mag  = active_magnitudes[i];

        if (IsOpenStringInterference(freq, string_id))
            continue;

        float err_exp = fabsf(freq - expected);
        float err_tgt = fabsf(freq - target);
        float min_err = (err_exp < err_tgt) ? err_exp : err_tgt;

        if (min_err > TUNE_SEARCH_WINDOW_HZ)
            continue;

        if (mag > best_mag) {
            best_mag      = mag;
            best_measured = freq;
        }
    }

    *out_measured   = best_measured;
    *out_confidence = best_mag;
}

/* =========================================================================
 * Public API — Init
 * ========================================================================= */

void TuningCtrl_Init(void)
{
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        TuningStringState_t *s = &s_state[i];
        memset(s, 0, sizeof(TuningStringState_t));

        s->state              = TUNE_STATE_IDLE;
        s->resonance_wait_hops_remaining = 0u;
        s->fft_clear_wait_hops_remaining = 0u;
        s->cmd_steps_abs      = 0;
        s->prev_cmd_steps_abs = 0;
        s->current_abs_steps  = 0;
        s->ff_steps_abs       = 0.0f;
        s->integral_state     = 0.0f;

        /* Sentinel: no note yet */
        s->prev_target_semi   = -999.0f;
        s->target_semi        = 0.0f;
        s->measured_semi      = 0.0f;
        s->last_valid_freq_hz = 0.0f;
        s->expected_freq_hz   = s_open_freq_hz[i];
    }

    s_debug_counter      = 0u;
    s_motor_enable_mask  = 0u;
    s_song_first_note    = 1u;
    s_cold_start_pending = 0u;
}

/* =========================================================================
 * TuningCtrl_Update — single-string per-hop update (all maths, no HAL)
 *
 * State machine:
 *   IDLE → RESONANCE_WAIT → FF_MOVE → FF_WAIT → PI_ACTIVE ↔ IN_TUNE
 *   IDLE → FF_MOVE → FF_WAIT → PI_ACTIVE ↔ IN_TUNE   (no solenoid fired)
 * ========================================================================= */

void TuningCtrl_Update(uint8_t string_id,
                       float   target_freq_hz,
                       float   measured_freq_hz,
                       float   confidence)
{
    if (string_id >= NUM_STRINGS) return;
    (void)confidence;

    TuningStringState_t *s = &s_state[string_id];

    /* ------------------------------------------------------------------ */
    /* 1. Convert to semitone domain                                        */
    /* ------------------------------------------------------------------ */
    float target_semi = freq_to_semi(string_id, target_freq_hz);
    s->target_semi    = target_semi;

    uint8_t measurement_trusted = (measured_freq_hz > 0.0f);
    uint8_t measurement_blocked = (s->fft_clear_wait_hops_remaining > 0u);
    uint8_t motor_busy          = g_shared_memory.motor_is_moving[string_id];
    float   meas_semi = 0.0f;

    if (measurement_trusted && !measurement_blocked) {
        meas_semi             = freq_to_semi(string_id, measured_freq_hz);
        s->measured_semi      = meas_semi;
        s->last_valid_freq_hz = measured_freq_hz;
        s->silent_hop_count   = 0u;
        s->string_warm        = 1u;
    } else if (measurement_blocked) {
        measurement_trusted = 0u;
        meas_semi = s->measured_semi;  /* freeze accepted state until FFT clears */
    } else {
        meas_semi = s->measured_semi;  /* hold last valid */
        s->silent_hop_count++;
        if (s->silent_hop_count >= (uint16_t)SONG_DECAY_COLD_HOPS) {
            s->string_warm = 0u;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 2. State machine                                                     */
    /* ------------------------------------------------------------------ */
    float pi_delta = 0.0f;  /* PI output (steps), added to FF position */
    uint8_t ff_command_issued = 0u;

    switch (s->state) {

    case TUNE_STATE_IDLE:
        /* No target — nothing to do */
        break;

    case TUNE_STATE_RESONANCE_WAIT:
        /* Hold the current motor target while the burst rings freely. */
        if (s->resonance_wait_hops_remaining > 0u) {
            break;
        }

        s->state = TUNE_STATE_FF_MOVE;
        /* Fall through once the breathing delay has expired. */

    case TUNE_STATE_FF_MOVE:
        /* Compute and issue the new ballistic target, then wait for the
         * first clean post-burst, post-motion FFT measurement. */
        s->integral_state = 0.0f;
        s->ff_steps_abs   = feedforward_steps(string_id, target_semi);
        s->state          = TUNE_STATE_FF_WAIT;
        ff_command_issued = 1u;
        break;

    case TUNE_STATE_FF_WAIT:
        /* Wait for the burst-clear window to expire and for CM4 to report
         * that the motor has stopped before enabling PI. */
        if (measurement_trusted && !motor_busy) {
            s->state = TUNE_STATE_PI_ACTIVE;
        }
        break;

    case TUNE_STATE_PI_ACTIVE: {
        if (!measurement_trusted) break;

        float e = target_semi - meas_semi;
        float abs_e = fabsf(e);

        /* Re-trigger feedforward if error is too large */
        if (abs_e > TUNING_FF_LOCK_SEMI) {
            s->integral_state                = 0.0f;
            s->resonance_wait_hops_remaining = 0u;
            s->fft_clear_wait_hops_remaining = 0u;
            s->state                         = TUNE_STATE_FF_MOVE;
            break;
        }

        /* Deadband → IN_TUNE */
        if (abs_e < TUNING_DEADBAND_SEMI) {
            s->lock_state = 1u;
            s->state      = TUNE_STATE_IN_TUNE;
            printf("[TC-PI] str%u | LOCKED err=%.4f st, pos=%ld\r\n",
                   (unsigned)string_id, (double)e,
                   (long)s->current_abs_steps);
            break;
        }

        /* Gain-scheduled PI */
        float kp_actual = gain_scheduled_kp(string_id, s->current_abs_steps);

        /* Accumulate integral */
        s->integral_state += e;

        /* Anti-windup: clamp integrator to physical step limits */
        float cmd_base = s->ff_steps_abs;
        float int_max = ((float)TUNING_STEPS_MAX_DEFAULT - cmd_base - kp_actual * e)
                        / TUNING_KI_Z;
        float int_min = ((float)TUNING_STEPS_MIN_DEFAULT - cmd_base - kp_actual * e)
                        / TUNING_KI_Z;
        if (int_max < int_min) { float t = int_max; int_max = int_min; int_min = t; }
        s->integral_state = Clampf(s->integral_state, int_min, int_max);

        pi_delta = kp_actual * e + TUNING_KI_Z * s->integral_state;

        printf("[TC-PI] str%u | err=%.3f st, Kp=%.1f, integ=%.1f, pi_delta=%.1f, ff=%.0f, cmd=%.0f\r\n",
               (unsigned)string_id, (double)e, (double)kp_actual,
               (double)s->integral_state, (double)pi_delta,
               (double)s->ff_steps_abs,
               (double)(s->ff_steps_abs + pi_delta));

        s->lock_state = 0u;
        break;
    }

    case TUNE_STATE_IN_TUNE:
        if (!measurement_trusted) break;

        /* Monitor for drift out of deadband */
        {
            float e = target_semi - meas_semi;
            if (fabsf(e) >= TUNING_DEADBAND_SEMI) {
                s->lock_state = 0u;
                s->state      = TUNE_STATE_PI_ACTIVE;
            }
        }
        break;

    default:
        s->state = TUNE_STATE_IDLE;
        break;
    }

    /* Store PI output for logging */
    s->last_pi_delta = pi_delta;

    /* ------------------------------------------------------------------ */
    /* 3. Compute raw commanded position                                     */
    /* ------------------------------------------------------------------ */
    int32_t last_cmd_steps = s->cmd_steps_abs;
    float raw_cmd = s->ff_steps_abs + pi_delta;

    if (s->state == TUNE_STATE_IDLE ||
        s->state == TUNE_STATE_RESONANCE_WAIT) {
        raw_cmd = (float)last_cmd_steps;
    }

    /* ------------------------------------------------------------------ */
    /* 4. Minimum move threshold (defeats backlash dither)                   */
    /* ------------------------------------------------------------------ */
    float delta_from_prev = raw_cmd - (float)last_cmd_steps;
    if (!ff_command_issued && fabsf(delta_from_prev) < TUNING_MIN_MOVE_STEPS) {
        raw_cmd = (float)last_cmd_steps;
    }

    /* ------------------------------------------------------------------ */
    /* 5. Slew limit                                                        */
    /* ------------------------------------------------------------------ */
    float max_delta = TUNING_MAX_STEP_RATE_SPS * TUNING_SAMPLE_TIME_S;
    float prev_cmd_f = (float)last_cmd_steps;
    float delta      = raw_cmd - prev_cmd_f;
    delta            = Clampf(delta, -max_delta, max_delta);
    float slewed_cmd = prev_cmd_f + delta;

    /* ------------------------------------------------------------------ */
    /* 6. Mechanical bounds                                                  */
    /* ------------------------------------------------------------------ */
    slewed_cmd = Clampf(slewed_cmd,
                        (float)TUNING_STEPS_MIN_DEFAULT,
                        (float)TUNING_STEPS_MAX_DEFAULT);

    /* ------------------------------------------------------------------ */
    /* 7. Store final command                                               */
    /* ------------------------------------------------------------------ */
    s->prev_cmd_steps_abs = last_cmd_steps;
    s->cmd_steps_abs      = (int32_t)roundf(slewed_cmd);

    /* Mirror actual motor position from shared memory */
    s->current_abs_steps = g_shared_memory.current_positions[string_id];

    /* Update expected frequency from commanded position (for FFT search).
     * Use plant model: semi → freq, via inverse polynomial. */
    float pred_semi = plant_steps_to_semi(string_id, (float)s->cmd_steps_abs);
    s->expected_freq_hz = semi_to_freq(string_id, pred_semi);

    /* ------------------------------------------------------------------ */
    /* 8. Stall detection                                                   */
    /* ------------------------------------------------------------------ */
    if (s->state == TUNE_STATE_PI_ACTIVE && measurement_trusted) {
        float freq_delta = fabsf(measured_freq_hz - s->last_valid_freq_hz);
        float abs_pi = fabsf(pi_delta);

        if (abs_pi > 200.0f && freq_delta < TUNING_STALL_MIN_FREQ_DELTA_HZ) {
            s->stall_counter++;
        } else {
            s->stall_counter = 0u;
        }

        if (s->stall_counter >= TUNING_STALL_TIMEOUT_HOPS) {
            s->fault_flags  |= TUNING_FAULT_STALL;
            s->cmd_steps_abs = s->current_abs_steps;
        }
    } else {
        s->stall_counter = 0u;
        s->fault_flags  &= (uint8_t)~TUNING_FAULT_STALL;
    }

    /* ------------------------------------------------------------------ */
    /* 9. Lock edge detection (for chord sync)                              */
    /* ------------------------------------------------------------------ */
    s->hop_count++;
}

/**
 * @brief Open-string frequencies for cross-string rejection.
 *
 * Any FFT peak within ±OPEN_STRING_REJECT_HZ of an open string that is NOT
 * the string we're tracking gets rejected.  This prevents sympathetic
 * resonance from B3 (246.94 Hz) hijacking the G3 fret 3-5 range, etc.
 */
static const float s_open_string_hz[NUM_TUNINGS][NUM_STRINGS] = {
    [TUNING_STANDARD] = { 82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f },
    [TUNING_DROP_D]   = { 73.42f,  82.41f, 146.83f, 196.00f, 246.94f, 329.63f },
    [TUNING_DROP_G2]  = { 82.41f,  98.00f, 146.83f, 196.00f, 246.94f, 329.63f },
};

/* Active tuning mode (set by TuningCtrl_SetTuning) */
static uint8_t s_active_tuning = TUNING_STANDARD;
#define OPEN_STRING_REJECT_HZ  5.0f   /* ±5 Hz around each open string */

/**
 * @brief Check whether a frequency is suspiciously close to an open string
 *        OTHER than the one we're tracking.
 *
 * @param freq       Candidate FFT peak frequency
 * @param string_id  The string we ARE tracking (don't reject its own open freq)
 * @return 1 if the peak should be rejected, 0 if it's safe
 */
static uint8_t IsOpenStringInterference(float freq, uint8_t string_id)
{
    for (uint8_t s = 0u; s < NUM_STRINGS; s++) {
        if (s == string_id) continue;               /* don't reject ourselves */
        if (fabsf(freq - s_open_string_hz[s_active_tuning][s]) <= OPEN_STRING_REJECT_HZ)
            return 1u;
    }
    return 0u;
}

/**
 * @brief Sweeps the FFT array to find the best peak matching the target string.
 *
 * Search strategy (evaluated in a single pass):
 *   1. Reject any peak within ±5 Hz of another string's open frequency.
 *   2. Among remaining candidates within ±TUNE_SEARCH_WINDOW_HZ of either
 *      the expected frequency OR the target frequency, pick the one with
 *      the highest magnitude.  Magnitude is the best discriminator when
 *      two candidates are close in frequency (e.g. B3 open at 247 Hz vs
 *      G3 fret 5 at 261 Hz).
 *
 * @param string_id    Which string we're tracking (for open-string rejection)
 * @param expected     Frequency predicted from commanded motor position
 * @param target       Final target frequency from song scheduler
 * @param out_measured  Detected frequency (0 if nothing found)
 * @param out_confidence  Peak magnitude
 */
static void GetFFTRadarLock(uint8_t string_id,
                            float expected, float target,
                            float* out_measured, float* out_confidence)
{
    float best_measured = 0.0f;
    float best_mag      = 0.0f;

    /* Build the search window: accept peaks near expected OR near target */
    for (uint8_t i = 0; i < active_string_count; i++) {
        float freq = active_frequencies[i];
        float mag  = active_magnitudes[i];

        /* Reject sympathetic resonance from other open strings */
        if (IsOpenStringInterference(freq, string_id))
            continue;

        /* Check if this peak is within the search window of either centre */
        float err_exp = fabsf(freq - expected);
        float err_tgt = fabsf(freq - target);
        float min_err = (err_exp < err_tgt) ? err_exp : err_tgt;

        if (min_err > TUNE_SEARCH_WINDOW_HZ)
            continue;   /* too far from both centres */

        /* Among valid candidates, pick the LOUDEST — magnitude is the
         * most reliable discriminator when multiple peaks compete. */
        if (mag > best_mag) {
            best_mag      = mag;
            best_measured = freq;
        }
    }

    *out_measured   = best_measured;
    *out_confidence = best_mag;
}

/* -------------------------------------------------------------------------
 * DualTrack_RunHop
 * Dual-string control path: drives both str0 and str1 every hop using
 * targets from DblString.  Uses exclusive peak claiming + anti-swap to
 * prevent the two FFT trackers from locking onto the same peak.
 * --------------------------------------------------------------------- */
static void DualTrack_RunHop(void)
{
    /* ---- Microstep safeguard (same as sequential path) ---- */
    for (uint8_t m = 0u; m < 4u; m++) {
        if (g_shared_memory.motor_microstep_status[m] != TUNING_REQUIRED_MICROSTEP_MODE) {
            printf("[TC-DT] *** HALT: Motor %u microstep mode is %u, expected %u ***\r\n",
                   (unsigned)m,
                   (unsigned)g_shared_memory.motor_microstep_status[m],
                   (unsigned)TUNING_REQUIRED_MICROSTEP_MODE);
            if (s_motor_enable_mask != 0u) {
                InterCore_SendMotorEnableMask(0u);
                s_motor_enable_mask = 0u;
            }
            return;
        }
    }

    /* ---- Check DblString is active ---- */
    if (!DblString_IsActive()) {
        if (s_motor_enable_mask != 0u) {
            InterCore_SendMotorEnableMask(0u);
            s_motor_enable_mask = 0u;
        }
        return;
    }

    /* ---- Enable both motors ---- */
    uint8_t needed_mask = (1u << 0) | (1u << 1);  /* str0 + str1 */
    if (needed_mask != s_motor_enable_mask) {
        InterCore_SendMotorEnableMask(needed_mask);
        s_motor_enable_mask = needed_mask;
    }

    /* ---- Get targets from DblString scheduler ---- */
    float target[2];
    target[0] = DblString_GetTargetFreq(0);
    target[1] = DblString_GetTargetFreq(1);

    /* ---- Two-pass FFT peak assignment with claiming + anti-swap ---- */
    float measured[2]   = { 0.0f, 0.0f };
    float confidence[2] = { 0.0f, 0.0f };
    int8_t claimed_idx  = -1;  /* FFT peak index claimed by str0 */

    for (uint8_t pass = 0; pass < 2u; pass++) {
        uint8_t s = pass;  /* str0 first, then str1 */
        float expected = s_state[s].expected_freq_hz;
        float tgt      = target[s];
        if (tgt < 1.0f) continue;  /* no target yet */

        /* Other string's expected frequency (for anti-swap) */
        uint8_t other = 1u - s;
        float other_expected = s_state[other].expected_freq_hz;

        float best_freq = 0.0f;
        float best_mag  = 0.0f;
        int8_t best_idx = -1;

        for (uint8_t i = 0u; i < active_string_count; i++) {
            float freq = active_frequencies[i];
            float mag  = active_magnitudes[i];

            /* Skip if claimed by previous pass */
            if (pass == 1u && (int8_t)i == claimed_idx) continue;

            /* Reject other open strings */
            if (IsOpenStringInterference(freq, s)) continue;

            /* Search window: near expected OR near target */
            float err_exp = fabsf(freq - expected);
            float err_tgt = fabsf(freq - tgt);
            float min_err = (err_exp < err_tgt) ? err_exp : err_tgt;
            if (min_err > TUNE_SEARCH_WINDOW_HZ) continue;

            /* Anti-swap: if this peak is closer to the OTHER string's
             * expected frequency than ours, it belongs to them. */
            float dist_ours  = fabsf(freq - expected);
            float dist_other = fabsf(freq - other_expected);
            if (other_expected > 1.0f && dist_other < dist_ours) continue;

            /* Pick loudest valid peak */
            if (mag > best_mag) {
                best_mag  = mag;
                best_freq = freq;
                best_idx  = (int8_t)i;
            }
        }

        measured[s]   = best_freq;
        confidence[s] = best_mag;

        /* Claim: str0's peak is reserved so str1 won't use it */
        if (pass == 0u && best_idx >= 0) {
            claimed_idx = best_idx;
        }
    }

    /* ---- Run PI controller for each string ---- */
    for (uint8_t s = 0; s < 2u; s++) {
        if (target[s] >= 1.0f) {
            TuningCtrl_Update(s, target[s], measured[s], confidence[s]);
        }
    }

    /* ---- Push motor commands for both strings ---- */
    {
        uint8_t move_mask = 0u;
        int32_t positions[NUM_MOTORS] = {0};
        for (uint8_t s = 0; s < 2u; s++) {
            if (target[s] >= 1.0f) {
                move_mask |= (1u << s);
                positions[s] = s_state[s].cmd_steps_abs;
            }
        }
        if (move_mask) {
            InterCore_SendMotorGotoBatch(move_mask, positions);
        }
    }

    /* ---- Solenoid actuation — edge-triggered from DblString ---- */
    for (uint8_t s = 0; s < 2u; s++) {
        if (DblString_ShouldFire(s)) {
            uint16_t burst_ms = DblString_GetBurstMs(s);
            float f = s_state[s].lock_state
                    ? s_state[s].last_valid_freq_hz : target[s];

            if (f >= 40.0f && f <= 800.0f) {
                InterCore_TriggerSolenoidHz(s, f, SOLENOID_DEFAULT_DUTY_CYCLE,
                                            SOLENOID_MODE_TIMED, burst_ms, 0u);
            }
            s_state[s].note_actuated = 1u;
            s_state[s].note_plucked  = 1u;
            printf("[TC-DT] Fire str%u: %.1f Hz burst=%u ms (lock=%u)\r\n",
                   (unsigned)s, (double)f, (unsigned)burst_ms,
                   (unsigned)s_state[s].lock_state);
        }
    }

    /* Track pluck age */
    for (uint8_t s = 0; s < 2u; s++) {
        s_state[s].hops_since_pluck++;
    }

    /* ---- Debug CSV (decimated) ---- */
    s_debug_counter++;
    if (s_debug_counter >= DEBUG_DECIM_HOPS) {
        s_debug_counter = 0u;
        for (uint8_t s = 0; s < 2u; s++) {
            if (target[s] < 1.0f) continue;
            const TuningStringState_t *ds = &s_state[s];
            float err_hz = target[s] - measured[s];
            printf("[TC-DT] %lu,%u,%.2f,%.2f,%.3f,%.1f,%.1f,%ld,%ld,%u,%u,%u,%.0f,%u\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned int)s,
                   (double)target[s],
                   (double)measured[s],
                   (double)err_hz,
                   (double)ds->ff_steps_abs,
                   (double)ds->trim_steps,
                   (long)ds->cmd_steps_abs,
                   (long)ds->current_abs_steps,
                   (unsigned int)ds->lock_state,
                   (unsigned int)ds->pi_enable,
                   (unsigned int)ds->delay_hold_counter,
                   (double)confidence[s],
                   (unsigned int)ds->fault_flags);
        }
    }
}

/* -------------------------------------------------------------------------
 * TuningCtrl_RunHop
 * High-level entry point.  Reads from song scheduler and FFT, calls
 * TuningCtrl_Update(), then issues inter-core commands.
 * --------------------------------------------------------------------- */
void TuningCtrl_RunHop(void)
{
    /* ---- Dual-track mode: use DblString scheduler ---- */
    if (s_dual_track_mode && DblString_IsActive()) {
        DualTrack_RunHop();
        return;
    }

    /* ------------------------------------------------------------------ */
    /* Microstep safeguard                                                  */
    /* ------------------------------------------------------------------ */
    for (uint8_t m = 0u; m < 4u; m++) {
        if (g_shared_memory.motor_microstep_status[m] != TUNING_REQUIRED_MICROSTEP_MODE) {
            printf("[TC] *** HALT: Motor %u microstep mode is %u, expected %u (1/16). "
                   "Tuning controller disabled. ***\r\n",
                   (unsigned)m,
                   (unsigned)g_shared_memory.motor_microstep_status[m],
                   (unsigned)TUNING_REQUIRED_MICROSTEP_MODE);
            if (s_motor_enable_mask != 0u) {
                InterCore_SendMotorEnableMask(0u);
                s_motor_enable_mask = 0u;
            }
            return;
        }
    }

    /* ------------------------------------------------------------------ */
    /* Song scheduler check                                                 */
    /* ------------------------------------------------------------------ */
    if (!SongSched_IsActive()) {
        if (s_motor_enable_mask != 0u) {
            InterCore_SendMotorEnableMask(0u);
            s_motor_enable_mask = 0u;
        }
        return;
    }

    uint8_t str    = SongSched_GetStringId();
    float   target = SongSched_GetTargetFreq();

    /* ------------------------------------------------------------------ */
    /* Motor power management                                               */
    /* ------------------------------------------------------------------ */
    uint8_t needed_mask = (1u << str);
    uint8_t chord = (SongSched_GetStringCount() >= 2u);
    uint8_t str2  = 0u;
    float   target2 = 0.0f;

    if (chord) {
        str2 = SongSched_GetStringId2();
        target2 = SongSched_GetTargetFreq2();
        needed_mask |= (1u << str2);
    }

    if (needed_mask != s_motor_enable_mask) {
        InterCore_SendMotorEnableMask(needed_mask);
        s_motor_enable_mask = needed_mask;
    }

    /* ------------------------------------------------------------------ */
    /* Note-change detection + note-start ownership                         */
    /* ------------------------------------------------------------------ */
    uint8_t new_note_primary   = IsNewNoteTarget(str, target);
    uint8_t new_note_secondary = 0u;
    uint8_t can_burst_primary  = 0u;
    uint8_t can_burst_secondary = 0u;
    uint8_t start_primary      = 0u;
    uint8_t start_secondary    = 0u;
    uint8_t burst_fired_primary = 0u;
    uint8_t burst_fired_secondary = 0u;
    uint16_t burst_ms_for_start = 0u;

    if (chord) {
        new_note_secondary = IsNewNoteTarget(str2, target2);
    }

    can_burst_primary = (new_note_primary && StringCanTriggerSolenoid(str, target));
    can_burst_secondary = (new_note_secondary &&
                           StringCanTriggerSolenoid(str2, target2));

    /* ------------------------------------------------------------------ */
    /* Cold-start burst — deferred for the very first note of the song      */
    /* Gives CM4 time to process motor-enable before the first burst.       */
    /* ------------------------------------------------------------------ */
    if (s_song_first_note && (new_note_primary || new_note_secondary)) {
        s_song_first_note = 0u;
        if (can_burst_primary || can_burst_secondary) {
            s_cold_start_pending = COLD_START_DEFER_HOPS;
        }
    }

    if (s_cold_start_pending > 0u) {
        s_cold_start_pending--;

        if (s_cold_start_pending == 0u) {
            const SongNote_t *cur = SongSched_GetNoteAt(SongSched_GetNoteIndex());
            uint16_t burst_ms = (cur && cur->burst_ms > 0u) ? cur->burst_ms : 175u;

            if (chord) {
                uint8_t sol_mask  = 0u;
                float   freqs[6] = {0.0f};

                if (can_burst_primary) {
                    sol_mask |= (1u << str);
                    freqs[str] = target;
                    burst_fired_primary = 1u;
                }
                if (can_burst_secondary) {
                    sol_mask |= (1u << str2);
                    freqs[str2] = target2;
                    burst_fired_secondary = 1u;
                }
                if (sol_mask) {
                    InterCore_TriggerSolenoidHzBatch(sol_mask, freqs,
                                                     SOLENOID_DEFAULT_DUTY_CYCLE,
                                                     SOLENOID_MODE_TIMED, burst_ms);
                }
            } else if (can_burst_primary) {
                InterCore_TriggerSolenoidHz(str, target,
                                            SOLENOID_DEFAULT_DUTY_CYCLE,
                                            SOLENOID_MODE_TIMED, burst_ms, 0u);
                burst_fired_primary = 1u;
            }

            burst_ms_for_start = burst_ms;
            start_primary      = new_note_primary;
            start_secondary    = new_note_secondary;

            printf("[TC] Cold-start burst: %u ms at %.1f Hz (deferred %u hops)\r\n",
                   (unsigned)burst_ms, (double)target,
                   (unsigned)COLD_START_DEFER_HOPS);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Solenoid actuation — BPM-scheduled, not convergence-gated           */
    /*                                                                      */
    /* ALL strings for a note fire at the BPM-scaled actuate delay after    */
    /* the note starts.  This keeps actuation locked to the song's BPM.    */
    /*                                                                      */
    /* If PI locks early, we wait.  If PI hasn't converged by the deadline  */
    /* we fire anyway at the best position the controller reached.          */
    /* Chord strings always fire simultaneously — same deadline.            */
    /* ------------------------------------------------------------------ */
    {
        uint32_t note_elapsed = SongSched_GetElapsedMs(HAL_GetTick());
        uint8_t  time_to_fire = (note_elapsed >= SongSched_GetActuateDelayMs());

        if (time_to_fire && !s_state[str].note_actuated) {

            /* Burst duration: per-note from song data, default 175ms */
            const SongNote_t *cur = SongSched_GetNoteAt(SongSched_GetNoteIndex());
            uint16_t burst_ms = (cur && cur->burst_ms > 0u) ? cur->burst_ms : 175u;

        if (chord) {
            uint8_t sol_mask  = 0u;
            float   freqs[6] = {0.0f};

            if (can_burst_primary) {
                sol_mask |= (1u << str);
                freqs[str] = target;
                burst_fired_primary = 1u;
            }
            if (can_burst_secondary) {
                sol_mask |= (1u << str2);
                freqs[str2] = target2;
                burst_fired_secondary = 1u;
            }
            if (sol_mask) {
                InterCore_TriggerSolenoidHzBatch(sol_mask, freqs,
                                                 SOLENOID_DEFAULT_DUTY_CYCLE,
                                                 SOLENOID_MODE_TIMED, burst_ms);
                printf("[TC] Chord actuate: %.1f/%.1f Hz burst=%ums\r\n",
                       (double)target, (double)target2, (unsigned)burst_ms);
            }
        } else if (can_burst_primary) {
            InterCore_TriggerSolenoidHz(str, target,
                                        SOLENOID_DEFAULT_DUTY_CYCLE,
                                        SOLENOID_MODE_TIMED, burst_ms, 0u);
            burst_fired_primary = 1u;
            printf("[TC] Actuate str%u: %.1f Hz burst=%ums\r\n",
                   (unsigned)str, (double)target, (unsigned)burst_ms);
        }

        burst_ms_for_start = burst_ms;
        start_primary      = new_note_primary;
        start_secondary    = new_note_secondary;
    }

    if (start_primary) {
        /* Log previous note's final state before transition resets it */
        {
            const TuningStringState_t *prev = &s_state[str];
            uint32_t prev_idx = SongSched_GetNoteIndex();
            if (prev_idx > 0u || prev->hop_count > 0u) {
                printf("[NOTE] t=%lu idx→%lu str%u tgt=%.2f Hz "
                       "state=%u hops=%lu lock=%u err=%.4f\r\n",
                       (unsigned long)HAL_GetTick(),
                       (unsigned long)prev_idx,
                       (unsigned)str, (double)target,
                       (unsigned)prev->state,
                       (unsigned long)prev->hop_count,
                       (unsigned)prev->lock_state,
                       (double)(prev->target_semi - prev->measured_semi));
            }
        }
        StartNoteTransition(str, target, burst_fired_primary, burst_ms_for_start);

        /* Sync note timer: duration counts from solenoid fire, not first tick */
        SongSched_ResetNoteTimer(HAL_GetTick());
    }
    if (chord && start_secondary) {
        StartNoteTransition(str2, target2,
                            burst_fired_secondary, burst_ms_for_start);
    }

    /* ------------------------------------------------------------------ */
    /* FFT measurement + control update (primary)                           */
    /* ------------------------------------------------------------------ */
    float measured = 0.0f, confidence = 0.0f;
    GetFFTRadarLock(str, s_state[str].expected_freq_hz, target,
                    &measured, &confidence);
    printf("[TC-DBG] str%u | tgt=%.2f exp=%.2f meas=%.2f mag=%.1f state=%u\r\n",
           (unsigned)str, (double)target,
           (double)s_state[str].expected_freq_hz,
           (double)measured, (double)confidence,
           (unsigned)s_state[str].state);
    TuningCtrl_Update(str, target, measured, confidence);

    /* ------------------------------------------------------------------ */
    /* FFT measurement + control update (secondary / chord)                 */
    /* ------------------------------------------------------------------ */
    if (chord) {
        float measured2 = 0.0f, confidence2 = 0.0f;
        GetFFTRadarLock(str2, s_state[str2].expected_freq_hz, target2,
                        &measured2, &confidence2);
        TuningCtrl_Update(str2, target2, measured2, confidence2);
    }

    /* ------------------------------------------------------------------ */
    /* Push motor commands — always send so CM4 has an up-to-date target    */
    /* ------------------------------------------------------------------ */
    {
        uint8_t  move_mask = (1u << str);
        int32_t  positions[NUM_MOTORS] = {0};
        positions[str] = s_state[str].cmd_steps_abs;

        if (chord) {
            move_mask       |= (1u << str2);
            positions[str2]  = s_state[str2].cmd_steps_abs;
        }

        InterCore_SendMotorGotoBatch(move_mask, positions);
    }

    /* ------------------------------------------------------------------ */
    /* Per-hop diagnostic logging                                            */
    /* [SNG] ts, note_idx, hop, state, tgt_hz, meas_hz, err_semi,           */
    /*       ff, pi_d, cmd, cur, mbusy, fft_w, res_w, lock, mag             */
    /* ------------------------------------------------------------------ */
    {
        const TuningStringState_t *ds = &s_state[str];
        float err_semi = ds->target_semi - ds->measured_semi;

        printf("[SNG] %lu,%lu,%lu,%u,%.2f,%.2f,%.4f,%.0f,%.1f,%ld,%ld,%u,%u,%u,%u,%.0f\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)SongSched_GetNoteIndex(),
               (unsigned long)ds->hop_count,
               (unsigned)ds->state,
               (double)target,
               (double)measured,
               (double)err_semi,
               (double)ds->ff_steps_abs,
               (double)ds->last_pi_delta,
               (long)ds->cmd_steps_abs,
               (long)ds->current_abs_steps,
               (unsigned)g_shared_memory.motor_is_moving[str],
               (unsigned)ds->fft_clear_wait_hops_remaining,
               (unsigned)ds->resonance_wait_hops_remaining,
               (unsigned)ds->lock_state,
               (double)confidence);
    }
}

/* =========================================================================
 * Getters
 * ========================================================================= */

int32_t TuningCtrl_GetCmdSteps(uint8_t string_id)
{
    if (string_id >= NUM_STRINGS) return 0;
    return s_state[string_id].cmd_steps_abs;
}

uint8_t TuningCtrl_GetLockState(uint8_t string_id)
{
    if (string_id >= NUM_STRINGS) return 0u;
    return s_state[string_id].lock_state;
}

const TuningStringState_t *TuningCtrl_GetStringState(uint8_t string_id)
{
    if (string_id >= NUM_STRINGS) return NULL;
    return &s_state[string_id];
}

/* =========================================================================
 * Reset helpers
 * ========================================================================= */

void TuningCtrl_ResetString(uint8_t string_id)
{
    if (string_id >= NUM_STRINGS) return;

    TuningStringState_t *s = &s_state[string_id];

    s->state              = TUNE_STATE_IDLE;
    s->resonance_wait_hops_remaining = 0u;
    s->fft_clear_wait_hops_remaining = 0u;
    s->integral_state     = 0.0f;
    s->ff_steps_abs       = 0.0f;
    s->lock_state         = 0u;
    s->prev_lock_state    = 0u;
    s->fault_flags        = 0u;
    s->stall_counter      = 0u;
    s->hop_count          = 0u;
    s->string_warm        = 0u;
    s->silent_hop_count   = 0u;
    s->chord_pluck_ready  = 0u;
    s->chord_pluck_wait_hops = 0u;

    /* Sentinel so new-note detection triggers on song restart */
    s->prev_target_semi   = -999.0f;
    s->target_semi        = 0.0f;
    s->measured_semi      = 0.0f;
    s->last_valid_freq_hz = 0.0f;
    s->expected_freq_hz   = s_open_freq_hz[string_id];
}

void TuningCtrl_ResetAll(void)
{
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        TuningCtrl_ResetString(i);
    }
    s_motor_enable_mask  = 0u;
    s_song_first_note    = 1u;
    s_cold_start_pending = 0u;
}

void TuningCtrl_SetDualTrackMode(uint8_t enable)
{
    s_dual_track_mode = enable ? 1u : 0u;
    printf("[TC] Dual-track mode %s\r\n", enable ? "ENABLED" : "DISABLED");
}

void TuningCtrl_SetTuning(uint8_t tuning_id)
{
    if (tuning_id >= NUM_TUNINGS) tuning_id = TUNING_STANDARD;
    s_active_tuning = tuning_id;

    /* Swap the feed-forward breakpoint tables */
    const float (*src_freq)[TUNING_FF_NUM_POINTS];
    const float (*src_steps)[TUNING_FF_NUM_POINTS];

    if (tuning_id == TUNING_DROP_D) {
        src_freq  = s_dropd_ff_freq_bp;
        src_steps = s_dropd_ff_steps_bp;
    } else if (tuning_id == TUNING_DROP_G2) {
        src_freq  = s_dropg2_ff_freq_bp;
        src_steps = s_dropg2_ff_steps_bp;
    } else {
        src_freq  = s_std_ff_freq_bp;
        src_steps = s_std_ff_steps_bp;
    }

    memcpy(tuning_ff_freq_bp,  src_freq,  sizeof(tuning_ff_freq_bp));
    memcpy(tuning_ff_steps_bp, src_steps, sizeof(tuning_ff_steps_bp));

    /* Re-fit the polynomial coefficients from the new breakpoints */
    Cal_FitFromBaseLUT();
}

uint8_t TuningCtrl_GetActiveTuning(void)
{
    return s_active_tuning;
}

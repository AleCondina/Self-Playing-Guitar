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

/* =========================================================================
 * Feed-forward LUT data (legacy — kept for SRAM calibration compat)
 * ========================================================================= */

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

/* =========================================================================
 * Semitone-domain polynomial coefficients
 *
 * Feedforward: steps = a·semi² + b·semi + c
 * Plant model: semi  = a·steps² + b·steps + c
 *
 * E2 (string 0) seeded from MATLAB identification.
 * Others seeded from legacy cal_poly_coeff via Cal_FitSemiPolynomial().
 * ========================================================================= */

float tuning_ff_semi_coeff[NUM_STRINGS][3] = {
    /* E2: from MATLAB system identification */
    { 29.8134f, 424.929f, -70.6621f },
    /* A2–E4: zeroed — seeded at boot from cal_poly_coeff or calibration */
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
};

float tuning_plant_coeff[NUM_STRINGS][3] = {
    /* E2: from MATLAB system identification */
    { -1.2433e-07f, 2.0937e-03f, 0.1599f },
    /* A2–E4: zeroed — seeded at boot from calibration */
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
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

/* =========================================================================
 * Domain conversion helpers
 * ========================================================================= */

static const float s_open_freq_hz[NUM_STRINGS] = {
    E2_HZ, A2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ
};

/** Hz → semitones relative to string's open frequency. */
static inline float freq_to_semi(uint8_t str, float freq_hz)
{
    if (freq_hz <= 0.0f) return 0.0f;
    return 12.0f * log2f(freq_hz / s_open_freq_hz[str]);
}

/** Semitones → Hz. */
static inline float semi_to_freq(uint8_t str, float semi)
{
    return s_open_freq_hz[str] * powf(2.0f, semi / 12.0f);
}

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static inline float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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
 * State machine: IDLE → FF_MOVE → FF_WAIT → PI_ACTIVE ↔ IN_TUNE
 * ========================================================================= */

void TuningCtrl_Update(uint8_t string_id,
                       float   target_freq_hz,
                       float   measured_freq_hz,
                       float   confidence)
{
    if (string_id >= NUM_STRINGS) return;

    TuningStringState_t *s = &s_state[string_id];

    /* ------------------------------------------------------------------ */
    /* 1. Convert to semitone domain                                        */
    /* ------------------------------------------------------------------ */
    float target_semi = freq_to_semi(string_id, target_freq_hz);
    s->target_semi    = target_semi;

    uint8_t measurement_trusted = (measured_freq_hz > 0.0f);
    float   meas_semi = 0.0f;

    if (measurement_trusted) {
        meas_semi             = freq_to_semi(string_id, measured_freq_hz);
        s->measured_semi      = meas_semi;
        s->last_valid_freq_hz = measured_freq_hz;
        s->silent_hop_count   = 0u;
        s->string_warm        = 1u;
    } else {
        meas_semi = s->measured_semi;  /* hold last valid */
        s->silent_hop_count++;
        if (s->silent_hop_count >= (uint16_t)SONG_DECAY_COLD_HOPS) {
            s->string_warm = 0u;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 2. Note-change detection → FF_MOVE                                   */
    /* ------------------------------------------------------------------ */
    const float NOTE_CHANGE_SEMI = 0.02f;  /* ~2 cents threshold */
    uint8_t is_new_note = (fabsf(target_semi - s->prev_target_semi)
                           > NOTE_CHANGE_SEMI);

    if (is_new_note) {
        /* Reset controller for new note */
        s->integral_state   = 0.0f;
        s->lock_state       = 0u;
        s->prev_lock_state  = 0u;
        s->stall_counter    = 0u;
        s->fault_flags      = 0u;
        s->hop_count        = 0u;
        s->prev_target_semi = target_semi;

        /* Compute feedforward position */
        s->ff_steps_abs = feedforward_steps(string_id, target_semi);
        s->state        = TUNE_STATE_FF_MOVE;
    }

    /* ------------------------------------------------------------------ */
    /* 3. State machine                                                     */
    /* ------------------------------------------------------------------ */
    float pi_delta = 0.0f;  /* PI output (steps), added to FF position */

    switch (s->state) {

    case TUNE_STATE_IDLE:
        /* No target — nothing to do */
        break;

    case TUNE_STATE_FF_MOVE:
        /* FF position was set above (new note) or by re-trigger below.
         * Transition to FF_WAIT — wait for first valid FFT measurement. */
        s->integral_state = 0.0f;
        s->state = TUNE_STATE_FF_WAIT;
        break;

    case TUNE_STATE_FF_WAIT:
        /* Wait for first valid FFT after the FF move.  The motor is
         * transiting to the FF position; the first FFT reading tells us
         * how close we got. */
        if (measurement_trusted) {
            s->state = TUNE_STATE_PI_ACTIVE;
        }
        break;

    case TUNE_STATE_PI_ACTIVE: {
        if (!measurement_trusted) break;

        float e = target_semi - meas_semi;
        float abs_e = fabsf(e);

        /* Re-trigger feedforward if error is too large */
        if (abs_e > TUNING_FF_LOCK_SEMI) {
            s->integral_state = 0.0f;
            s->ff_steps_abs   = feedforward_steps(string_id, target_semi);
            s->state          = TUNE_STATE_FF_MOVE;
            break;
        }

        /* Deadband → IN_TUNE */
        if (abs_e < TUNING_DEADBAND_SEMI) {
            s->lock_state = 1u;
            s->state      = TUNE_STATE_IN_TUNE;
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

    /* ------------------------------------------------------------------ */
    /* 4. Compute raw commanded position: FF + PI output                    */
    /* ------------------------------------------------------------------ */
    float raw_cmd = s->ff_steps_abs + pi_delta;

    /* ------------------------------------------------------------------ */
    /* 5. Minimum move threshold (defeats backlash dither)                   */
    /* ------------------------------------------------------------------ */
    float delta_from_prev = raw_cmd - (float)s->prev_cmd_steps_abs;
    if (fabsf(delta_from_prev) < TUNING_MIN_MOVE_STEPS &&
        s->state != TUNE_STATE_FF_MOVE) {
        raw_cmd = (float)s->prev_cmd_steps_abs;
    }

    /* ------------------------------------------------------------------ */
    /* 6. Slew limit                                                        */
    /* ------------------------------------------------------------------ */
    float max_delta = TUNING_MAX_STEP_RATE_SPS * TUNING_SAMPLE_TIME_S;
    float prev_cmd_f = (float)s->prev_cmd_steps_abs;
    float delta      = raw_cmd - prev_cmd_f;
    delta            = Clampf(delta, -max_delta, max_delta);
    float slewed_cmd = prev_cmd_f + delta;

    /* ------------------------------------------------------------------ */
    /* 7. Mechanical bounds                                                  */
    /* ------------------------------------------------------------------ */
    slewed_cmd = Clampf(slewed_cmd,
                        (float)TUNING_STEPS_MIN_DEFAULT,
                        (float)TUNING_STEPS_MAX_DEFAULT);

    /* ------------------------------------------------------------------ */
    /* 8. Store final command                                               */
    /* ------------------------------------------------------------------ */
    s->prev_cmd_steps_abs = s->cmd_steps_abs;
    s->cmd_steps_abs      = (int32_t)roundf(slewed_cmd);

    /* Mirror actual motor position from shared memory */
    s->current_abs_steps = g_shared_memory.current_positions[string_id];

    /* Update expected frequency from commanded position (for FFT search).
     * Use plant model: semi → freq, via inverse polynomial. */
    float pred_semi = plant_steps_to_semi(string_id, (float)s->cmd_steps_abs);
    s->expected_freq_hz = semi_to_freq(string_id, pred_semi);

    /* ------------------------------------------------------------------ */
    /* 9. Stall detection                                                   */
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
    /* 10. Lock edge detection (for chord sync)                             */
    /* ------------------------------------------------------------------ */
    s->prev_lock_state = s->lock_state;

    /* ------------------------------------------------------------------ */
    /* 11. Increment hop counter                                            */
    /* ------------------------------------------------------------------ */
    s->hop_count++;
}

/* =========================================================================
 * TuningCtrl_RunHop — high-level entry point
 *
 * Reads song scheduler + FFT, calls TuningCtrl_Update() for each active
 * string, then issues inter-core motor and solenoid commands.
 * ========================================================================= */

void TuningCtrl_RunHop(void)
{
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
    /* Note-change detection (for solenoid fire)                            */
    /* ------------------------------------------------------------------ */
    float prev_target_semi_1 = s_state[str].prev_target_semi;
    float new_target_semi_1  = freq_to_semi(str, target);
    uint8_t new_note_primary = (fabsf(new_target_semi_1 - prev_target_semi_1)
                                > 0.02f);

    uint8_t new_note_secondary = 0u;
    if (chord) {
        float prev_target_semi_2 = s_state[str2].prev_target_semi;
        float new_target_semi_2  = freq_to_semi(str2, target2);
        new_note_secondary = (fabsf(new_target_semi_2 - prev_target_semi_2)
                              > 0.02f);
    }

    /* ------------------------------------------------------------------ */
    /* FFT measurement + control update (primary)                           */
    /* ------------------------------------------------------------------ */
    float measured = 0.0f, confidence = 0.0f;
    GetFFTRadarLock(str, s_state[str].expected_freq_hz, target,
                    &measured, &confidence);
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
    /* Cold-start burst — deferred for the very first note of the song      */
    /* Gives CM4 time to process motor-enable before solenoid fires.        */
    /* ------------------------------------------------------------------ */
    if (s_song_first_note && (new_note_primary || new_note_secondary)) {
        s_song_first_note    = 0u;
        s_cold_start_pending = COLD_START_DEFER_HOPS;
    }

    if (s_cold_start_pending > 0u) {
        s_cold_start_pending--;

        if (s_cold_start_pending == 0u) {
            /* Fire the initial burst */
            const SongNote_t *cur = SongSched_GetNoteAt(SongSched_GetNoteIndex());
            uint16_t burst_ms = (cur && cur->burst_ms > 0u) ? cur->burst_ms : 175u;

            if (chord) {
                uint8_t sol_mask    = 0u;
                float   freqs[6]   = {0.0f};
                if (target  >= 40.0f && target  <= 800.0f) {
                    sol_mask |= (1u << str);  freqs[str]  = target;
                }
                if (target2 >= 40.0f && target2 <= 800.0f) {
                    sol_mask |= (1u << str2); freqs[str2] = target2;
                }
                if (sol_mask) {
                    InterCore_TriggerSolenoidHzBatch(sol_mask, freqs,
                                                     SOLENOID_DEFAULT_DUTY_CYCLE,
                                                     SOLENOID_MODE_TIMED, burst_ms);
                }
            } else {
                if (target >= 40.0f && target <= 800.0f) {
                    InterCore_TriggerSolenoidHz(str, target,
                                                SOLENOID_DEFAULT_DUTY_CYCLE,
                                                SOLENOID_MODE_TIMED, burst_ms, 0u);
                }
            }
            printf("[TC] Cold-start burst: %u ms at %.1f Hz (deferred %u hops)\r\n",
                   (unsigned)burst_ms, (double)target,
                   (unsigned)COLD_START_DEFER_HOPS);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Solenoid actuation — fire immediately on note change                 */
    /*                                                                      */
    /* Every new note gets a solenoid burst at the TARGET frequency.         */
    /* The burst fires on the same hop as the FF move — motor transits      */
    /* during the burst.  No BPM-delayed actuation.                         */
    /* ------------------------------------------------------------------ */
    if (!s_cold_start_pending) {
        if (new_note_primary || new_note_secondary) {
            const SongNote_t *cur = SongSched_GetNoteAt(SongSched_GetNoteIndex());
            uint16_t burst_ms = (cur && cur->burst_ms > 0u) ? cur->burst_ms : 72u;

            if (chord) {
                uint8_t sol_mask  = 0u;
                float   freqs[6] = {0.0f};

                if (new_note_primary && target >= 40.0f && target <= 800.0f) {
                    sol_mask |= (1u << str);  freqs[str]  = target;
                }
                if (new_note_secondary && target2 >= 40.0f && target2 <= 800.0f) {
                    sol_mask |= (1u << str2); freqs[str2] = target2;
                }
                if (sol_mask) {
                    InterCore_TriggerSolenoidHzBatch(sol_mask, freqs,
                                                     SOLENOID_DEFAULT_DUTY_CYCLE,
                                                     SOLENOID_MODE_TIMED, burst_ms);
                    printf("[TC] Chord actuate: %.1f/%.1f Hz burst=%ums\r\n",
                           (double)target, (double)target2, (unsigned)burst_ms);
                }
            } else if (new_note_primary) {
                if (target >= 40.0f && target <= 800.0f) {
                    InterCore_TriggerSolenoidHz(str, target,
                                                SOLENOID_DEFAULT_DUTY_CYCLE,
                                                SOLENOID_MODE_TIMED, burst_ms, 0u);
                    printf("[TC] Actuate str%u: %.1f Hz burst=%ums\r\n",
                           (unsigned)str, (double)target, (unsigned)burst_ms);
                }
            }
        }
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
    /* Debug CSV logging (decimated)                                        */
    /* Format: timestamp, str, state, target_semi, meas_semi, err_semi,     */
    /*         ff_steps, cmd_steps, cur_steps, lock, confidence, faults     */
    /* ------------------------------------------------------------------ */
    s_debug_counter++;
    if (s_debug_counter >= DEBUG_DECIM_HOPS) {
        s_debug_counter = 0u;

        const TuningStringState_t *ds = &s_state[str];
        float err_semi = ds->target_semi - ds->measured_semi;

        printf("[TC] %lu,%u,%u,%.3f,%.3f,%.4f,%.1f,%ld,%ld,%u,%.0f,%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned)str,
               (unsigned)ds->state,
               (double)ds->target_semi,
               (double)ds->measured_semi,
               (double)err_semi,
               (double)ds->ff_steps_abs,
               (long)ds->cmd_steps_abs,
               (long)ds->current_abs_steps,
               (unsigned)ds->lock_state,
               (double)confidence,
               (unsigned)ds->fault_flags);
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

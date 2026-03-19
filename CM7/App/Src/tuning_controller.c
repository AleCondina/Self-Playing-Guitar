/**
 ******************************************************************************
 * @file    tuning_controller.c
 * @brief   Closed-loop guitar string tuning controller — CM7 implementation
 *
 * See tuning_controller.h for the full architecture description.
 *
 * Key interactions with the rest of the codebase
 * -----------------------------------------------
 *  fft_processing       – active_frequencies[i] / active_frequencies[i]
 *  song_scheduler       – SongSched_GetStringId() / SongSched_GetTargetFreq()
 *  inter_core_cm7       – InterCore_SendMotorGoto(), InterCore_TriggerSolenoids()
 *                          InterCore_SendCommand (solenoid stop)
 *  shared_memory        – g_shared_memory.current_positions[] (motor feedback)
 *
 * Feed-forward LUT calibration (TODO)
 * ------------------------------------
 * The tables below contain PLACEHOLDER values derived from standard guitar
 * tuning.  They assume:
 *   position 0  = motor at home (open string)
 *   positive    = increased tension (higher pitch)
 *
 * To calibrate:
 *  1. Home each motor (Motor_Home()).
 *  2. Slowly advance in steps and record frequency from FFT.
 *  3. Fill in tuning_ff_freq_bp and tuning_ff_steps_bp per string.
 *  4. Re-run with a sine sweep to verify linearity between breakpoints.
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

/* Tuning tolerances - adjust these based on your physical guitar */
#define TUNE_SEARCH_WINDOW_HZ 15.0f     /* Max allowed drift from target before ignoring */

/* =========================================================================
 * Feed-forward LUT data
 *
 * Layout: [string_id][breakpoint_index]
 * tuning_ff_freq_bp – target frequency at each breakpoint (Hz) TODO: Use macros in application_config
 * tuning_ff_steps_bp – absolute motor position at each breakpoint (microsteps)
 *
 * String order: 0=E2(82Hz), 1=A2(110Hz), 2=D3(147Hz),
 *               3=G3(196Hz), 4=B3(247Hz), 5=E4(330Hz)
 *
 * TODO: replace with measured values during hardware commissioning.
 *       Current values assume ~10 microsteps per Hz as a rough linear first-guess.
 * ========================================================================= */
float tuning_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* Measured frequencies from LUT RUN 1 */
    /* E2 frets 0-6 */ {  82.41f,  82.69f,  89.97f,  96.05f, 100.07f, 106.37f, 112.82f },
    /* A2 frets 0-6 */ { 110.00f, 115.54f, 121.57f, 127.57f, 139.23f, 145.69f, 151.22f },
    /* D3 frets 0-6 */ { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    /* G3 frets 0-6 */ { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    /* B3 frets 0-6 */ { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    /* E4 frets 0-6 */ { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* Measured microsteps from LUT RUN 1 */
    /* E2 */ {   0.0f, 267.0f, 1080.0f, 1623.0f, 2029.0f, 2706.0f, 3385.0f },
    /* A2 */ {   0.0f,  751.0f, 1549.0f, 2370.0f, 4103.0f, 5013.0f, 5955.0f },
    /* D3 */ {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    /* G3 */ {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    /* B3 */ {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    /* E4 */ {   0.0f, 100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

/* =========================================================================
 * Private state
 * ========================================================================= */

static TuningStringState_t s_state[NUM_STRINGS];

/* Decimation counter for debug printing (print every N hops) */
#define DEBUG_DECIM_HOPS    5u
static uint32_t s_debug_counter = 0u;

/* Bitmask of motors currently enabled by the tuning controller.
 * Used to detect transitions and avoid redundant enable commands. */
static uint8_t s_motor_enable_mask = 0u;

/* Flag: 1 = the very first note of the song has not yet been plucked.
 * Set by TuningCtrl_Init() / song start, cleared after the initial burst. */
static uint8_t s_song_first_note = 1u;

/* Deferred cold-start burst: count down hops before firing the initial burst.
 * 0 = not pending.  Set to COLD_START_DEFER_HOPS on first note detection,
 * decremented each hop, burst fires when it reaches 1.  This gives CM4
 * time to process the motor-enable command before the solenoid fires. */
#define COLD_START_DEFER_HOPS  2u
static uint8_t s_cold_start_pending = 0u;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief Clamp a float value between lo and hi.
 */
static inline float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief Look up absolute feed-forward steps for a given target frequency.
 *
 * Performs linear interpolation between the nearest breakpoints.
 * Clips to the first or last breakpoint value if outside the table range.
 *
 * @param  str  String index 0–5
 * @param  freq Target frequency in Hz
 * @return Interpolated absolute step position (float microsteps)
 */
static float FF_Interpolate(uint8_t str, float freq)
{
    /* Evaluate the calibrated quadratic: steps = a + b·f + c·f²
     * cal_poly_coeff is seeded at boot from the base LUT (Cal_FitFromBaseLUT)
     * and updated after each calibration run (Cal_FitPolynomial). */
    float a     = cal_poly_coeff[str][0];
    float b     = cal_poly_coeff[str][1];
    float c     = cal_poly_coeff[str][2];
    float steps = a + b * freq + c * freq * freq;
    if (steps < 0.0f) steps = 0.0f;

    /* Apply tune offset — compensates for string being flat or sharp
     * relative to standard tuning.  Set by the 'tune' command. */
    steps += (float)Cal_TuneGetOffset(str);

    return steps;
}

/**
 * @brief Inverse polynomial: given a motor position in microsteps, estimate
 *        the frequency the string should be producing.
 *
 * Solves:  steps = a + b·f + c·f²  →  c·f² + b·f + (a - steps) = 0
 * Returns the positive root via the quadratic formula, or a linear
 * fallback when c ≈ 0.
 *
 * @param  str   String index 0–5
 * @param  steps Current absolute motor position (microsteps)
 * @return Estimated frequency in Hz (clamped to open-string freq minimum)
 */
static float FF_InverseInterpolate(uint8_t str, float steps)
{
    float a = cal_poly_coeff[str][0];
    float b = cal_poly_coeff[str][1];
    float c = cal_poly_coeff[str][2];

    /* Open-string frequency is the minimum possible */
    float f_open = tuning_ff_freq_bp[str][0];

    if (fabsf(c) < 1e-9f) {
        /* Linear: steps = a + b·f  →  f = (steps - a) / b */
        if (fabsf(b) < 1e-9f) return f_open;
        float f = (steps - a) / b;
        return (f > f_open) ? f : f_open;
    }

    /* Quadratic: c·f² + b·f + (a - steps) = 0 */
    float disc = b * b - 4.0f * c * (a - steps);
    if (disc < 0.0f) return f_open;

    float sqrt_disc = sqrtf(disc);
    /* Take the root that gives a positive, physically meaningful frequency */
    float f1 = (-b + sqrt_disc) / (2.0f * c);
    float f2 = (-b - sqrt_disc) / (2.0f * c);

    /* Pick the root closest to the open-string range */
    float f = (f1 >= f_open) ? f1 : f2;
    if (f < f_open) f = f_open;

    return f;
}

/* =========================================================================
 * Public API implementation
 * ========================================================================= */

void TuningCtrl_Init(void)
{
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        TuningStringState_t *s = &s_state[i];
        memset(s, 0, sizeof(TuningStringState_t));

        /* Safe starting position is home (open string) */
        s->cmd_steps_abs      = 0;
        s->prev_cmd_steps_abs = 0;
        s->current_abs_steps  = 0;

        /* Start with open-string FF steps (freq = breakpoint[0]) */
        s->ff_steps_abs = tuning_ff_steps_bp[i][0];

        /* Sentinel: no note yet */
        s->last_target_freq_hz = -1.0f;
        s->last_valid_freq_hz  =  0.0f;

        /* New tracking state */
        s->expected_freq_hz = tuning_ff_freq_bp[i][0];
        s->pi_hop_count     = 0u;
        s->note_actuated    = 0u;
    }
    s_debug_counter      = 0u;
    s_motor_enable_mask  = 0u;
    s_song_first_note    = 1u;
    s_cold_start_pending = 0u;
}

/* -------------------------------------------------------------------------
 * TuningCtrl_Update
 * Single-string per-hop update.  All maths is here; no HAL calls.
 * HAL/inter-core calls are made by TuningCtrl_RunHop().
 * --------------------------------------------------------------------- */
void TuningCtrl_Update(uint8_t string_id,
                       float   target_freq_hz,
                       float   measured_freq_hz,
                       float   confidence)
{
    if (string_id >= NUM_STRINGS) return;

    TuningStringState_t *s = &s_state[string_id];

    /* ------------------------------------------------------------------ */
    /* 1. Note-change detection                                             */
    /* ------------------------------------------------------------------ */
    const float NOTE_CHANGE_THRESHOLD_HZ = 0.05f;
    uint8_t is_new_note = (fabsf(target_freq_hz - s->last_target_freq_hz)
                           > NOTE_CHANGE_THRESHOLD_HZ);

    if (is_new_note) {
        /* Reset PI state immediately */
        s->integral_state    = 0.0f;
        s->trim_steps        = 0.0f;
        s->lock_state        = 0u;
        s->pi_enable         = 0u;
        s->stall_counter     = 0u;
        s->fault_flags       = 0u;

        /* Warm/cold adaptive hold:
         * Warm string → FFT already has live signal → PI can start after 1 hop.
         * Cold string → solenoid burst fires first (in TuningCtrl_RunHop) →
         *               keep full 3-hop hold while FFT acquires resonance.      */
        uint16_t hold = s->string_warm ? (uint16_t)SONG_WARM_DELAY_FRAMES
                                       : (uint16_t)SONG_COLD_DELAY_FRAMES;
        s->delay_hold_counter = hold;

        /* Reset per-note pluck tracking */
        s->note_plucked     = 0u;
        s->hops_since_pluck = 0u;

        /* Reset per-note PI convergence tracking */
        s->pi_hop_count  = 0u;
        s->note_actuated = 0u;

        /* Update FF position immediately so motor starts moving */
        s->ff_steps_abs        = FF_Interpolate(string_id, target_freq_hz);
        s->last_target_freq_hz = target_freq_hz;

    } else if (s->delay_hold_counter > 0u) {
        /* Still in post-note-change hold period */
        s->delay_hold_counter--;
        s->pi_enable = 0u;

    } else {
        /* Normal operation */
        s->pi_enable = 1u;
    }

    /* ------------------------------------------------------------------ */
    /* 2. Measurement gate                                                  */
    /* Peaks in active_frequencies[] are already SNR-vetted by              */
    /* FFT_ProcessBlock's adaptive per-bin noise floor.  We only need to    */
    /* check that GetFFTRadarLock returned a valid measurement (non-zero).  */
    /* ------------------------------------------------------------------ */
    uint8_t measurement_trusted = (measured_freq_hz > 0.0f);

    float use_freq;
    if (measurement_trusted) {
        s->last_valid_freq_hz = measured_freq_hz;
        use_freq = measured_freq_hz;

        /* String is (or has become) warm */
        s->silent_hop_count = 0u;
        s->string_warm      = 1u;
    } else {
        /* Hold last valid estimate; do NOT update PI integral */
        use_freq = s->last_valid_freq_hz;

        /* Track consecutive silent hops; declare cold after threshold */
        s->silent_hop_count++;
        if (s->silent_hop_count >= (uint16_t)SONG_DECAY_COLD_HOPS) {
            s->string_warm = 0u;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 3. Frequency error                                                   */
    /* ------------------------------------------------------------------ */
    float e = target_freq_hz - use_freq;

    /* ------------------------------------------------------------------ */
    /* 4. Lock-band hysteresis (Hz-based, absolute)                         */
    /* ------------------------------------------------------------------ */
    float abs_e = fabsf(e);
    float lock_in_hz  = TUNING_LOCK_BAND_IN_HZ;
    float lock_out_hz = TUNING_LOCK_BAND_OUT_HZ;

    if (s->lock_state) {
        if (abs_e >= lock_out_hz) {
            s->lock_state = 0u;
        }
    } else {
        if (abs_e <= lock_in_hz) {
            s->lock_state = 1u;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 5. Discrete PI with anti-windup (back-calculation)                  */
    /* Integrator only accumulates when PI enabled AND measurement trusted  */
    /* ------------------------------------------------------------------ */
    if (s->pi_enable && measurement_trusted) {
        /* Only accumulate integral when error is already small (like tune).
         * This prevents windup from large transient errors. */
        if (abs_e <= TUNING_INTEGRAL_ENABLE_HZ) {
            s->integral_state += TUNING_KI * e * TUNING_SAMPLE_TIME_S;
        } else {
            /* Reset integrator when far from target — start fresh */
            s->integral_state = 0.0f;
        }

        /* Compute raw PI output */
        float raw_trim = TUNING_KP * e + s->integral_state;

        /* Step-cap: scale maximum correction with error size.
         * More aggressive in the medium-error range to converge faster
         * while still preventing overshoot when close to target. */
        float step_cap;
        if (abs_e > 40.0f)      step_cap = TUNING_OUT_LIMIT_STEPS;
        else if (abs_e > 15.0f) step_cap = 1600.0f;
        else if (abs_e > 5.0f)  step_cap = 640.0f;
        else if (abs_e > 1.0f)  step_cap = 240.0f;
        else                    step_cap = 120.0f;

        /* Clamp output with adaptive step cap */
        float clamped_trim = Clampf(raw_trim, -step_cap, step_cap);

        /* Back-calculate anti-windup: reduce integrator by the clamped amount */
        if (raw_trim != clamped_trim) {
            s->integral_state -= (raw_trim - clamped_trim);
            s->fault_flags    |= TUNING_FAULT_SATURATED;
        } else {
            s->fault_flags &= (uint8_t)~TUNING_FAULT_SATURATED;
        }

        /* Minimum-move threshold to defeat backlash dither.
         * Only suppress very small incremental changes; allow large corrections. */
        float delta_trim = clamped_trim - s->trim_steps;
        if (fabsf(delta_trim) >= TUNING_MIN_MOVE_STEPS) {
            s->trim_steps = clamped_trim;
        }
        /* else: keep s->trim_steps as-is this hop */
    }
    /* Count PI hops for convergence limit */
    if (s->pi_enable && measurement_trusted) {
        s->pi_hop_count++;
    }

    /* When PI is disabled: hold trim_steps at its last value (do not zero it).
     * This ensures the motor stays where it was, not at the FF position alone. */

    /* ------------------------------------------------------------------ */
    /* 6. Compute raw commanded position                                    */
    /* ------------------------------------------------------------------ */
    float raw_cmd = s->ff_steps_abs + s->trim_steps;

    /* ------------------------------------------------------------------ */
    /* 7. Slew limit                                                        */
    /* Prevents requesting more position change per hop than the motor can  */
    /* physically achieve.                                                  */
    /* ------------------------------------------------------------------ */
    float max_delta = TUNING_MAX_STEP_RATE_SPS * TUNING_SAMPLE_TIME_S;
    float prev_cmd_f = (float)s->prev_cmd_steps_abs;
    float delta      = raw_cmd - prev_cmd_f;
    delta            = Clampf(delta, -max_delta, max_delta);
    float slewed_cmd = prev_cmd_f + delta;

    /* ------------------------------------------------------------------ */
    /* 8. Mechanical bounds                                                  */
    /* ------------------------------------------------------------------ */
    slewed_cmd = Clampf(slewed_cmd,
                        (float)TUNING_STEPS_MIN_DEFAULT,
                        (float)TUNING_STEPS_MAX_DEFAULT);

    /* ------------------------------------------------------------------ */
    /* 9. Store final command                                               */
    /* ------------------------------------------------------------------ */
    s->prev_cmd_steps_abs = s->cmd_steps_abs;
    s->cmd_steps_abs      = (int32_t)roundf(slewed_cmd);

    /* Mirror actual motor position from shared memory */
    s->current_abs_steps = g_shared_memory.current_positions[string_id];

    /* Update expected frequency from the COMMANDED motor position.
     * Using cmd_steps_abs (not current_abs_steps from CM4 feedback)
     * eliminates the inter-core lag — the search window moves
     * immediately when a new position is commanded, rather than
     * waiting for the motor to physically arrive. */
    s->expected_freq_hz = FF_InverseInterpolate(string_id,
                                                 (float)s->cmd_steps_abs);

    /* ------------------------------------------------------------------ */
    /* 10. Stall detection                                                  */
    /* Large ongoing PI output with no measurable frequency change →        */
    /* motor may be stalled.                                                */
    /* ------------------------------------------------------------------ */
    if (!s->lock_state && s->pi_enable && measurement_trusted) {
        float abs_trim = fabsf(s->trim_steps);
        float freq_delta = fabsf(measured_freq_hz - s->last_valid_freq_hz);

        if ((abs_trim > (TUNING_OUT_LIMIT_STEPS * 0.5f)) &&
            (freq_delta < TUNING_STALL_MIN_FREQ_DELTA_HZ)) {
            s->stall_counter++;
        } else {
            s->stall_counter = 0u;
        }

        if (s->stall_counter >= TUNING_STALL_TIMEOUT_HOPS) {
            s->fault_flags |= TUNING_FAULT_STALL;
            /* Stop issuing large commands; let motor rest */
            s->cmd_steps_abs = s->current_abs_steps;
        }
    } else {
        s->stall_counter = 0u;
        s->fault_flags  &= (uint8_t)~TUNING_FAULT_STALL;
    }

    /* ------------------------------------------------------------------ */
    /* 11. Increment hop counter                                            */
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
static const float s_open_string_hz[NUM_STRINGS] = {
    82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f
};
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
        if (fabsf(freq - s_open_string_hz[s]) <= OPEN_STRING_REJECT_HZ)
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
 * TuningCtrl_RunHop
 * High-level entry point.  Reads from song scheduler and FFT, calls
 * TuningCtrl_Update(), then issues inter-core commands.
 * --------------------------------------------------------------------- */
void TuningCtrl_RunHop(void)
{
    /* ------------------------------------------------------------------ */
    /* Microstep safeguard: verify all configured motors are in 1/16 mode.  */
    /* If any motor is wrong, halt the controller and warn over UART.       */
    /* Running with incorrect microstepping will produce wrong position      */
    /* commands that can damage the mechanical system.                       */
    /* ------------------------------------------------------------------ */
    for (uint8_t m = 0u; m < 4u; m++) {  /* Motors 0-3 are configured */
        if (g_shared_memory.motor_microstep_status[m] != TUNING_REQUIRED_MICROSTEP_MODE) {
            printf("[TC] *** HALT: Motor %u microstep mode is %u, expected %u (1/16). "
                   "Tuning controller disabled to prevent damage. ***\r\n",
                   (unsigned)m,
                   (unsigned)g_shared_memory.motor_microstep_status[m],
                   (unsigned)TUNING_REQUIRED_MICROSTEP_MODE);
            /* Disable all motors as a safety measure */
            if (s_motor_enable_mask != 0u) {
                InterCore_SendMotorEnableMask(0u);
                s_motor_enable_mask = 0u;
            }
            return;
        }
    }

    /* Get current note from song scheduler */
    if (!SongSched_IsActive()) {
        /* Song stopped: disable all motors the tuning controller enabled */
        if (s_motor_enable_mask != 0u) {
            InterCore_SendMotorEnableMask(0u);
            s_motor_enable_mask = 0u;
        }
        return;
    }

    uint8_t  str    = SongSched_GetStringId();
    float    target = SongSched_GetTargetFreq();

    /* --- Motor power management: enable only active string motors --- */
    uint8_t needed_mask = (1u << str);
    if (SongSched_GetStringCount() >= 2u) {
        needed_mask |= (1u << SongSched_GetStringId2());
    }
    if (needed_mask != s_motor_enable_mask) {
        InterCore_SendMotorEnableMask(needed_mask);
        s_motor_enable_mask = needed_mask;
    }

    /* ------------------------------------------------------------------ */
    /* Detect note changes (primary and secondary strings)                  */
    /* ------------------------------------------------------------------ */
    uint8_t new_note_primary = (fabsf(target - s_state[str].last_target_freq_hz)
                                 > 0.05f);

    /* --- Primary String FFT Lock --- */
    float measured = 0.0f;
    float confidence = 0.0f;
    GetFFTRadarLock(str, s_state[str].expected_freq_hz, target, &measured, &confidence);

    /* Run one hop of the controller */
    TuningCtrl_Update(str, target, measured, confidence);

    /* ------------------------------------------------------------------ */
    /* Second string (chord support) — run control update first so both    */
    /* motor commands can be batched into a single inter-core notification. */
    /* ------------------------------------------------------------------ */
    uint8_t  str2          = 0u;
    float    target2       = 0.0f;
    uint8_t  new_note_secondary = 0u;
    uint8_t  chord = (SongSched_GetStringCount() >= 2u);

    if (chord) {
        str2    = SongSched_GetStringId2();
        target2 = SongSched_GetTargetFreq2();

        new_note_secondary = (fabsf(target2 - s_state[str2].last_target_freq_hz)
                               > 0.05f);

        float measured2 = 0.0f, confidence2 = 0.0f;
        GetFFTRadarLock(str2, s_state[str2].expected_freq_hz, target2, &measured2, &confidence2);
        TuningCtrl_Update(str2, target2, measured2, confidence2);
    }

    /* ------------------------------------------------------------------ */
    /* Cold-start burst — ONLY for the very first note of the song.        */
    /*                                                                      */
    /* DEFERRED by COLD_START_DEFER_HOPS to give CM4 time to process the   */
    /* motor-enable command before the solenoid fires.  On hop 0 we just   */
    /* arm the counter; the actual burst fires on hop COLD_START_DEFER_HOPS */
    /* when CM4 has had several 1ms ticks to enable the motors and settle.  */
    /* ------------------------------------------------------------------ */

    /* Arm the deferred burst on the first note detection */
    if (s_song_first_note && (new_note_primary || new_note_secondary)) {
        s_song_first_note    = 0u;
        s_cold_start_pending = COLD_START_DEFER_HOPS;
    }

    /* Count down and fire when ready */
    if (s_cold_start_pending > 0u) {
        s_cold_start_pending--;

        if (s_cold_start_pending == 0u) {
            /* Actually fire the burst NOW */
            uint16_t burst_ms = (uint16_t)SONG_INITIAL_PLUCK_MS;  /* 1000 ms */

            float ef1 = s_state[str].expected_freq_hz;
            float f   = (ef1 >= 40.0f && ef1 <= 800.0f) ? ef1 : 0.0f;

            if (chord) {
                uint8_t burst_mask  = 0u;
                float   burst_freqs[6] = {0.0f};
                float ef2 = s_state[str2].expected_freq_hz;
                float f2  = (ef2 >= 40.0f && ef2 <= 800.0f) ? ef2 : 0.0f;
                if (f  > 1e-6f) { burst_mask |= (1u << str);  burst_freqs[str]  = f;  }
                if (f2 > 1e-6f) { burst_mask |= (1u << str2); burst_freqs[str2] = f2; }
                if (burst_mask) {
                    InterCore_TriggerSolenoidHzBatch(burst_mask, burst_freqs,
                                                     SOLENOID_DEFAULT_DUTY_CYCLE,
                                                     SOLENOID_MODE_TIMED, burst_ms);
                }
            } else {
                if (f > 1e-6f) {
                    InterCore_TriggerSolenoidHz(str, f, SOLENOID_DEFAULT_DUTY_CYCLE,
                                                SOLENOID_MODE_TIMED, burst_ms, 0u);
                }
            }
            /* Mark note 0 as already actuated — no second fire at 1.5 s */
            s_state[str].note_actuated = 1u;
            s_state[str].note_plucked  = 1u;
            if (chord) {
                s_state[str2].note_actuated = 1u;
                s_state[str2].note_plucked  = 1u;
            }
            printf("[TC] Cold-start burst: %u ms at %.1f Hz (deferred %u hops)\r\n",
                   (unsigned)burst_ms, (double)f, (unsigned)COLD_START_DEFER_HOPS);
        }
    }

    /* ------------------------------------------------------------------ */
    /* Push motor commands — ALWAYS send the current position so CM4 has   */
    /* an up-to-date target.  The previous logic only sent on change,      */
    /* which meant the very first command (position 0 → 0 on open string)  */
    /* was silently dropped, and motors never received subsequent targets.  */
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
    /* Solenoid actuation — BPM-scheduled, not convergence-gated           */
    /*                                                                      */
    /* ALL strings for a note fire at exactly SONG_ACTUATE_DELAY_MS after   */
    /* the note starts.  This keeps actuation locked to the song's BPM.    */
    /*                                                                      */
    /* If PI locks early, we wait.  If PI hasn't converged by the deadline  */
    /* we fire anyway at the best position the controller reached.          */
    /* Chord strings always fire simultaneously — same deadline.            */
    /* ------------------------------------------------------------------ */
    {
        uint32_t note_elapsed = SongSched_GetElapsedMs(HAL_GetTick());
        uint8_t  time_to_fire = (note_elapsed >= SONG_ACTUATE_DELAY_MS);

        if (time_to_fire && !s_state[str].note_actuated) {

            /* Burst duration: 500ms whole notes, 250ms half notes */
            const SongNote_t *cur = SongSched_GetNoteAt(SongSched_GetNoteIndex());
            uint16_t burst_ms = (cur && cur->duration_ms <= 2000u) ? 250u : 500u;

            if (chord) {
                /* Chord: fire BOTH strings at the same scheduled instant */
                uint8_t sol_mask = 0u;
                float   freqs[6] = {0.0f};

                float f1 = s_state[str].lock_state
                         ? s_state[str].last_valid_freq_hz : target;
                float f2_freq = s_state[str2].lock_state
                              ? s_state[str2].last_valid_freq_hz : target2;

                if (f1 >= 40.0f && f1 <= 800.0f) {
                    sol_mask |= (1u << str);  freqs[str] = f1;
                }
                if (f2_freq >= 40.0f && f2_freq <= 800.0f) {
                    sol_mask |= (1u << str2); freqs[str2] = f2_freq;
                }
                if (sol_mask) {
                    InterCore_TriggerSolenoidHzBatch(sol_mask, freqs,
                                                     SOLENOID_DEFAULT_DUTY_CYCLE,
                                                     SOLENOID_MODE_TIMED, burst_ms);
                }
                s_state[str].note_actuated  = 1u;
                s_state[str2].note_actuated = 1u;
                printf("[TC] Chord actuate @%lums: %.1f/%.1f Hz burst=%ums (lock=%u/%u)\r\n",
                       (unsigned long)note_elapsed,
                       (double)f1, (double)f2_freq,
                       (unsigned)burst_ms,
                       (unsigned)s_state[str].lock_state,
                       (unsigned)s_state[str2].lock_state);

            } else {
                /* Single string */
                float f = s_state[str].lock_state
                        ? s_state[str].last_valid_freq_hz : target;
                if (f >= 40.0f && f <= 800.0f) {
                    InterCore_TriggerSolenoidHz(str, f, SOLENOID_DEFAULT_DUTY_CYCLE,
                                                SOLENOID_MODE_TIMED, burst_ms, 0u);
                }
                s_state[str].note_actuated = 1u;
                printf("[TC] Actuate str%u @%lums: %.1f Hz burst=%ums (lock=%u)\r\n",
                       (unsigned)str, (unsigned long)note_elapsed,
                       (double)f, (unsigned)burst_ms,
                       (unsigned)s_state[str].lock_state);
            }
        }
    }

    /* Track pluck age */
    s_state[str].hops_since_pluck++;
    if (chord) s_state[str2].hops_since_pluck++;

    /* ------------------------------------------------------------------ */
    /* Debug CSV logging (decimated)                                        */
    /* Format: timestamp_ms, str, target_hz, meas_hz, err_hz,              */
    /*         ff_steps, trim_steps, cmd_steps, cur_steps,                  */
    /*         lock, pi_en, delay_ctr, confidence, faults                  */
    /* ------------------------------------------------------------------ */
    s_debug_counter++;
    if (s_debug_counter >= DEBUG_DECIM_HOPS) {
        s_debug_counter = 0u;

        const TuningStringState_t *ds = &s_state[str];
        float err_hz = target - measured;

        printf("[TC] %lu,%u,%.2f,%.2f,%.3f,%.1f,%.1f,%ld,%ld,%u,%u,%u,%.0f,%u\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned int)str,
               (double)target,
               (double)measured,
               (double)err_hz,
               (double)ds->ff_steps_abs,
               (double)ds->trim_steps,
               (long)ds->cmd_steps_abs,
               (long)ds->current_abs_steps,
               (unsigned int)ds->lock_state,
               (unsigned int)ds->pi_enable,
               (unsigned int)ds->delay_hold_counter,
               (double)confidence,
               (unsigned int)ds->fault_flags);
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

    s->integral_state     = 0.0f;
    s->trim_steps         = 0.0f;
    s->lock_state         = 0u;
    s->prev_lock_state    = 0u;
    s->delay_hold_counter = (uint16_t)TUNING_DELAY_FRAMES;
    s->pi_enable          = 0u;
    s->fault_flags        = 0u;
    s->stall_counter      = 0u;
    s->note_actuated      = 0u;
    s->note_plucked       = 0u;
    s->pi_hop_count       = 0u;
    s->string_warm        = 0u;
    s->silent_hop_count   = 0u;

    /* Reset target sentinel so new-note detection triggers on song restart */
    s->last_target_freq_hz = -1.0f;
    s->last_valid_freq_hz  =  0.0f;
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

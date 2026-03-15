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
    /* Equal temperament: f(n) = f_open * 2^(n/12) for frets 0-6 */
    /* E2 frets 0-6 */ {  82.41f,  87.31f,  92.50f,  98.00f, 103.83f, 110.00f, 116.54f },
    /* A2 frets 0-6 */ { 110.00f, 116.54f, 123.47f, 130.81f, 138.59f, 146.83f, 155.56f },
    /* D3 frets 0-6 */ { 146.83f, 155.56f, 164.81f, 174.61f, 185.00f, 196.00f, 207.65f },
    /* G3 frets 0-6 */ { 196.00f, 207.65f, 220.00f, 233.08f, 246.94f, 261.63f, 277.18f },
    /* B3 frets 0-6 */ { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    /* E4 frets 0-6 */ { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    /* Placeholder: linear 0..600 microsteps across frets 0-6.
     * Run the LUT builder ('mapfrets' command) to replace with measured values. */
    /* E2 */ { 0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
    /* A2 */ { 0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
    /* D3 */ { 0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
    /* G3 */ { 0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
    /* B3 */ { 0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
    /* E4 */ { 0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
};

/* =========================================================================
 * Private state
 * ========================================================================= */

static TuningStringState_t s_state[NUM_STRINGS];

/* Decimation counter for debug printing (print every N hops) */
#define DEBUG_DECIM_HOPS    5u
static uint32_t s_debug_counter = 0u;

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
 * @brief Linear interpolation between two points.
 */
static inline float Lerpf(float x0, float y0, float x1, float y1, float x)
{
    if (x1 == x0) return y0;
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
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
    const float *fbp = tuning_ff_freq_bp[str];
    const float *sbp = tuning_ff_steps_bp[str];
    const uint8_t n  = TUNING_FF_NUM_POINTS;

    /* Below first breakpoint: clip */
    if (freq <= fbp[0]) return sbp[0];

    /* Above last breakpoint: clip */
    if (freq >= fbp[n - 1u]) return sbp[n - 1u];

    /* Find surrounding interval and interpolate */
    for (uint8_t i = 0u; i < (n - 1u); i++) {
        if (freq >= fbp[i] && freq <= fbp[i + 1u]) {
            return Lerpf(fbp[i], sbp[i], fbp[i + 1u], sbp[i + 1u], freq);
        }
    }

    /* Should not reach here, but be safe */
    return sbp[n - 1u];
}

/**
 * @brief Apply solenoid enable/disable based on lock-state edge transitions.
 *
 * On rising lock edge:  trigger solenoid for this string in infinite mode.
 * On falling lock edge: stop the solenoid.
 *
 * @param string_id   String index 0–5
 * @param target_freq Target frequency (used to identify the nearest ARR value)
 */
static void ApplyActuation(uint8_t string_id,
                           float   target_freq,
                           uint8_t lock_now,
                           uint8_t lock_prev)
{
    if (lock_now && !lock_prev) {
        /* Rising edge: start solenoid at the exact target frequency.
         * Uses CMD_TRIGGER_SOLENOID_HZ → Solenoid_SetFrequencyHz() on CM4.
         * This sets ARR = (2400000 / freq_hz) - 1 directly instead of using
         * the coarse string/fret lookup table. */
        float freq = (target_freq >= 40.0f && target_freq <= 800.0f)
                            ? target_freq : 0.0f;       
        if (freq > 1e-6) {
            InterCore_TriggerSolenoidHz(string_id,
                                        freq,
                                        SOLENOID_DEFAULT_DUTY_CYCLE,
                                        SOLENOID_MODE_INFINITE,
                                        0u,
                                        0u);
        }

    } else if (!lock_now && lock_prev) {
        /* Falling edge: stop this solenoid */
        g_shared_memory.solenoid_id = string_id;
        g_shared_memory.motor_command_flags |= CMD_STOP_SOLENOID;
        InterCore_NotifyCM4();
    }
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
    }
    s_debug_counter = 0u;
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
        s->delay_hold_counter = (uint16_t)TUNING_DELAY_FRAMES;
        s->pi_enable         = 0u;
        s->stall_counter     = 0u;
        s->fault_flags       = 0u;

        /* Update FF position immediately so motor starts moving */
        s->ff_steps_abs      = FF_Interpolate(string_id, target_freq_hz);
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
    /* 2. SNR / confidence gate                                             */
    /* ------------------------------------------------------------------ */
    uint8_t measurement_trusted = (confidence >= TUNING_SNR_THRESHOLD);

    float use_freq;
    if (measurement_trusted) {
        s->last_valid_freq_hz = measured_freq_hz;
        use_freq = measured_freq_hz;
    } else {
        /* Hold last valid estimate; do NOT update PI integral */
        use_freq = s->last_valid_freq_hz;
    }

    /* ------------------------------------------------------------------ */
    /* 3. Frequency error                                                   */
    /* ------------------------------------------------------------------ */
    float e = target_freq_hz - use_freq;

    /* ------------------------------------------------------------------ */
    /* 4. Lock-band hysteresis                                              */
    /* ------------------------------------------------------------------ */
    float abs_e = fabsf(e);

    if (s->lock_state) {
        if (abs_e >= TUNING_LOCK_BAND_OUT_HZ) {
            s->lock_state = 0u;
        }
    } else {
        if (abs_e <= TUNING_LOCK_BAND_IN_HZ) {
            s->lock_state = 1u;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 5. Discrete PI with anti-windup (back-calculation)                  */
    /* Integrator only accumulates when PI enabled AND measurement trusted  */
    /* ------------------------------------------------------------------ */
    if (s->pi_enable && measurement_trusted) {
        /* Accumulate integral */
        s->integral_state += TUNING_KI * e * TUNING_SAMPLE_TIME_S;

        /* Compute raw PI output */
        float raw_trim = TUNING_KP * e + s->integral_state;

        /* Clamp output */
        float clamped_trim = Clampf(raw_trim,
                                    -TUNING_OUT_LIMIT_STEPS,
                                     TUNING_OUT_LIMIT_STEPS);

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
 * @brief Sweeps the FFT array to find the closest valid frequency to the target.
 * @param target The expected physical frequency from the MIDI file.
 * @param out_measured Pointer to store the detected frequency.
 * @param out_confidence Pointer to store the signal magnitude (confidence).
 */
static void GetFFTRadarLock(float target, float* out_measured, float* out_confidence)
{
    float best_measured = 0.0f;
    float best_mag      = 0.0f;
    float min_error     = 9999.0f;

    /* 1. Sweep the active radar pings */
    for (uint8_t i = 0; i < active_string_count; i++) {
        float error = fabsf(active_frequencies[i] - target);
        
        if (error < min_error) {
            min_error = error;
            best_measured = active_frequencies[i];
            best_mag = active_magnitudes[i];
        }
    }

    /* 2. Validate the lock */
    if (active_string_count > 0 && min_error <= TUNE_SEARCH_WINDOW_HZ) {
        if (best_mag > NOISE_GATE_THRESHOLD) {
            *out_measured = best_measured;
            *out_confidence = best_mag;
            return; /* Successful lock */
        }
    }

    /* Target is missing, out of bounds, or too quiet */
    *out_measured = 0.0f;
    *out_confidence = 0.0f;
}

/* -------------------------------------------------------------------------
 * TuningCtrl_RunHop
 * High-level entry point.  Reads from song scheduler and FFT, calls
 * TuningCtrl_Update(), then issues inter-core commands.
 * --------------------------------------------------------------------- */
void TuningCtrl_RunHop(void)
{
    /* Get current note from song scheduler */
    if (!SongSched_IsActive()) return;

    uint8_t  str    = SongSched_GetStringId();
    float    target = SongSched_GetTargetFreq();

    /* --- Primary String FFT Lock --- */
    float measured = 0.0f;
    float confidence = 0.0f;
    GetFFTRadarLock(target, &measured, &confidence);

    /* Store previous lock state before update */
    uint8_t prev_lock = s_state[str].lock_state;

    /* Run one hop of the controller */
    TuningCtrl_Update(str, target, measured, confidence);

    uint8_t curr_lock = s_state[str].lock_state;

    /* ------------------------------------------------------------------ */
    /* Push motor command to CM4 if position changed                        */
    /* ------------------------------------------------------------------ */
    if (s_state[str].cmd_steps_abs != s_state[str].prev_cmd_steps_abs) {
        InterCore_SendMotorGoto(str, s_state[str].cmd_steps_abs);
    }

    /* ------------------------------------------------------------------ */
    /* Coordinate solenoid actuation based on lock-state transitions        */
    /* ------------------------------------------------------------------ */
    ApplyActuation(str, target, curr_lock, prev_lock);

    /* ------------------------------------------------------------------ */
    /* Second string (chord support)                                        */
    /* ------------------------------------------------------------------ */
    if (SongSched_GetStringCount() >= 2u) {
        uint8_t str2     = SongSched_GetStringId2();
        float   target2  = SongSched_GetTargetFreq2();

        /* --- Secondary String FFT Lock --- */
        float measured2 = 0.0f;
        float confidence2 = 0.0f;
        GetFFTRadarLock(target2, &measured2, &confidence2);
        
        uint8_t prev_lock2 = s_state[str2].lock_state;
        TuningCtrl_Update(str2, target2, measured2, confidence2);
        uint8_t curr_lock2 = s_state[str2].lock_state;

        if (s_state[str2].cmd_steps_abs != s_state[str2].prev_cmd_steps_abs) {
            InterCore_SendMotorGoto(str2, s_state[str2].cmd_steps_abs);
        }
        ApplyActuation(str2, target2, curr_lock2, prev_lock2);
    }

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

    /* Re-compute FF for the last known target */
    if (s->last_target_freq_hz > 0.0f) {
        s->ff_steps_abs = FF_Interpolate(string_id, s->last_target_freq_hz);
    }
}

void TuningCtrl_ResetAll(void)
{
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        TuningCtrl_ResetString(i);
    }
}

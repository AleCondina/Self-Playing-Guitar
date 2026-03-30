/**
 ******************************************************************************
 * @file    tuning_controller.c
 * @brief   Semitone-domain FF+PI guitar string tuning controller — CM7
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

#define TUNE_SEARCH_WINDOW_HZ  15.0f
#define NOTE_CHANGE_SEMI       0.02f
#define OPEN_STRING_REJECT_HZ   5.0f
#define COLD_START_DEFER_HOPS   2u

/* Forward declaration — used by StartNoteTransition before definition */
static float simple_ff_steps(uint8_t str, float target_semi);

/* =========================================================================
 * Feed-forward LUT data
 * ========================================================================= */

float tuning_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {  82.41f,  82.69f,  89.97f,  96.05f, 100.07f, 106.37f, 112.82f },
    { 110.00f, 115.54f, 121.57f, 127.57f, 139.23f, 145.69f, 151.22f },
    { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {   0.0f,  267.0f, 1080.0f, 1623.0f, 2029.0f, 2706.0f, 3385.0f },
    {   0.0f,  751.0f, 1549.0f, 2370.0f, 4103.0f, 5013.0f, 5955.0f },
    {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

static const float s_std_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {  82.41f,  82.69f,  89.97f,  96.05f, 100.07f, 106.37f, 112.82f },
    { 110.00f, 115.54f, 121.57f, 127.57f, 139.23f, 145.69f, 151.22f },
    { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

static const float s_std_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {   0.0f,  267.0f, 1080.0f, 1623.0f, 2029.0f, 2706.0f, 3385.0f },
    {   0.0f,  751.0f, 1549.0f, 2370.0f, 4103.0f, 5013.0f, 5955.0f },
    {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

static const float s_dropd_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    { D2_HZ, Ds2_HZ, E2_HZ, F2_HZ, Fs2_HZ, G2_HZ, Gs2_HZ },
    { E2_HZ, F2_HZ, Fs2_HZ, G2_HZ, Gs2_HZ, A2_HZ, As2_HZ },
    { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

static const float s_dropd_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {   0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
    {   0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f },
    {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

static const float s_dropg2_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {  82.41f,  82.69f,  89.97f,  96.05f, 100.07f, 106.37f, 112.82f },
    { G2_HZ, Gs2_HZ, A2_HZ, As2_HZ, B2_HZ, C3_HZ, Cs3_HZ },
    { 146.83f, 160.03f, 170.43f, 179.45f, 188.51f, 197.45f, 205.19f },
    { 196.00f, 207.65f, 233.38f, 248.76f, 262.49f, 277.12f, 292.19f },
    { 246.94f, 261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f },
    { 329.63f, 349.23f, 369.99f, 392.00f, 415.30f, 440.00f, 466.16f },
};

static const float s_dropg2_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS] = {
    {   0.0f,  267.0f, 1080.0f, 1623.0f, 2029.0f, 2706.0f, 3385.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f, 2331.0f, 3363.0f, 4417.0f, 5477.0f, 6533.0f, 7499.0f },
    {   0.0f,  910.0f, 2583.0f, 3485.0f, 4480.0f, 5523.0f, 6646.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
    {   0.0f,  100.0f,  200.0f,  300.0f,  400.0f,  500.0f,  600.0f },
};

/* =========================================================================
 * Semitone-domain polynomial coefficients
 * ========================================================================= */

float tuning_ff_semi_coeff[NUM_STRINGS][3] = {
    { 29.8134f, 424.929f, -70.6621f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
};

float tuning_plant_coeff[NUM_STRINGS][3] = {
    { -1.2433e-07f, 2.0937e-03f, 0.1599f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
};

/* =========================================================================
 * Private state
 * ========================================================================= */

static const float s_open_freq_hz[NUM_TUNINGS][NUM_STRINGS] = {
    [TUNING_STANDARD] = { E2_HZ, A2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ },
    [TUNING_DROP_D]   = { D2_HZ, E2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ },
    [TUNING_DROP_G2]  = { E2_HZ, G2_HZ, D3_HZ, G3_HZ, B3_HZ, E4_HZ },
};

static TuningStringState_t s_state[NUM_STRINGS];
static uint8_t s_motor_enable_mask = 0u;
static uint8_t s_song_first_note = 1u;
static uint8_t s_cold_start_pending = 0u;
static uint8_t s_active_tuning = TUNING_STANDARD;

/* =========================================================================
 * Helpers
 * ========================================================================= */

static inline float ActiveOpenFreq(uint8_t str)
{
    return s_open_freq_hz[s_active_tuning][str];
}

static inline float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float freq_to_semi(uint8_t str, float freq_hz)
{
    if (freq_hz <= 0.0f) return 0.0f;
    return 12.0f * log2f(freq_hz / ActiveOpenFreq(str));
}

static inline float semi_to_freq(uint8_t str, float semi)
{
    return ActiveOpenFreq(str) * powf(2.0f, semi / 12.0f);
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
    float new_target_semi = freq_to_semi(string_id, target_freq_hz);
    return (fabsf(new_target_semi - prev_target_semi) > NOTE_CHANGE_SEMI);
}

static uint8_t BurstClearWaitHops(uint16_t burst_ms)
{
    float fft_window_ms = ((float)FFT_SIZE * 1000.0f) / (float)AUDIO_SAMPLE_RATE_HZ;
    float hop_ms = TUNING_SAMPLE_TIME_S * 1000.0f;
    float wait_hops = ceilf((((float)burst_ms) + fft_window_ms) / hop_ms);
    wait_hops = Clampf(wait_hops, 0.0f, 255.0f);
    return (uint8_t)wait_hops;
}

static void StartNoteTransition(uint8_t string_id,
                                float target_freq_hz,
                                uint8_t burst_fired,
                                uint16_t burst_ms)
{
    TuningStringState_t *s = &s_state[string_id];
    float target_semi = freq_to_semi(string_id, target_freq_hz);

    s->integral_state = 0.0f;
    s->lock_state = 0u;
    s->prev_lock_state = 0u;
    s->stall_counter = 0u;
    s->fault_flags = 0u;
    s->hop_count = 0u;
    s->prev_target_semi = target_semi;
    s->resonance_wait_hops_remaining = 0u;
    s->fft_clear_wait_hops_remaining = 0u;

    if (burst_fired) {
        /* Cold-start: wait for FFT to clear solenoid contamination */
        s->state = TUNE_STATE_RESONANCE_WAIT;
        s->resonance_wait_hops_remaining = TUNING_RESONANCE_WAIT_HOPS;
        s->fft_clear_wait_hops_remaining = BurstClearWaitHops(burst_ms);
        /* Apply simple FF even during burst wait so motor starts moving now */
        s->ff_steps_abs = simple_ff_steps(string_id, target_semi);
    } else {
        /* No burst: go straight to PI_ACTIVE, apply simple FF immediately */
        s->state = TUNE_STATE_PI_ACTIVE;
        s->ff_steps_abs = simple_ff_steps(string_id, target_semi);
    }
}

static float feedforward_steps(uint8_t str, float target_semi)
{
    const float *c = tuning_ff_semi_coeff[str];
    float steps = c[0] * target_semi * target_semi +
                  c[1] * target_semi +
                  c[2];

    steps += (float)Cal_TuneGetOffset(str);
    return steps;
}

/* Simple linear feedforward — replaces polynomial for song playback.
 * One constant (TUNING_SIMPLE_FF_STEPS_PER_SEMI) derived from LUT average.
 * Gets motor ~90% correct; PI handles the remaining error. */
static float simple_ff_steps(uint8_t str, float target_semi)
{
    return TUNING_SIMPLE_FF_STEPS_PER_SEMI * target_semi
           + (float)Cal_TuneGetOffset(str);
}

static float plant_steps_to_semi(uint8_t str, float steps)
{
    const float *c = tuning_plant_coeff[str];
    return c[0] * steps * steps + c[1] * steps + c[2];
}

static float gain_scheduled_kp(uint8_t str, int32_t current_steps)
{
    const float *c = tuning_plant_coeff[str];
    float k_local = 2.0f * c[0] * (float)current_steps + c[1];

    if (fabsf(k_local) < 1e-6f) {
        k_local = (k_local >= 0.0f) ? 1e-6f : -1e-6f;
    }

    return TUNING_KP * TUNING_K_AVG / k_local;
}

static uint8_t IsOpenStringInterference(float freq, uint8_t string_id)
{
    for (uint8_t s = 0u; s < NUM_STRINGS; s++) {
        if (s == string_id) continue;
        if (fabsf(freq - ActiveOpenFreq(s)) <= OPEN_STRING_REJECT_HZ) {
            return 1u;
        }
    }
    return 0u;
}

static void GetFFTRadarLock(uint8_t string_id,
                            float expected,
                            float target,
                            float *out_measured,
                            float *out_confidence)
{
    float best_measured = 0.0f;
    float best_mag = 0.0f;

    for (uint8_t i = 0u; i < active_string_count; i++) {
        float freq = active_frequencies[i];
        float mag = active_magnitudes[i];

        if (IsOpenStringInterference(freq, string_id)) {
            continue;
        }

        {
            float err_exp = fabsf(freq - expected);
            float err_tgt = fabsf(freq - target);
            float min_err = (err_exp < err_tgt) ? err_exp : err_tgt;

            if (min_err > TUNE_SEARCH_WINDOW_HZ) {
                continue;
            }
        }

        if (mag > best_mag) {
            best_mag = mag;
            best_measured = freq;
        }
    }

    *out_measured = best_measured;
    *out_confidence = best_mag;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void TuningCtrl_Init(void)
{
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        TuningStringState_t *s = &s_state[i];
        memset(s, 0, sizeof(*s));

        s->state = TUNE_STATE_IDLE;
        s->prev_target_semi = -999.0f;
        s->expected_freq_hz = ActiveOpenFreq(i);
    }

    s_motor_enable_mask = 0u;
    s_song_first_note = 1u;
    s_cold_start_pending = 0u;
}

void TuningCtrl_Update(uint8_t string_id,
                       float target_freq_hz,
                       float measured_freq_hz,
                       float confidence)
{
    if (string_id >= NUM_STRINGS) return;
    (void)confidence;

    TuningStringState_t *s = &s_state[string_id];
    float target_semi = freq_to_semi(string_id, target_freq_hz);
    float pi_delta = 0.0f;
    uint8_t ff_command_issued = 0u;
    uint8_t measurement_trusted = (measured_freq_hz > 0.0f);
    uint8_t measurement_blocked = (s->fft_clear_wait_hops_remaining > 0u);
    uint8_t motor_busy = g_shared_memory.motor_is_moving[string_id];
    float meas_semi = s->measured_semi;

    s->target_semi = target_semi;

    if (measurement_trusted && !measurement_blocked) {
        meas_semi = freq_to_semi(string_id, measured_freq_hz);
        s->measured_semi = meas_semi;
        s->last_valid_freq_hz = measured_freq_hz;
        s->silent_hop_count = 0u;
        s->string_warm = 1u;
    } else if (measurement_blocked) {
        measurement_trusted = 0u;
    } else {
        s->silent_hop_count++;
        if (s->silent_hop_count >= (uint16_t)SONG_DECAY_COLD_HOPS) {
            s->string_warm = 0u;
        }
    }

    switch (s->state) {
    case TUNE_STATE_IDLE:
        break;

    case TUNE_STATE_RESONANCE_WAIT:
        if (s->resonance_wait_hops_remaining > 0u) {
            break;
        }
        /* Burst clear done — go straight to PI (FF already applied in StartNoteTransition) */
        s->state = TUNE_STATE_PI_ACTIVE;
        break;

    case TUNE_STATE_FF_MOVE:
        /* Legacy state — redirect to PI_ACTIVE */
        s->ff_steps_abs = simple_ff_steps(string_id, target_semi);
        s->state = TUNE_STATE_PI_ACTIVE;
        ff_command_issued = 1u;
        break;

    case TUNE_STATE_FF_WAIT:
        /* Legacy state — redirect to PI_ACTIVE */
        s->state = TUNE_STATE_PI_ACTIVE;
        break;

    case TUNE_STATE_PI_ACTIVE:
        if (!measurement_trusted) {
            break;
        }

        /* Recompute simple FF each hop (target may have changed) */
        s->ff_steps_abs = simple_ff_steps(string_id, target_semi);

        {
            float e = target_semi - meas_semi;
            float abs_e = fabsf(e);

            if (abs_e < TUNING_DEADBAND_SEMI) {
                s->lock_state = 1u;
                s->state = TUNE_STATE_IN_TUNE;
                break;
            }

            {
                float kp_actual = gain_scheduled_kp(string_id, s->current_abs_steps);
                float cmd_base;
                float int_max;
                float int_min;

                s->integral_state += e;
                cmd_base = s->ff_steps_abs;
                int_max = ((float)TUNING_STEPS_MAX_DEFAULT - cmd_base - kp_actual * e) /
                          TUNING_KI_Z;
                int_min = ((float)TUNING_STEPS_MIN_DEFAULT - cmd_base - kp_actual * e) /
                          TUNING_KI_Z;

                if (int_max < int_min) {
                    float t = int_max;
                    int_max = int_min;
                    int_min = t;
                }

                s->integral_state = Clampf(s->integral_state, int_min, int_max);
                pi_delta = kp_actual * e + TUNING_KI_Z * s->integral_state;
            }

            s->lock_state = 0u;
        }
        break;

    case TUNE_STATE_IN_TUNE:
        if (!measurement_trusted) {
            break;
        }
        {
            float e = target_semi - meas_semi;
            if (fabsf(e) >= TUNING_DEADBAND_SEMI) {
                s->lock_state = 0u;
                s->state = TUNE_STATE_PI_ACTIVE;
            }
        }
        break;

    default:
        s->state = TUNE_STATE_IDLE;
        break;
    }

    s->last_pi_delta = pi_delta;

    {
        int32_t last_cmd_steps = s->cmd_steps_abs;
        float raw_cmd = s->ff_steps_abs + pi_delta;

        if (s->state == TUNE_STATE_IDLE) {
            raw_cmd = (float)last_cmd_steps;
        }
        /* In RESONANCE_WAIT: let motor move to FF position (already set) */

        if (!ff_command_issued &&
            fabsf(raw_cmd - (float)last_cmd_steps) < TUNING_MIN_MOVE_STEPS) {
            raw_cmd = (float)last_cmd_steps;
        }

        {
            float max_delta = TUNING_MAX_STEP_RATE_SPS * TUNING_SAMPLE_TIME_S;
            float prev_cmd_f = (float)last_cmd_steps;
            float delta = raw_cmd - prev_cmd_f;
            float slewed_cmd;

            delta = Clampf(delta, -max_delta, max_delta);
            slewed_cmd = prev_cmd_f + delta;
            slewed_cmd = Clampf(slewed_cmd,
                                (float)TUNING_STEPS_MIN_DEFAULT,
                                (float)TUNING_STEPS_MAX_DEFAULT);

            s->prev_cmd_steps_abs = last_cmd_steps;
            s->cmd_steps_abs = (int32_t)roundf(slewed_cmd);
        }
    }

    s->current_abs_steps = g_shared_memory.current_positions[string_id];

    {
        float pred_semi = plant_steps_to_semi(string_id, (float)s->cmd_steps_abs);
        s->expected_freq_hz = semi_to_freq(string_id, pred_semi);
    }

    if (s->state == TUNE_STATE_PI_ACTIVE && measurement_trusted) {
        float freq_delta = fabsf(measured_freq_hz - s->last_valid_freq_hz);
        float abs_pi = fabsf(pi_delta);

        if (abs_pi > 200.0f && freq_delta < TUNING_STALL_MIN_FREQ_DELTA_HZ) {
            s->stall_counter++;
        } else {
            s->stall_counter = 0u;
        }

        if (s->stall_counter >= TUNING_STALL_TIMEOUT_HOPS) {
            s->fault_flags |= TUNING_FAULT_STALL;
            s->cmd_steps_abs = s->current_abs_steps;
        }
    } else {
        s->stall_counter = 0u;
        s->fault_flags &= (uint8_t)~TUNING_FAULT_STALL;
    }

    s->prev_lock_state = s->lock_state;

    if (s->state == TUNE_STATE_RESONANCE_WAIT &&
        s->resonance_wait_hops_remaining > 0u) {
        s->resonance_wait_hops_remaining--;
    }

    if (s->fft_clear_wait_hops_remaining > 0u) {
        s->fft_clear_wait_hops_remaining--;
    }

    s->hop_count++;
}

void TuningCtrl_RunHop(void)
{
    for (uint8_t m = 0u; m < 4u; m++) {
        if (g_shared_memory.motor_microstep_status[m] != TUNING_REQUIRED_MICROSTEP_MODE) {
            printf("[TC] *** HALT: Motor %u microstep mode is %u, expected %u (1/16). Tuning controller disabled. ***\r\n",
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

    if (!SongSched_IsActive()) {
        if (s_motor_enable_mask != 0u) {
            InterCore_SendMotorEnableMask(0u);
            s_motor_enable_mask = 0u;
        }
        return;
    }

    {
        uint8_t str = SongSched_GetStringId();
        float target = SongSched_GetTargetFreq();
        uint8_t needed_mask = (uint8_t)(1u << str);
        uint8_t chord = (SongSched_GetStringCount() >= 2u);
        uint8_t str2 = 0u;
        float target2 = 0.0f;
        uint8_t new_note_primary;
        uint8_t new_note_secondary = 0u;
        uint8_t can_burst_primary;
        uint8_t can_burst_secondary = 0u;
        uint8_t start_primary = 0u;
        uint8_t start_secondary = 0u;
        uint8_t burst_fired_primary = 0u;
        uint8_t burst_fired_secondary = 0u;
        uint16_t burst_ms_for_start = 0u;
        float measured = 0.0f;
        float confidence = 0.0f;

        if (chord) {
            str2 = SongSched_GetStringId2();
            target2 = SongSched_GetTargetFreq2();
            needed_mask |= (uint8_t)(1u << str2);
        }

        if (needed_mask != s_motor_enable_mask) {
            InterCore_SendMotorEnableMask(needed_mask);
            s_motor_enable_mask = needed_mask;
        }

        new_note_primary = IsNewNoteTarget(str, target);
        if (chord) {
            new_note_secondary = IsNewNoteTarget(str2, target2);
        }

        can_burst_primary = (new_note_primary && StringCanTriggerSolenoid(str, target));
        if (chord) {
            can_burst_secondary =
                (new_note_secondary && StringCanTriggerSolenoid(str2, target2));
        }

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
                    uint8_t sol_mask = 0u;
                    float freqs[6] = {0.0f};

                    if (can_burst_primary) {
                        sol_mask |= (uint8_t)(1u << str);
                        freqs[str] = target;
                        burst_fired_primary = 1u;
                    }
                    if (can_burst_secondary) {
                        sol_mask |= (uint8_t)(1u << str2);
                        freqs[str2] = target2;
                        burst_fired_secondary = 1u;
                    }
                    if (sol_mask != 0u) {
                        InterCore_TriggerSolenoidHzBatch(sol_mask,
                                                         freqs,
                                                         SOLENOID_DEFAULT_DUTY_CYCLE,
                                                         SOLENOID_MODE_TIMED,
                                                         burst_ms);
                    }
                } else if (can_burst_primary) {
                    InterCore_TriggerSolenoidHz(str,
                                                target,
                                                SOLENOID_DEFAULT_DUTY_CYCLE,
                                                SOLENOID_MODE_TIMED,
                                                burst_ms,
                                                0u);
                    burst_fired_primary = 1u;
                }

                burst_ms_for_start = burst_ms;
                start_primary = new_note_primary;
                start_secondary = new_note_secondary;

                printf("[TC] Cold-start burst: %u ms at %.1f Hz (deferred %u hops)\r\n",
                       (unsigned)burst_ms,
                       (double)target,
                       (unsigned)COLD_START_DEFER_HOPS);
            }
        }

        if (!s_cold_start_pending &&
            (new_note_primary || new_note_secondary) &&
            !start_primary &&
            !start_secondary) {
            const SongNote_t *cur = SongSched_GetNoteAt(SongSched_GetNoteIndex());
            uint16_t burst_ms = (cur && cur->burst_ms > 0u) ? cur->burst_ms : 72u;

            {
                if (chord) {
                    uint8_t sol_mask = 0u;
                    float freqs[6] = {0.0f};

                    if (can_burst_primary) {
                        sol_mask |= (uint8_t)(1u << str);
                        freqs[str] = target;
                        burst_fired_primary = 1u;
                    }
                    if (can_burst_secondary) {
                        sol_mask |= (uint8_t)(1u << str2);
                        freqs[str2] = target2;
                        burst_fired_secondary = 1u;
                    }
                    if (sol_mask != 0u) {
                        InterCore_TriggerSolenoidHzBatch(sol_mask,
                                                         freqs,
                                                         SOLENOID_DEFAULT_DUTY_CYCLE,
                                                         SOLENOID_MODE_TIMED,
                                                         burst_ms);
                        printf("[TC] Chord actuate: %.1f/%.1f Hz burst=%ums\r\n",
                               (double)target,
                               (double)target2,
                               (unsigned)burst_ms);
                    }
                } else if (can_burst_primary) {
                    InterCore_TriggerSolenoidHz(str,
                                                target,
                                                SOLENOID_DEFAULT_DUTY_CYCLE,
                                                SOLENOID_MODE_TIMED,
                                                burst_ms,
                                                0u);
                    burst_fired_primary = 1u;
                    printf("[TC] Actuate str%u: %.1f Hz burst=%ums\r\n",
                           (unsigned)str,
                           (double)target,
                           (unsigned)burst_ms);
                }
            }

            burst_ms_for_start = burst_ms;
            start_primary = new_note_primary;
            start_secondary = new_note_secondary;
        }

        if (start_primary) {
            const TuningStringState_t *prev = &s_state[str];
            uint32_t prev_idx = SongSched_GetNoteIndex();

            if (prev_idx > 0u || prev->hop_count > 0u) {
                printf("[NOTE] t=%lu idx→%lu str%u tgt=%.2f Hz state=%u hops=%lu lock=%u err=%.4f\r\n",
                       (unsigned long)HAL_GetTick(),
                       (unsigned long)prev_idx,
                       (unsigned)str,
                       (double)target,
                       (unsigned)prev->state,
                       (unsigned long)prev->hop_count,
                       (unsigned)prev->lock_state,
                       (double)(prev->target_semi - prev->measured_semi));
            }

            StartNoteTransition(str, target, burst_fired_primary, burst_ms_for_start);
            SongSched_ResetNoteTimer(HAL_GetTick());
        }

        if (chord && start_secondary) {
            StartNoteTransition(str2, target2, burst_fired_secondary, burst_ms_for_start);
        }

        GetFFTRadarLock(str,
                        s_state[str].expected_freq_hz,
                        target,
                        &measured,
                        &confidence);
        TuningCtrl_Update(str, target, measured, confidence);

        if (chord) {
            float measured2 = 0.0f;
            float confidence2 = 0.0f;

            GetFFTRadarLock(str2,
                            s_state[str2].expected_freq_hz,
                            target2,
                            &measured2,
                            &confidence2);
            TuningCtrl_Update(str2, target2, measured2, confidence2);
        }

        {
            uint8_t move_mask = (uint8_t)(1u << str);
            int32_t positions[NUM_MOTORS] = {0};
            const TuningStringState_t *ds = &s_state[str];
            float err_semi = ds->target_semi - ds->measured_semi;

            positions[str] = s_state[str].cmd_steps_abs;
            if (chord) {
                move_mask |= (uint8_t)(1u << str2);
                positions[str2] = s_state[str2].cmd_steps_abs;
            }

            InterCore_SendMotorGotoBatch(move_mask, positions);
        }
    }
}

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

void TuningCtrl_ResetString(uint8_t string_id)
{
    if (string_id >= NUM_STRINGS) return;

    s_state[string_id].state = TUNE_STATE_IDLE;
    s_state[string_id].resonance_wait_hops_remaining = 0u;
    s_state[string_id].fft_clear_wait_hops_remaining = 0u;
    s_state[string_id].integral_state = 0.0f;
    s_state[string_id].ff_steps_abs = 0.0f;
    s_state[string_id].lock_state = 0u;
    s_state[string_id].prev_lock_state = 0u;
    s_state[string_id].fault_flags = 0u;
    s_state[string_id].stall_counter = 0u;
    s_state[string_id].hop_count = 0u;
    s_state[string_id].string_warm = 0u;
    s_state[string_id].silent_hop_count = 0u;
    s_state[string_id].chord_pluck_ready = 0u;
    s_state[string_id].chord_pluck_wait_hops = 0u;
    s_state[string_id].prev_target_semi = -999.0f;
    s_state[string_id].target_semi = 0.0f;
    s_state[string_id].measured_semi = 0.0f;
    s_state[string_id].last_valid_freq_hz = 0.0f;
    s_state[string_id].expected_freq_hz = ActiveOpenFreq(string_id);
}

void TuningCtrl_ResetAll(void)
{
    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        TuningCtrl_ResetString(i);
    }
    s_motor_enable_mask = 0u;
    s_song_first_note = 1u;
    s_cold_start_pending = 0u;
}

void TuningCtrl_SetTuning(uint8_t tuning_id)
{
    const float (*src_freq)[TUNING_FF_NUM_POINTS];
    const float (*src_steps)[TUNING_FF_NUM_POINTS];

    if (tuning_id >= NUM_TUNINGS) {
        tuning_id = TUNING_STANDARD;
    }

    s_active_tuning = tuning_id;

    if (tuning_id == TUNING_DROP_D) {
        src_freq = s_dropd_ff_freq_bp;
        src_steps = s_dropd_ff_steps_bp;
    } else if (tuning_id == TUNING_DROP_G2) {
        src_freq = s_dropg2_ff_freq_bp;
        src_steps = s_dropg2_ff_steps_bp;
    } else {
        src_freq = s_std_ff_freq_bp;
        src_steps = s_std_ff_steps_bp;
    }

    memcpy(tuning_ff_freq_bp, src_freq, sizeof(tuning_ff_freq_bp));
    memcpy(tuning_ff_steps_bp, src_steps, sizeof(tuning_ff_steps_bp));
    Cal_FitFromBaseLUT();

    for (uint8_t i = 0u; i < NUM_STRINGS; i++) {
        s_state[i].expected_freq_hz = ActiveOpenFreq(i);
    }
}

uint8_t TuningCtrl_GetActiveTuning(void)
{
    return s_active_tuning;
}

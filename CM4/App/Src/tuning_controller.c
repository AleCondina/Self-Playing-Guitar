#include "tuning_controller.h"
#include <math.h>

/* ---- You must implement these hooks to match your motor + solenoid drivers ---- */

/* Move motor position by delta steps (signed). Non-blocking preferred. */
static void Motor_MoveDelta(uint8_t string_id, int32_t delta_steps);

/* Optional: start/stop excitation (solenoid) for a given string. */
static void Solenoid_Start(uint8_t string_id, uint8_t velocity);
static void Solenoid_Stop(uint8_t string_id);

/* ---------------------------------------------------------------------------- */

static StringTuner_t g_tuner[6];

void TuningController_Init(void)
{
    for (int i = 0; i < 6; i++) {
        g_tuner[i].target_hz = 0.0f;
        g_tuner[i].measured_hz = 0.0f;
        g_tuner[i].err_hz = 0.0f;
        g_tuner[i].i_err = 0.0f;

        // Start conservative; you will tune these on hardware
        g_tuner[i].kp = 0.08f;   // steps per Hz (conceptually; see below)
        g_tuner[i].ki = 0.01f;

        g_tuner[i].max_step_per_update = 8.0f;

        g_tuner[i].motor_pos = 0;
        g_tuner[i].motor_pos_min = -200000;
        g_tuner[i].motor_pos_max =  200000;

        g_tuner[i].active = 0;
        g_tuner[i].note_end_ms = 0;
        g_tuner[i].velocity = 100;
    }
}

void TuningController_StartNote(uint8_t string_id,
                                float target_hz,
                                uint16_t duration_ms,
                                uint8_t velocity,
                                uint32_t now_ms)
{
    if (string_id >= 6) return;

    StringTuner_t* t = &g_tuner[string_id];
    t->target_hz = target_hz;
    t->i_err = 0.0f;
    t->active = 1;
    t->note_end_ms = now_ms + (uint32_t)duration_ms;
    t->velocity = velocity;

    // Start excitation (you can also delay until "close enough")
    Solenoid_Start(string_id, velocity);
}

void TuningController_Update(uint32_t now_ms, const float measured_hz[6])
{
    for (uint8_t s = 0; s < 6; s++) {
        StringTuner_t* t = &g_tuner[s];
        if (!t->active)
            continue;

        // End note timing
        if (now_ms >= t->note_end_ms) {
            Solenoid_Stop(s);
            t->active = 0;
            continue;
        }

        float y = measured_hz[s];
        if (y <= 0.0f) {
            // no measurement this tick; skip control update
            continue;
        }

        t->measured_hz = y;
        float err = (t->target_hz - y);
        t->err_hz = err;

        // Simple PI controller in "steps per update"
        t->i_err += err;

        float u = t->kp * err + t->ki * t->i_err;

        // Clamp move per update (rate limiting)
        if (u >  t->max_step_per_update) u =  t->max_step_per_update;
        if (u < -t->max_step_per_update) u = -t->max_step_per_update;

        int32_t delta = (int32_t)lroundf(u);

        // Bounds / safety
        int32_t new_pos = t->motor_pos + delta;
        if (new_pos < t->motor_pos_min) delta = t->motor_pos_min - t->motor_pos;
        if (new_pos > t->motor_pos_max) delta = t->motor_pos_max - t->motor_pos;

        if (delta != 0) {
            Motor_MoveDelta(s, delta);
            t->motor_pos += delta;
        }

        // Optional: only excite when close enough
        // if (fabsf(err) < 1.0f) Solenoid_Start(s, t->velocity);
        // else Solenoid_Stop(s);
    }
}

/* ------------------- PLACEHOLDER HOOKS  ------------------- */

static void Motor_MoveDelta(uint8_t string_id, int32_t delta_steps)
{
    (void)string_id; (void)delta_steps;
    // TODO: call  motor driver, e.g. Motor_SetTargetPosition(...) or Motor_SetSpeed(...)
}

static void Solenoid_Start(uint8_t string_id, uint8_t velocity)
{
    (void)string_id; (void)velocity;
    // TODO: call solenoid driver: PWM duty/current based on velocity
}

static void Solenoid_Stop(uint8_t string_id)
{
    (void)string_id;
    // TODO: stop that solenoid PWM
}

/**
 ******************************************************************************
 * @file    tuning_controller.h
 * @brief   Semitone-domain simple-linear-FF + PI guitar string tuning controller
 *
 * Architecture
 * ------------
 * Runs entirely on CM7 at the FFT hop rate (≈ 85 ms per hop at
 * 6 kHz / 1024-point half-buffer). For each active string the controller:
 *
 *  1. Converts the target frequency to semitones relative to the string's
 *     open frequency for the active tuning mode.
 *  2. Computes a feedforward step command from a simple linear model:
 *     steps = K * target_semi  (single constant, no calibration needed).
 *  3. Runs a gain-scheduled PI controller in the semitone domain with
 *     anti-windup. On cold-start notes with solenoid burst, the controller
 *     waits for FFT clear before enabling PI. On subsequent notes (no burst),
 *     PI is active immediately.
 *
 * State machine:
 *   Cold-start:  IDLE → RESONANCE_WAIT (burst clear) → PI_ACTIVE ↔ IN_TUNE
 *   Subsequent:  IDLE → PI_ACTIVE ↔ IN_TUNE  (no burst, immediate PI)
 *   - IN_TUNE monitors for drift back above deadband (10 cents)
 ******************************************************************************
 */

#ifndef TUNING_CONTROLLER_H
#define TUNING_CONTROLLER_H

#include <stdint.h>
#include "application_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Timing
 * ========================================================================= */

#define TUNING_SAMPLE_TIME_S \
    ((float)(FFT_SIZE / 2) / (float)AUDIO_SAMPLE_RATE_HZ)

#define TUNING_RESONANCE_WAIT_HOPS  0u  /* was 1 — no wait on non-burst notes */

/* =========================================================================
 * Semitone-domain control constants
 * ========================================================================= */

#define TUNING_KP              400.0f   /* was 47.0 — 8.5x for responsive positioning */
#define TUNING_KI_Z             40.0f   /* was 4.699 — scaled with Kp */
#define TUNING_K_AVG            1.596e-03f
#define TUNING_DEADBAND_SEMI    0.05f   /* was 0.05 — 10 cents, musically acceptable */
#define TUNING_FF_LOCK_SEMI     0.50f

/* =========================================================================
 * Simple linear feedforward (replaces calibrated polynomial)
 * ========================================================================= */

#define TUNING_SIMPLE_FF_STEPS_PER_SEMI  570.0f  /* avg from LUT: ~570 steps/semitone */

/* =========================================================================
 * Mechanical limits and motor parameters
 * ========================================================================= */

#define TUNING_MIN_MOVE_STEPS        4.0f
#define TUNING_MAX_STEP_RATE_SPS     65000.0f
#define TUNING_STEPS_MAX_DEFAULT     8000
#define TUNING_STEPS_MIN_DEFAULT    -800
#define TUNING_REQUIRED_MICROSTEP_MODE  4u

/* =========================================================================
 * Stall detection
 * ========================================================================= */

#define TUNING_STALL_TIMEOUT_HOPS       20u
#define TUNING_STALL_MIN_FREQ_DELTA_HZ  0.3f

/* =========================================================================
 * Song playback parameters
 * ========================================================================= */

#define SONG_DECAY_COLD_HOPS             4u
#define SONG_CHORD_PLUCK_TIMEOUT_HOPS    2u

/* =========================================================================
 * Feed-forward LUT tables
 * ========================================================================= */

#define TUNING_FF_NUM_POINTS    7u
extern float tuning_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];
extern float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];

/* =========================================================================
 * Per-string polynomial coefficients
 * ========================================================================= */

extern float tuning_ff_semi_coeff[NUM_STRINGS][3];
extern float tuning_plant_coeff[NUM_STRINGS][3];

/* =========================================================================
 * State machine
 * ========================================================================= */

typedef enum {
    TUNE_STATE_IDLE,
    TUNE_STATE_RESONANCE_WAIT,
    TUNE_STATE_FF_MOVE,
    TUNE_STATE_FF_WAIT,
    TUNE_STATE_PI_ACTIVE,
    TUNE_STATE_IN_TUNE
} TuneState_t;

/* Fault flags */
#define TUNING_FAULT_STALL      (1u << 0)
#define TUNING_FAULT_SATURATED  (1u << 1)
#define TUNING_FAULT_MICROSTEP  (1u << 2)

typedef struct {
    TuneState_t state;
    uint8_t     resonance_wait_hops_remaining;
    uint8_t     fft_clear_wait_hops_remaining;

    float    target_semi;
    float    prev_target_semi;
    float    measured_semi;

    float    last_valid_freq_hz;
    float    expected_freq_hz;

    float    ff_steps_abs;
    float    integral_state;
    float    last_pi_delta;

    int32_t  cmd_steps_abs;
    int32_t  prev_cmd_steps_abs;
    int32_t  current_abs_steps;

    uint8_t  lock_state;
    uint8_t  prev_lock_state;

    uint8_t  fault_flags;
    uint32_t stall_counter;
    uint32_t hop_count;

    uint8_t  string_warm;
    uint16_t silent_hop_count;

    uint8_t  chord_pluck_ready;
    uint8_t  chord_pluck_wait_hops;
} TuningStringState_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

void TuningCtrl_Init(void);
void TuningCtrl_RunHop(void);
void TuningCtrl_Update(uint8_t string_id,
                       float   target_freq_hz,
                       float   measured_freq_hz,
                       float   confidence);
int32_t TuningCtrl_GetCmdSteps(uint8_t string_id);
uint8_t TuningCtrl_GetLockState(uint8_t string_id);
const TuningStringState_t *TuningCtrl_GetStringState(uint8_t string_id);
void TuningCtrl_ResetString(uint8_t string_id);
void TuningCtrl_ResetAll(void);
void TuningCtrl_SetTuning(uint8_t tuning_id);
uint8_t TuningCtrl_GetActiveTuning(void);

#ifdef __cplusplus
}
#endif

#endif /* TUNING_CONTROLLER_H */

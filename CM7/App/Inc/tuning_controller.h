/**
 ******************************************************************************
 * @file    tuning_controller.h
 * @brief   Semitone-domain FF+PI guitar string tuning controller
 *
 * Architecture
 * ------------
 * Runs entirely on CM7 at the FFT hop rate (≈ 170 ms per hop at
 * 6 kHz / 2048-point half-buffer).  For each active string the controller:
 *
 *  1. Converts the target frequency to semitones relative to the string's
 *     open frequency.
 *  2. Computes a feedforward step command from a per-string quadratic
 *     polynomial in the semitone domain (steps = a·semi² + b·semi + c).
 *  3. Runs a gain-scheduled PI controller in the semitone domain with
 *     anti-windup.  The PI only activates after the first valid FFT
 *     measurement arrives following a feedforward move.
 *  4. Fires the solenoid immediately on each note change (burst duration
 *     from song data), not on a delayed timer.
 *
 * State machine: IDLE → FF_MOVE → FF_WAIT → PI_ACTIVE ↔ IN_TUNE
 *   - FF_MOVE re-triggers if |error| > FF_LOCK threshold (0.5 semi)
 *   - IN_TUNE monitors for drift back above deadband (5 cents)
 *
 * Integration points
 * ------------------
 *  CM7/App/Src/app_cm7.c       – calls TuningCtrl_Init() once; calls
 *                                 TuningCtrl_RunHop() after every FFT block.
 *  CM7/App/Inc/inter_core_cm7.h – motor positions and solenoid triggers.
 *  CM7/App/Inc/song_scheduler.h – source of (string_id, target_freq_hz).
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

/**
 * @brief Effective hop period (seconds).
 * = half-buffer size / sample rate = 1024 / 6000 ≈ 0.1707 s.
 */
#define TUNING_SAMPLE_TIME_S \
    ((float)(FFT_SIZE / 2) / (float)AUDIO_SAMPLE_RATE_HZ)

/* =========================================================================
 * Semitone-domain control constants (from MATLAB system identification)
 * ========================================================================= */

/** Proportional gain (steps per semitone of error). */
#define TUNING_KP              47.0f

/** Integral gain (steps per semitone per sample). */
#define TUNING_KI_Z             4.699f

/** Average plant gain for gain scheduling (semi/step at midrange). */
#define TUNING_K_AVG            1.596e-03f

/** Deadband: no correction when |error| < 5 cents = 0.05 semitones. */
#define TUNING_DEADBAND_SEMI    0.05f

/** Re-trigger feedforward if |error| exceeds 0.5 semitones. */
#define TUNING_FF_LOCK_SEMI     0.50f

/* =========================================================================
 * Mechanical limits and motor parameters
 * ========================================================================= */

/** Minimum change in step command to apply (defeats backlash dither). */
#define TUNING_MIN_MOVE_STEPS   4.0f

/** Slew limit (steps/s). Per-hop max = this × TUNING_SAMPLE_TIME_S. */
#define TUNING_MAX_STEP_RATE_SPS    65000.0f

/** Mechanical step limits. */
#define TUNING_STEPS_MAX_DEFAULT    8000
#define TUNING_STEPS_MIN_DEFAULT   -800

/** Required microstep mode (1/16 = mode 4). */
#define TUNING_REQUIRED_MICROSTEP_MODE  4u

/* =========================================================================
 * Stall detection
 * ========================================================================= */

#define TUNING_STALL_TIMEOUT_HOPS       20u
#define TUNING_STALL_MIN_FREQ_DELTA_HZ  0.3f

/* =========================================================================
 * Song playback parameters
 * ========================================================================= */

/** Consecutive silent hops before string is declared cold. */
#define SONG_DECAY_COLD_HOPS             4u

/** Chord sync: max hops to wait for partner string. */
#define SONG_CHORD_PLUCK_TIMEOUT_HOPS    2u

/* =========================================================================
 * Feed-forward LUT tables (legacy — kept for calibration SRAM compat)
 * ========================================================================= */

#define TUNING_FF_NUM_POINTS    7u
extern float tuning_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];
extern float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];

/* =========================================================================
 * Per-string polynomial coefficients
 *
 * Feedforward:  steps = a·semi² + b·semi + c  (semitones → steps)
 * Plant model:  semi  = a·steps² + b·steps + c (steps → semitones)
 *
 * E2 (string 0) seeded from MATLAB identification.
 * Other strings seeded from legacy cal_poly_coeff until calibrated.
 * Updated by Cal_FitSemiPolynomial() after each calibration run.
 * ========================================================================= */

extern float tuning_ff_semi_coeff[NUM_STRINGS][3];  /**< [str][a,b,c] FF */
extern float tuning_plant_coeff[NUM_STRINGS][3];    /**< [str][a,b,c] plant */

/* =========================================================================
 * State machine
 * ========================================================================= */

typedef enum {
    TUNE_STATE_IDLE,       /**< No target, waiting for command               */
    TUNE_STATE_FF_MOVE,    /**< Feedforward move executing, integrator reset  */
    TUNE_STATE_FF_WAIT,    /**< Waiting for first valid FFT measurement       */
    TUNE_STATE_PI_ACTIVE,  /**< PI feedback running, correcting residual      */
    TUNE_STATE_IN_TUNE     /**< Within deadband, monitoring for drift         */
} TuneState_t;

/* =========================================================================
 * Per-string run-time state
 * ========================================================================= */

/** Fault flags */
#define TUNING_FAULT_STALL      (1u << 0)
#define TUNING_FAULT_SATURATED  (1u << 1)
#define TUNING_FAULT_MICROSTEP  (1u << 2)

typedef struct {
    /* State machine */
    TuneState_t state;              /**< Current control phase                */

    /* Semitone-domain tracking */
    float    target_semi;           /**< Target pitch (semitones from open)   */
    float    prev_target_semi;      /**< Target at previous hop (note change) */
    float    measured_semi;         /**< Last measured pitch (semitones)       */

    /* Hz-domain tracking (for FFT search window) */
    float    last_valid_freq_hz;    /**< Last accepted FFT frequency (Hz)     */
    float    expected_freq_hz;      /**< Predicted freq from motor position   */

    /* Controller state (microsteps) */
    float    ff_steps_abs;          /**< Feedforward absolute position        */
    float    integral_state;        /**< PI integrator accumulator (semi)     */

    /* Commanded and actual positions */
    int32_t  cmd_steps_abs;         /**< Final integer command (microsteps)   */
    int32_t  prev_cmd_steps_abs;    /**< Command at previous hop (slew limit) */
    int32_t  current_abs_steps;     /**< Motor position read from CM4         */

    /* Lock state */
    uint8_t  lock_state;            /**< 1 = in-tune (within deadband)        */
    uint8_t  prev_lock_state;       /**< Lock at previous hop (edge detect)   */

    /* Fault / diagnostics */
    uint8_t  fault_flags;           /**< Bit field: TUNING_FAULT_*            */
    uint32_t stall_counter;         /**< Consecutive hops above stall thr     */
    uint32_t hop_count;             /**< Total hops (wraps at UINT32_MAX)     */

    /* Song playback — string energy state */
    uint8_t  string_warm;           /**< 1 = string vibrating recently        */
    uint16_t silent_hop_count;      /**< Consecutive hops with no FFT signal  */

    /* Chord synchronization */
    uint8_t  chord_pluck_ready;     /**< Locked, waiting for chord partner    */
    uint8_t  chord_pluck_wait_hops; /**< Hops spent waiting for partner       */
} TuningStringState_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/** Initialize all per-string tuning states to safe defaults. */
void TuningCtrl_Init(void);

/** Run one complete control hop for all active strings. */
void TuningCtrl_RunHop(void);

/** Run one control hop for a single string (low-level). */
void TuningCtrl_Update(uint8_t string_id,
                       float   target_freq_hz,
                       float   measured_freq_hz,
                       float   confidence);

/** Get the most recently computed absolute motor step command. */
int32_t TuningCtrl_GetCmdSteps(uint8_t string_id);

/** Get the current lock state (1 = in-tune). */
uint8_t TuningCtrl_GetLockState(uint8_t string_id);

/** Get a read-only pointer to the full per-string state. */
const TuningStringState_t *TuningCtrl_GetStringState(uint8_t string_id);

/** Force-reset a string's controller state. */
void TuningCtrl_ResetString(uint8_t string_id);

/** Force-reset all strings. */
void TuningCtrl_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* TUNING_CONTROLLER_H */

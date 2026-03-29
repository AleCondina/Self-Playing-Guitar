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
 *     anti-windup.  On note changes that fire a solenoid, the controller
 *     inserts a fixed resonance wait before feedforward, then only enables
 *     PI after the burst-clear window has expired and the motor has stopped.
 *  4. Fires the solenoid immediately on each note change (burst duration
 *     from song data), then lets the string breathe for one hop before FF.
 *
 * State machine:
 *   IDLE → RESONANCE_WAIT → FF_MOVE → FF_WAIT → PI_ACTIVE ↔ IN_TUNE
 *   IDLE → FF_MOVE → FF_WAIT → PI_ACTIVE ↔ IN_TUNE   (no solenoid fired)
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

/** Fixed post-burst breathing time before feedforward starts. */
#define TUNING_RESONANCE_WAIT_HOPS  1u

/* =========================================================================
 * Semitone-domain control constants (from MATLAB system identification)
 * ========================================================================= */

/**
 * @brief PI hold hops when string is WARM (already vibrating).
 *
 * The FFT already has a live signal, so PI can start correcting after just
 * 1 hop (~170 ms).  TUNE FOR HARDWARE — increase if PI overreacts.
 */
#define SONG_WARM_DELAY_FRAMES   1u

/**
 * @brief PI hold hops when string is COLD (at rest / first note).
 *
 * A solenoid burst fires first to excite the string; PI holds for 1 hop
 * while the FFT acquires the resonance.  Reduced from 2 to match warm
 * delay — with good feedforward the motor is already near the target.
 */
#define SONG_COLD_DELAY_FRAMES   1u

/**
 * @brief Consecutive silent hops before a string is declared cold.
 *
 * After this many hops with no FFT signal the string is considered to have
 * fully decayed.  The next note change will trigger a cold-start burst.
 * ~4 hops ≈ 680 ms.  TUNE FOR HARDWARE.
 */
#define SONG_DECAY_COLD_HOPS     4u

/**
 * @brief Chord synchronization: hops to wait for both strings to lock before
 * firing.  If the slower string hasn't locked within this many hops after the
 * faster one, fire both solenoids anyway so the chord is never silent.
 * 2 hops ≈ 340 ms.  TUNE FOR HARDWARE.
 */
#define SONG_CHORD_PLUCK_TIMEOUT_HOPS   2u

/**
 * @brief Re-excite sustain: hops between sustain re-trigger attempts.
 *
 * Set to 0 (disabled): the solenoid fires once per note on PI lock and the
 * string is allowed to ring and decay naturally.  Only enable if strings
 * decay faster than the note duration and re-excitation is needed.
 */
#define SONG_SUSTAIN_REEXCITE_HOPS   0u

/* Solenoid actuate delay is now BPM-scaled at runtime.
 * Use SongSched_GetActuateDelayMs() from song_scheduler.h.
 * Base value (1200 ms at 47 BPM) defined as SONG_ACTUATE_BASE_MS
 * in song_scheduler.c.  Higher BPM → shorter PI window. */

/**
 * @brief Duration of the very first pluck of the song (ms).
 *
 * A short initial burst gives the string enough energy to sustain
 * through the first note while the motor settles.
 */
#define SONG_INITIAL_PLUCK_MS        35u

/**
 * @brief Lock-band thresholds expressed in Hz (absolute).
 *
 * Fixed Hz thresholds match the solenoid drive units and avoid the
 * overly-tight bands that cents produce at low frequencies (the FFT
 * resolution is ~2.93 Hz/bin, so sub-Hz bands are unreliable).
 *
 * 1.0 Hz enter / 2.0 Hz exit gives comfortable hysteresis across
 * all commissioned strings (E2 82 Hz – G3 296 Hz).
 */
#define TUNING_LOCK_BAND_IN_HZ     0.2f    /**< Enter lock when |error| ≤ 0.2 Hz */
#define TUNING_LOCK_BAND_OUT_HZ    0.5f    /**< Exit  lock when |error| ≥ 0.5 Hz */

/**
 * @brief Minimum change in trim_steps to apply (defeats backlash dither).
 * If |new_trim - old_trim| < this, the trim is not updated this hop.
 * Units: microsteps.  TUNE FOR HARDWARE.
 */
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
    TUNE_STATE_IDLE,            /**< No target, waiting for command            */
    TUNE_STATE_RESONANCE_WAIT,  /**< Post-burst breathing delay before FF      */
    TUNE_STATE_FF_MOVE,         /**< Feedforward move executing, integrator reset */
    TUNE_STATE_FF_WAIT,         /**< Waiting for clean FFT + motor stop        */
    TUNE_STATE_PI_ACTIVE,       /**< PI feedback running, correcting residual   */
    TUNE_STATE_IN_TUNE          /**< Within deadband, monitoring for drift      */
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
    TuneState_t state;                       /**< Current control phase          */
    uint8_t     resonance_wait_hops_remaining; /**< Hops left in post-burst wait */
    uint8_t     fft_clear_wait_hops_remaining; /**< Hops left before FFT is trusted */

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
    float    last_pi_delta;         /**< PI output from last hop (for logging)*/

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

/**
 * @brief  Get a read-only pointer to the full per-string state.
 *
 * Use for diagnostics and logging.  Do not modify through this pointer.
 * @return NULL if string_id is out of range.
 */
const TuningStringState_t *TuningCtrl_GetStringState(uint8_t string_id);

/**
 * @brief  Force-reset a string's PI integrator, lock state, and delay counter.
 *
 * Useful when an external event (emergency stop, re-home) invalidates the
 * current control state.
 */
void TuningCtrl_ResetString(uint8_t string_id);

/**
 * @brief  Force-reset all strings.
 */
void TuningCtrl_ResetAll(void);

/**
 * @brief  Enable or disable dual-track mode for double_string playback.
 *
 * When enabled, TuningCtrl_RunHop() drives BOTH str0 and str1 motors
 * every hop using targets from DblString_GetTargetFreq(), with exclusive
 * FFT peak claiming and anti-swap tracking.
 *
 * @param enable  1 = dual-track, 0 = sequential (song_scheduler)
 */
void TuningCtrl_SetDualTrackMode(uint8_t enable);

/**
 * @brief  Switch tuning mode (standard / drop D).
 *
 * Swaps the feed-forward breakpoint tables, re-fits the polynomial
 * coefficients, and updates the open-string rejection array.
 * Does NOT reset controller state — call TuningCtrl_ResetAll() separately
 * if a song restart is desired.
 *
 * @param  tuning_id  TUNING_STANDARD (0) or TUNING_DROP_D (1)
 */
void TuningCtrl_SetTuning(uint8_t tuning_id);

/**
 * @brief  Get the currently active tuning mode.
 * @return TUNING_STANDARD (0) or TUNING_DROP_D (1)
 */
uint8_t TuningCtrl_GetActiveTuning(void);

#ifdef __cplusplus
}
#endif

#endif /* TUNING_CONTROLLER_H */

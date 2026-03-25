/**
 ******************************************************************************
 * @file    tuning_controller.h
 * @brief   Closed-loop guitar string tuning controller
 *
 * Architecture
 * ------------
 * Runs entirely on CM7 at the FFT hop rate (≈ 170 ms per hop at
 * 6 kHz / 2048-point half-buffer).  For each active string the controller:
 *
 *  1. Looks up an absolute motor-step target from a per-string feed-forward
 *     LUT keyed on target frequency.
 *  2. Trims the LUT output with a discrete PI controller that only activates
 *     after a configurable number of "hold" hops following a note change.
 *  3. Applies a slew limit to the final command before pushing it to CM4
 *     via InterCore_SendMotorGoto().
 *  4. Manages solenoid enable/disable based on the lock state.
 *
 * Integration points
 * ------------------
 *  CM7/App/Src/app_cm7.c    – calls TuningCtrl_Init() once; calls
 *                              TuningCtrl_RunHop() after every Goertzel block.
 *  CM7/App/Inc/inter_core_cm7.h – used to push motor positions and trigger
 *                              solenoids.
 *  CM7/App/Inc/goertzel_processing.h – source of per-string frequency and
 *                              confidence (magnitude) estimates.
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
 * Compile-time parameters
 * =========================================================================
 * All "TUNE FOR HARDWARE" values are initial guesses.  Use the CSV debug
 * log to run identification tests and update these before commissioning.
 * ========================================================================= */

/**
 * @brief Effective hop period (seconds).
 *
 * = half-buffer size / sample rate = 1024 / 6000 ≈ 0.1707 s.
 * Used for the PI integrator accumulation and slew-limit calculation.
 */
#define TUNING_SAMPLE_TIME_S \
    ((float)(FFT_SIZE / 2) / (float)AUDIO_SAMPLE_RATE_HZ)

/**
 * @brief Hops to hold PI disabled after a note/string change (generic default).
 *
 * Song playback overrides this with SONG_WARM_DELAY_FRAMES /
 * SONG_COLD_DELAY_FRAMES depending on string temperature state.
 * 3 hops ≈ 510 ms at the default rate.
 */
#define TUNING_DELAY_FRAMES     3u

/* =========================================================================
 * Song playback mode parameters
 * =========================================================================
 * These govern the warm/cold string strategy used during song playback.
 * A "warm" string is still vibrating from the previous note; a "cold" string
 * is at rest (first note of song, or string has decayed).
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

/**
 * @brief Fixed actuation delay from note start (milliseconds).
 *
 * All strings (single or chord) fire their solenoid at exactly this
 * time after the note begins, regardless of whether PI has locked.
 * This keeps actuation locked to the song's BPM timing.
 *
 * If PI converges early, the controller waits.  If PI hasn't converged
 * by this deadline, it fires anyway at the best position reached.
 *
 * 1200 ms gives the motor/PI time to settle with the aggressive PI
 * tuning (wider step caps, integral enable at 5 Hz, cold delay = 1).
 * TUNE FOR HARDWARE.
 */
#define SONG_ACTUATE_DELAY_MS        1200u

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

/** PI trim output clamp (±microsteps).  TUNE FOR HARDWARE. */
#define TUNING_OUT_LIMIT_STEPS  6400.0f

/**
 * @brief Slew limit on cmd_steps_abs per hop (microsteps/s).
 * Per-hop delta limit = TUNING_MAX_STEP_RATE_SPS * TUNING_SAMPLE_TIME_S.
 * Must not exceed the motor's physical step rate.  TUNE FOR HARDWARE.
 */
#define TUNING_MAX_STEP_RATE_SPS    65000.0f

/**
 * @brief Proportional gain (microsteps per Hz of frequency error).
 *
 * Matched to calibration tune controller (KP=16) but scaled up because
 * the song PI runs every hop without motor-busy gating.  TUNE FOR HARDWARE.
 */
#define TUNING_KP   32.0f

/**
 * @brief Integral gain (microsteps per Hz per second).
 *
 * Discrete integral:  integral += TUNING_KI * e * TUNING_SAMPLE_TIME_S
 * Only accumulates when |error| < TUNING_INTEGRAL_ENABLE_HZ (like tune).
 */
#define TUNING_KI   12.0f

/**
 * @brief Integral enable band (Hz).  Only accumulate integral when the
 * error is already small — prevents windup from large initial errors.
 * Inspired by TUNE_INTEGRAL_ENABLE_BAND_HZ in calibration_controller.
 */
#define TUNING_INTEGRAL_ENABLE_HZ   5.0f

/**
 * @brief Number of hops with a large unresolved error before a stall is flagged.
 *
 * Stall = large commanded displacement but frequency barely moves.
 * TUNE FOR HARDWARE.
 */
#define TUNING_STALL_TIMEOUT_HOPS   20u

/**
 * @brief Frequency change required to declare a stall during a move.
 * If the motor commanded more than TUNING_OUT_LIMIT_STEPS but frequency
 * moved less than this, the motor may be stalled.  TUNE FOR HARDWARE (Hz).
 */
#define TUNING_STALL_MIN_FREQ_DELTA_HZ  0.3f

/**
 * @brief Required microstep mode for safe operation.
 *
 * All controller limits and gains are calibrated for 1/16 microstepping.
 * Running with a different mode will produce incorrect position commands
 * that can damage the mechanical system.  The controller checks this at
 * every hop and halts with a warning if any configured motor is not in
 * this mode.  Value matches MotorMicrostepMode_t on CM4.
 */
#define TUNING_REQUIRED_MICROSTEP_MODE  4u  /* 1/16 step */

/** Number of LUT breakpoints per string in the feed-forward tables. */
#define TUNING_FF_NUM_POINTS    7u

/**
 * @brief Mechanical step limits per string.
 *
 * Home (position 0) = open string.  Positive steps tighten = higher pitch.
 * Negative steps allowed as a small margin below home.
 * TODO: measure per-string physical travel and replace these defaults.
 */
#define TUNING_STEPS_MAX_DEFAULT    8000
#define TUNING_STEPS_MIN_DEFAULT   -800

/* =========================================================================
 * Feed-forward LUT tables
 * Defined in tuning_controller.c.  Columns: frequency (Hz), steps (integer).
 * TODO: calibrate per-string by measuring actual frequency vs. motor steps.
 * ========================================================================= */

extern float tuning_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];
extern float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];

/* =========================================================================
 * Per-string run-time state
 * ========================================================================= */

/** Fault flags stored in TuningStringState_t.fault_flags */
#define TUNING_FAULT_STALL      (1u << 0)  /**< Motor stall suspected      */
#define TUNING_FAULT_SATURATED  (1u << 1)  /**< PI integral saturated      */
#define TUNING_FAULT_MICROSTEP  (1u << 2)  /**< Wrong microstep mode — HALT */

/**
 * @brief Complete run-time state for one guitar string.
 *
 * All float positions are in microsteps referenced to motor position 0
 * (open-string / home position).
 */
typedef struct {
    /* Frequency tracking */
    float    last_target_freq_hz;   /**< Target freq at previous hop     */
    float    last_valid_freq_hz;    /**< Last freq accepted above SNR thr */

    /* Controller positions (microsteps, float for sub-step accumulation) */
    float    ff_steps_abs;          /**< Feed-forward absolute steps      */
    float    trim_steps;            /**< PI output trim                   */
    float    integral_state;        /**< PI integrator accumulator        */

    /* Commanded and actual positions */
    int32_t  cmd_steps_abs;         /**< Final integer command (microsteps) */
    int32_t  prev_cmd_steps_abs;    /**< Command at previous hop          */
    int32_t  current_abs_steps;     /**< Motor position read from CM4     */

    /* Lock state */
    uint8_t  lock_state;            /**< 1 = in-tune and stable           */
    uint8_t  prev_lock_state;       /**< Lock at previous hop (edge detect) */

    /* Control enables */
    uint8_t  pi_enable;             /**< 1 = PI accumulating this hop     */
    uint16_t delay_hold_counter;    /**< Hops remaining in post-note hold */

    /* Fault / diagnostics */
    uint8_t  fault_flags;           /**< Bit field: TUNING_FAULT_*        */
    uint32_t stall_counter;         /**< Consecutive hops above stall thr */
    uint32_t hop_count;             /**< Total hops (wraps at UINT32_MAX) */

    /* Song playback — warm/cold string state */
    uint8_t  string_warm;           /**< 1 = string vibrating recently, 0 = at rest */
    uint16_t silent_hop_count;      /**< Consecutive hops with no FFT signal         */
    uint8_t  note_plucked;          /**< 1 = solenoid fired at least once this note  */
    uint16_t hops_since_pluck;      /**< Hops elapsed since last solenoid fire       */

    /* Chord synchronization — holds solenoid until partner string also locks */
    uint8_t  chord_pluck_ready;     /**< Locked, waiting for chord partner to lock   */
    uint8_t  chord_pluck_wait_hops; /**< Hops spent waiting for chord partner        */

    /* Per-note PI convergence tracking */
    uint16_t pi_hop_count;          /**< PI hops elapsed this note (for max-hop limit) */
    uint8_t  note_actuated;         /**< 1 = solenoid fired after PI converge this note */

    /* String frequency tracking (from stepper position) */
    float    expected_freq_hz;      /**< Predicted freq from current motor position    */
} TuningStringState_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialize all per-string tuning states to safe defaults.
 *
 * Must be called once, after Motor_Init() and Goertzel_Init() have completed
 * successfully and before the first call to TuningCtrl_RunHop().
 */
void TuningCtrl_Init(void);

/**
 * @brief  Run one complete control hop for all active strings.
 *
 * Intended to be called from app_cm7.c every time Audio_HasNewSamples()
 * returns true (after both FFT_ProcessBlock and Goertzel_ProcessBlock have
 * been called on the same buffer).
 *
 * Internally calls TuningCtrl_Update() for the currently active string
 * obtained from the song scheduler.  Also issues InterCore_SendMotorGoto()
 * and solenoid commands as needed.
 */
void TuningCtrl_RunHop(void);

/**
 * @brief  Run one control hop for a single string (low-level; prefer RunHop).
 *
 * @param  string_id        String index 0–5 (E2 = 0, …, E4 = 5)
 * @param  target_freq_hz   Desired pitch in Hz from the song scheduler
 * @param  measured_freq_hz Per-string frequency from FFT active_frequencies[]
 * @param  confidence       Target - FFT magnitude value for this string
 */
void TuningCtrl_Update(uint8_t string_id,
                       float   target_freq_hz,
                       float   measured_freq_hz,
                       float   confidence);

/**
 * @brief  Get the most recently computed absolute motor step command.
 * @param  string_id  String index 0–5
 * @return Commanded position in microsteps (0 = open string / home).
 *         Returns 0 if string_id is out of range.
 */
int32_t TuningCtrl_GetCmdSteps(uint8_t string_id);

/**
 * @brief  Get the current lock state for a string.
 * @return 1 if locked (in-tune and stable), 0 otherwise.
 */
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

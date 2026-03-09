/**
 ******************************************************************************
 * @file    tuning_controller.h
 * @brief   Closed-loop guitar string tuning controller
 *
 * Architecture
 * ------------
 * Runs entirely on CM7 at the Goertzel/FFT hop rate (≈ 170 ms per hop at
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
 * @brief Hops to hold PI disabled after a note/string change.
 *
 * Provides time for the Goertzel estimator to lock onto the new pitch
 * after the motor has moved.  3 hops ≈ 510 ms at the default rate.
 * Increase if measurement latency is longer.  TUNE FOR HARDWARE.
 */
#define TUNING_DELAY_FRAMES     3u

/** Enter lock when |freq_error| ≤ this value (Hz).  TUNE FOR HARDWARE. */
#define TUNING_LOCK_BAND_IN_HZ  1.0f

/**
 * @brief Exit lock when |freq_error| ≥ this value (Hz).
 * Must be > TUNING_LOCK_BAND_IN_HZ to provide hysteresis.
 * TUNE FOR HARDWARE.
 */
#define TUNING_LOCK_BAND_OUT_HZ 2.5f

/**
 * @brief Minimum change in trim_steps to apply (defeats backlash dither).
 * If |new_trim - old_trim| < this, the trim is not updated this hop.
 * Units: microsteps.  TUNE FOR HARDWARE.
 */
#define TUNING_MIN_MOVE_STEPS   2.0f

/** PI trim output clamp (±microsteps).  TUNE FOR HARDWARE. */
#define TUNING_OUT_LIMIT_STEPS  400.0f

/**
 * @brief Slew limit on cmd_steps_abs per hop (microsteps/s).
 * Per-hop delta limit = TUNING_MAX_STEP_RATE_SPS * TUNING_SAMPLE_TIME_S.
 * Must not exceed the motor's physical step rate.  TUNE FOR HARDWARE.
 */
#define TUNING_MAX_STEP_RATE_SPS    5000.0f

/**
 * @brief Proportional gain (microsteps per Hz of frequency error).
 *
 * Rough estimate: if the relationship is ~10 steps/Hz, start with 3-5 and
 * increase until oscillation, then halve.  TUNE FOR HARDWARE.
 */
#define TUNING_KP   3.0f

/**
 * @brief Integral gain (microsteps per Hz per second).
 *
 * Discrete integral:  integral += TUNING_KI * e * TUNING_SAMPLE_TIME_S
 * Start at 1/3 of Kp / Ti where Ti ≈ 2-5 s.  TUNE FOR HARDWARE.
 */
#define TUNING_KI   1.5f

/**
 * @brief Goertzel magnitude threshold for measurement trust.
 *
 * Below this value the PI integrator is frozen and the last valid frequency
 * estimate is held.  Units match Goertzel_GetMagnitude() output.
 * TUNE FOR HARDWARE (run with no string, observe magnitude noise floor).
 */
#define TUNING_SNR_THRESHOLD    500.0f

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

/** Number of LUT breakpoints per string in the feed-forward tables. */
#define TUNING_FF_NUM_POINTS    4u

/**
 * @brief Mechanical step limits per string.
 *
 * Home (position 0) = open string.  Positive steps tighten = higher pitch.
 * Negative steps allowed as a small margin below home.
 * TODO: measure per-string physical travel and replace these defaults.
 */
#define TUNING_STEPS_MAX_DEFAULT    2000
#define TUNING_STEPS_MIN_DEFAULT   -50

/* =========================================================================
 * Feed-forward LUT tables
 * Defined in tuning_controller.c.  Columns: frequency (Hz), steps (integer).
 * TODO: calibrate per-string by measuring actual frequency vs. motor steps.
 * ========================================================================= */

extern const float tuning_ff_freq_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];
extern const float tuning_ff_steps_bp[NUM_STRINGS][TUNING_FF_NUM_POINTS];

/* =========================================================================
 * Per-string run-time state
 * ========================================================================= */

/** Fault flags stored in TuningStringState_t.fault_flags */
#define TUNING_FAULT_STALL      (1u << 0)  /**< Motor stall suspected   */
#define TUNING_FAULT_SATURATED  (1u << 1)  /**< PI integral saturated   */

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
 * @param  measured_freq_hz Per-string frequency from Goertzel_GetExactFreq()
 * @param  confidence       Goertzel_GetMagnitude() value for this string
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

#ifdef __cplusplus
}
#endif

#endif /* TUNING_CONTROLLER_H */

/**
 ******************************************************************************
 * @file    calibration_controller.h
 * @brief   Guitar string fret calibration with PI lock-in and SRAM persistence
 *
 * Calibration flow (per string, E2 → E4)
 * ----------------------------------------
 *  1. Print a prompt — user manually plucks the string so the system can
 *     detect the current open-string frequency and confirm the string is live.
 *  2. Motor moves to position 0 (open/unfretted) and settles.
 *  3. For each half-semitone target 1 … CAL_NUM_TARGETS (plus −1 fret):
 *     a. Motor advances toward the fret at CAL_MOTOR_SPEED_FAST, then
 *        decelerates in the final CAL_SLOW_ZONE steps at CAL_MOTOR_SPEED_SLOW
 *        for fine positioning.
 *     b. Solenoid fires for CAL_SOL_DURATION_MS at the fret's resonant
 *        frequency (f_open × 2^(fret/12)).  FFT sensing is inhibited
 *        (fft_sensing_enabled = 0) during the burst to block crosstalk.
 *     c. After CAL_BURST_WAIT_HOPS the FFT window is fully post-burst and
 *        sensing is re-enabled.
 *     d. An inline PI controller drives the motor to the exact frequency.
 *        Once locked (|error| ≤ CAL_LOCK_BAND_HZ for
 *        CAL_LOCK_CONSEC_HOPS consecutive hops), the actual motor position
 *        from shared memory is recorded.
 *     e. If the string decays before lock, the solenoid re-bursts (up to
 *        CAL_MAX_RETRIES times, with doubled burst energy on retry ≥ 2).
 *  4. A 7-point LUT and a quadratic least-squares polynomial
 *       steps = a + b·f + c·f²
 *     are derived from the 8 measured (steps, freq) pairs.
 *  5. Both LUT and polynomial are saved to backup SRAM (VBAT-backed,
 *     0x38800000) with a magic-number guard so they survive power cycles.
 *  6. After all frets are measured, that string's motor is returned to
 *     position 0 before calibration advances to the next string.
 *
 * Usage
 * -----
 *  Boot time: call Cal_LoadFromSRAM() to restore a previous calibration.
 *  Calibration: call Cal_Start(mask), then route every audio hop through
 *               Cal_Hop() INSTEAD of TuningCtrl_RunHop().
 *               Poll Cal_GetStatus() for completion.
 *
 * String mask: bit N = 1 → calibrate string N (0=E2, 1=A2, … 5=E4).
 * Only strings 1–3 have live solenoid hardware; string 0 relies on the
 * initial manual pluck (solenoid re-excitation is a safe no-op for it).
 *
 * UART output format
 *   [CAL]      str, step, freq_hz         – raw measurement
 *   [CAL-LUT]  str, bp_idx, freq_hz, steps – LUT breakpoint
 *   [CAL-POLY] str: a=… b=… c=…           – polynomial coefficients
 ******************************************************************************
 */

#ifndef CALIBRATION_CONTROLLER_H
#define CALIBRATION_CONTROLLER_H

#include <stdint.h>
#include "application_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Quadratic feed-forward polynomial  (steps = a + b·f + c·f²)
 *
 * Set by Cal_FitPolynomial() at the end of each string's calibration.
 * Accessible by the tuning controller for smoother feedforward between
 * LUT breakpoints.
 * ========================================================================= */
extern float cal_poly_coeff[NUM_STRINGS][3];   /**< [str][0=a, 1=b, 2=c] */

/* =========================================================================
 * Status
 * ========================================================================= */

/**
 * @brief Calibration process status.
 */
typedef enum {
    CAL_STATUS_IDLE    = 0, /**< Not started                        */
    CAL_STATUS_RUNNING = 1, /**< Calibration in progress            */
    CAL_STATUS_DONE    = 2, /**< All strings calibrated and saved   */
    CAL_STATUS_ERROR   = 3  /**< Unrecoverable error                */
} CalStatus_t;

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Seed polynomial coefficients from the hardcoded base LUT data.
 *
 * Call when Cal_LoadFromSRAM() returns 0 so that the tuning controller has a
 * valid feedforward polynomial from first boot, before any calibration run.
 * Fits steps = a + b·f + c·f² for all strings using tuning_ff_freq_bp and
 * tuning_ff_steps_bp as the input points.
 */
void Cal_FitFromBaseLUT(void);

/**
 * @brief Attempt to restore a previous calibration from backup SRAM.
 *
 * Call once at boot, before TuningCtrl_Init().  If a valid calibration is
 * found (magic number + checksum pass), tuning_ff_freq_bp,
 * tuning_ff_steps_bp, and cal_poly_coeff are loaded from SRAM and the
 * function returns 1.  Otherwise returns 0 (tables unchanged).
 */

uint8_t Cal_LoadFromFlash(void);


/**
 * @brief Start calibration for a bitmask of strings.
 *
 * @param string_mask  Bit N = 1 → calibrate string N (0=E2 … 5=E4).
 *                     Strings absent from the mask are skipped and their
 *                     existing LUT entries are preserved.
 * Motors for the selected strings must be enabled and at position 0.
 */
void Cal_Start(uint8_t string_mask);

/**
 * @brief Start calibration for a single string.
 * @param str_id  String index 0–5.
 */
void Cal_StartStr(uint8_t str_id);

/**
 * @brief Start calibration for all six strings (E2 → E4).
 */
void Cal_StartAll(void);

/**
 * @brief Advance the calibration state machine by one audio hop.
 *
 * Call every time Audio_HasNewSamples() fires, AFTER FFT_ProcessBlock()
 * has been called on the same buffer.
 * Do NOT call TuningCtrl_RunHop() while calibration is running.
 */
void Cal_Hop(void);

/**
 * @brief Return current calibration status.
 */
CalStatus_t Cal_GetStatus(void);

/* =========================================================================
 * Tune API — open-string pitch correction (bidirectional motor adjustment)
 *
 * Usage:
 *   Cal_TuneStart(mask) → route each audio hop through Cal_TuneHop()
 *   Poll Cal_TuneGetStatus() for completion.
 *   After tuning, Cal_TuneGetOffset(str) returns the motor offset (steps)
 *   that the tuning controller should add to its feedforward output.
 *
 * The motor moves forward (positive steps) if the string is flat, or
 * backward (negative steps) if the string is sharp, to bring the open
 * string to standard tuning. The commissioned tune workflow currently
 * supports strings 0-3; string 0 may still require manual re-plucks
 * because it has no dedicated re-excite solenoid.
 * ========================================================================= */

/**
 * @brief Start tuning for a bitmask of strings.
 * @param string_mask  Bit N = 1 → tune string N (currently 0=E2 … 3=G3).
 */
void Cal_TuneStart(uint8_t string_mask);

/**
 * @brief Advance the tune state machine by one audio hop.
 *
 * Call every time Audio_HasNewSamples() fires, AFTER FFT_ProcessBlock().
 * Do NOT call TuningCtrl_RunHop() while tuning is running.
 */
void Cal_TuneHop(void);

/**
 * @brief Return current tune status.
 */
CalStatus_t Cal_TuneGetStatus(void);

/**
 * @brief Return the tune offset (steps) for the given string.
 *
 * Add this to the feedforward output in FF_Interpolate() so that all
 * subsequent motor commands compensate for the string being flat or sharp.
 * Returns 0 if no tuning has been performed for this string.
 */
int32_t Cal_TuneGetOffset(uint8_t str);

/**
 * @brief Clear all tune offsets in RAM.
 */
void Cal_TuneClearOffsets(void);

/**
 * @brief Initialize tune offsets for the current boot.
 * @return Always 0; backup-SRAM persistence is disabled for tune offsets.
 */
uint8_t Cal_TuneLoadFromSRAM(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_CONTROLLER_H */

/**
 ******************************************************************************
 * @file    calibration_controller.h
 * @brief   Guitar string fret calibration with LUT feed-forward lock-in and SRAM persistence
 *
 * Calibration flow (per commissioned string)
 * ------------------------------------------
 *  1. Selected string is driven back to the current logical open/home area.
 *  2. The open string is locked with the same tune-style FFT/PI flow, then
 *     the motor position is zeroed at that locked point.
 *  3. For each remaining fret breakpoint:
 *     a. The currently loaded tuning LUT (from SRAM or the hardcoded defaults)
 *        provides the first motor-step estimate.
 *     b. The motor moves to that estimate and settles.
 *     c. The string is re-excited at the tracked FFT frequency when available,
 *        otherwise at the target fret frequency. Strings without a live
 *        solenoid fall back to manual plucks.
 *     d. A tune-style PI loop runs with no timeout until the fret is
 *        confirmed, then the actual motor position and locked FFT frequency
 *        are recorded.
 *  4. The measured fret points are written directly back into
 *     `tuning_ff_freq_bp` / `tuning_ff_steps_bp`.
 *  5. A quadratic least-squares polynomial `steps = a + b·f + c·f²` is
 *     fit from the measured points.
 *  6. LUT and polynomial are saved to backup SRAM (VBAT-backed, 0x38800000)
 *     so they survive power cycles.
 *  7. The motor returns to position 0 before calibration advances.
 *
 * Usage
 * -----
 *  Boot time: call Cal_LoadFromSRAM() to restore a previous calibration.
 *  Calibration: call Cal_Start(mask), then route every audio hop through
 *               Cal_Hop() INSTEAD of TuningCtrl_RunHop().
 *               Poll Cal_GetStatus() for completion.
 *
 * String mask: bit N = 1 → calibrate string N (0=E2, 1=A2, … 5=E4).
 * Strings 0–2 have commissioned re-excite solenoids. String 3 can still be
 * calibrated, but requires manual plucks when the string decays. Strings 4–5
 * are currently filtered out because their stepper hardware is not wired.
 *
 * UART output format
 *   [CAL]      str, fret, step, meas_hz, target_hz  – confirmed fret point
 *   [CAL-LUT]  str, bp_idx, freq_hz, steps          – direct LUT breakpoint
 *   [CAL-POLY] str: a=… b=… c=…                     – polynomial coefficients
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
uint8_t Cal_LoadFromSRAM(void);

/**
 * @brief Start calibration for a bitmask of strings.
 *
 * @param string_mask  Bit N = 1 → calibrate string N (0=E2 … 5=E4).
 *                     Strings absent from the mask are skipped and their
 *                     existing LUT entries are preserved.
 * The calibration controller homes each selected string back to position 0
 * before measuring fret breakpoints.
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

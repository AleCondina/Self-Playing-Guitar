/**
 ******************************************************************************
 * @file    calibration_controller.h
 * @brief   Guitar string feed-forward calibration
 *
 * Sweeps each string (E2 → E4) from open to fret 7 by advancing the motor
 * in small increments, measuring the actual pitch via FFT after each move,
 * and writing the resulting (frequency, step) pairs into the tuning
 * controller's feed-forward lookup table.
 *
 * Usage
 * -----
 *  1. Call Cal_Start() once.  All motors must already be at position 0.
 *  2. Every audio hop (after FFT_ProcessBlock has run), call Cal_Hop()
 *     INSTEAD of TuningCtrl_RunHop().
 *  3. Poll Cal_GetStatus().  When it returns CAL_STATUS_DONE (or ERROR),
 *     resume normal play by switching back to TuningCtrl_RunHop().
 *
 * The calibration prints CSV progress lines to UART:
 *   [CAL]     str, step, freq_hz          – one line per measurement
 *   [CAL-LUT] str, bp_idx, freq_hz, steps – one line per breakpoint written
 ******************************************************************************
 */

#ifndef CALIBRATION_CONTROLLER_H
#define CALIBRATION_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calibration process status.
 */
typedef enum {
    CAL_STATUS_IDLE    = 0, /**< Not started                          */
    CAL_STATUS_RUNNING = 1, /**< Sweep in progress                    */
    CAL_STATUS_DONE    = 2, /**< All strings calibrated successfully   */
    CAL_STATUS_ERROR   = 3  /**< No signal or other failure           */
} CalStatus_t;

/**
 * @brief Start the calibration sequence from string 0 (E2).
 *
 * All motors must be at position 0 before calling this.
 * Resets all internal state and begins the E2 sweep immediately on the
 * first Cal_Hop() call.
 */
void Cal_Start(void);

/**
 * @brief Advance the calibration state machine by one audio hop.
 *
 * Must be called every time Audio_HasNewSamples() fires, AFTER
 * FFT_ProcessBlock() has been called on the same buffer.
 * Do NOT call TuningCtrl_RunHop() while calibration is running.
 */
void Cal_Hop(void);

/**
 * @brief Return current calibration status.
 */
CalStatus_t Cal_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_CONTROLLER_H */

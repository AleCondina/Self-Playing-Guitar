/**
 ******************************************************************************
 * @file    lut_builder.h
 * @brief   Fret-by-fret feed-forward LUT measurement tool
 *
 * Standalone commissioning tool that measures the actual motor step positions
 * for frets 0-6 on each guitar string.  The user manually plucks the string
 * to keep it ringing while the motor sweeps forward only, using FFT peak
 * detection to identify when each fret's target frequency is reached.
 *
 * Results are written into the tuning controller's feed-forward LUT arrays
 * and printed to UART in a copy-paste-ready format for hardcoding.
 *
 * Usage
 * -----
 *  1. Call LutBuilder_Start(mask) or LutBuilder_StartAll() once.
 *  2. Every audio hop (after FFT_ProcessBlock), call LutBuilder_Hop().
 *  3. Poll LutBuilder_GetStatus().  When DONE or ERROR, resume normal play.
 *  4. Copy the [LUT-RESULT] lines from UART into tuning_controller.c.
 *
 * String mask bit layout: bit N = string N (0=E2, 1=A2, 2=D3, 3=G3, 4=B3, 5=E4).
 ******************************************************************************
 */

#ifndef LUT_BUILDER_H
#define LUT_BUILDER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LUT_STATUS_IDLE    = 0,
    LUT_STATUS_RUNNING = 1,
    LUT_STATUS_DONE    = 2,
    LUT_STATUS_ERROR   = 3
} LutBuilderStatus_t;

/**
 * @brief Start the LUT builder for a bitmask of strings.
 * @param string_mask  Bit N = 1 to measure string N.
 */
void LutBuilder_Start(uint8_t string_mask);

/**
 * @brief Start the LUT builder for all six strings.
 */
void LutBuilder_StartAll(void);

/**
 * @brief Advance the LUT builder state machine by one audio hop.
 *
 * Must be called every time Audio_HasNewSamples() fires, AFTER
 * FFT_ProcessBlock() has run on the same buffer.
 */
void LutBuilder_Hop(void);

/**
 * @brief Return current LUT builder status.
 */
LutBuilderStatus_t LutBuilder_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* LUT_BUILDER_H */

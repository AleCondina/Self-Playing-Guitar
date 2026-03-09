#ifndef FFT_PROCESSING_H
#define FFT_PROCESSING_H

#include <stdint.h>
#include "arm_math.h"
#include "application_config.h"

extern float active_frequencies[];
extern uint8_t active_string_count;

void FFT_Init(void);

/**
 * @brief Process one half-buffer of ADC samples
 * @param adc_buffer Pointer to uint16 ADC samples
 */
void FFT_ProcessBlock(uint16_t *adc_buffer);

/**
 * @brief Get detected fundamental frequency (Hz)
 */
float FFT_GetFundamental(void);

/**
 * @brief Get harmonic amplitudes
 * @param out Array of size MAX_HARMONICS
 */
void FFT_GetHarmonics(float *out);

#endif

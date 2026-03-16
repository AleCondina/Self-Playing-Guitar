#ifndef FFT_PROCESSING_H
#define FFT_PROCESSING_H

#include <stdint.h>
#include "arm_math.h"
#include "application_config.h"

/* Adaptive noise floor parameters (used by FFT_ProcessBlock) */
#define NOISE_FLOOR_ALPHA       0.05f   /* EMA smoothing: higher = faster adaptation (0.01–0.1) */
#define NOISE_FLOOR_SNR         20.0f    /* Peak must be this many times above the local noise floor */
#define NOISE_FLOOR_MIN         5000.0f /* Absolute minimum noise floor (ambient peaks ~10k, this gives 2x margin) */

/* Legacy absolute threshold for downstream controllers (tuning_controller, lut_builder).
 * Safe to keep low — the adaptive noise floor in FFT_ProcessBlock already filters noise,
 * so active_magnitudes[] entries that reach these controllers are real signals. */
#define NOISE_GATE_THRESHOLD    50000.0f

extern float active_frequencies[];
extern float active_magnitudes[];
extern uint8_t active_string_count;
extern uint8_t fft_print_enabled;

void FFT_Init(void);

/**
 * @brief Process one half-buffer of ADC samples
 * @param adc_buffer Pointer to uint16 ADC samples
 */
void FFT_ProcessBlock(uint16_t *adc_buffer);

#endif

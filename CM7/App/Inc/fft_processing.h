#ifndef FFT_PROCESSING_H
#define FFT_PROCESSING_H

#include <stdint.h>
#include "arm_math.h"
#include "application_config.h"

/* Adaptive noise floor parameters (used by FFT_ProcessBlock) */
#define NOISE_FLOOR_ALPHA       0.05f    /* EMA smoothing: higher = faster adaptation (0.01–0.1) */
#define NOISE_FLOOR_SNR         8.0f     /* Peak must be this many × above local noise floor
                                          * (was 20 — too aggressive for higher strings whose
                                          *  magnitudes are inherently weaker than low E2)    */
#define NOISE_FLOOR_MIN         5000.0f  /* Absolute minimum noise floor (ambient peaks ~10k, this gives 2x margin) */
#define NOISE_FLOOR_MAX         20000.0f /* Cap: prevents ratcheting during sustained playing */
#define NOISE_FLOOR_GUARD       6.0f     /* Only update noise floor when bin < guard × current floor */
#define NOISE_FLOOR_DECAY       0.997f   /* Per-frame decay toward MIN (~2 s half-life at 5.86 Hz hop) */


extern float active_frequencies[];
extern float active_magnitudes[];
extern uint8_t active_string_count;
extern uint8_t fft_print_enabled;

/* Debug snapshots — updated every FFT frame, printed by app_cm7.c */
extern float fft_dbg_mag[3];   /* mag at bin28(E2), bin67(G3), bin112(E4) */
extern float fft_dbg_nf[3];    /* noise floor at same bins */
extern float fft_dbg_peak;     /* peak magnitude in search range */

void FFT_Init(void);

/**
 * @brief Process one half-buffer of ADC samples
 * @param adc_buffer Pointer to uint16 ADC samples
 */
void FFT_ProcessBlock(uint16_t *adc_buffer);

#endif

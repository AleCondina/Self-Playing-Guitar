#ifndef GOERTZEL_PROCESSING_H
#define GOERTZEL_PROCESSING_H

#include <stdint.h>

#define NUM_STRINGS 6

/* Structure to hold the DSP state for a single target frequency */
typedef struct {
    float target_freq;   /* The ideal open string frequency (Hz) */
    float coeff;         /* Pre-calculated Goertzel coefficient */
    float cos_w;         /* Pre-calculated cosine term */
    float sin_w;         /* Pre-calculated sine term */
    
    float prev_phase;    /* Phase angle from the previous DMA block */
    float current_freq;  /* The exact calculated physical frequency */
    float magnitude;     /* Signal strength (useful for noise gating) */
} GoertzelState_t;

/* Public API */
void Goertzel_Init(void);
void Goertzel_ProcessBlock(uint16_t *adc_buffer);
float Goertzel_GetExactFreq(uint8_t string_index);
float Goertzel_GetMagnitude(uint8_t string_index);

#endif /* GOERTZEL_PROCESSING_H */
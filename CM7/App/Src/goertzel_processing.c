#include "goertzel_processing.h"
#include "application_config.h" /* For AUDIO_SAMPLE_RATE_HZ and AUDIO_BUFFER_SIZE */
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif


//TODO: add more notes
/* Standard guitar standard tuning (E2, A2, D3, G3, B3, E4) */
static const float open_strings[NUM_STRINGS] = {
    82.41f, 110.00f, 146.83f, 196.00f, 246.94f, 329.63f
};

static GoertzelState_t strings[NUM_STRINGS];
static uint32_t block_size;
static float dt;

void Goertzel_Init(void) {
    block_size = AUDIO_BUFFER_SIZE / 2; /* Half-buffer size */
    dt = (float)block_size / (float)AUDIO_SAMPLE_RATE_HZ; /* Time per block */

    for (int i = 0; i < NUM_STRINGS; i++)
    {
        strings[i].target_freq = open_strings[i];
        
        /* Pre-calculate math constants to save CPU in the loop */
        float omega = 2.0f * PI * strings[i].target_freq / (float)AUDIO_SAMPLE_RATE_HZ;
        strings[i].cos_w = cosf(omega);
        strings[i].sin_w = sinf(omega);
        strings[i].coeff = 2.0f * strings[i].cos_w;
        
        strings[i].prev_phase = 0.0f;
        strings[i].current_freq = strings[i].target_freq;
        strings[i].magnitude = 0.0f;
    }
}

void Goertzel_ProcessBlock(uint16_t *adc_buffer) {
    
    for (int i = 0; i < NUM_STRINGS; i++)
    {
        GoertzelState_t *state = &strings[i];
        float s_prev1 = 0.0f;
        float s_prev2 = 0.0f;

        /* 1. The lightning-fast IIR Loop */
        for (uint32_t n = 0; n < block_size; n++)
        {
            /* Convert 12-bit ADC (0-4095) to float (-1.0 to 1.0) */
            float x = ((float)adc_buffer[n] - 2048.0f) / 2048.0f;
            
            float s = x + (state->coeff * s_prev1) - s_prev2;
            s_prev2 = s_prev1;
            s_prev1 = s;
        }

        /* 2. Complex Output Calculation */
        float real = s_prev1 - (s_prev2 * state->cos_w);
        float imag = s_prev2 * state->sin_w;

        /* 3. Magnitude (Volume) */
        state->magnitude = sqrtf((real * real) + (imag * imag));

        /* 4. Phase Extraction & Frequency Refinement */
        float current_phase = atan2f(imag, real);
        float delta_phase = current_phase - state->prev_phase;

        /* Phase Unwrapping: Keep delta between -PI and +PI */
        while (delta_phase > PI)  delta_phase -= 2.0f * PI;
        while (delta_phase < -PI) delta_phase += 2.0f * PI;

        /* Calculate physical frequency offset */
        float freq_error = delta_phase / (2.0f * PI * dt);
        
        /* Update final estimated frequency */
        state->current_freq = state->target_freq + freq_error;
        state->prev_phase = current_phase;
    }
}

float Goertzel_GetExactFreq(uint8_t string_index) {
    if (string_index >= NUM_STRINGS) return 0.0f;
    return strings[string_index].current_freq;
}

float Goertzel_GetMagnitude(uint8_t string_index) {
    if (string_index >= NUM_STRINGS) return 0.0f;
    return strings[string_index].magnitude;
}
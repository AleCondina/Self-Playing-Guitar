#include "fft_processing.h"
#include "application_config.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define TIM6_CALIBRATION_FACTOR 1.005245182f // using a calibration value since the clk speed is 64Mhz and Fs 6Khz isnt divisible

#define NOISE_GATE_THRESHOLD 120000.0f // to be tuned 

#define SEARCH_START_BIN (60 / FFT_RESOLUTION) // around 60 Hz
#define SEARCH_END_BIN   (440 / FFT_RESOLUTION) // around 440 Hz 

/* =======================
   Static buffers
   ======================= */

static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t fft_mag[FFT_HALF_SIZE];
static float32_t window[FFT_SIZE];

static arm_rfft_fast_instance_f32 fft_instance;

static float fundamental_hz = 0.0f;
static float harmonics[MAX_HARMONICS];

float active_frequencies[NUM_STRINGS];
uint8_t active_string_count;



/* =======================
   Initialization
   ======================= */

void FFT_Init(void) {
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    /* Hanning window */
    arm_hanning_f32(window, FFT_SIZE);
}

/* =======================
   FFT processing
   ======================= */
void FFT_ProcessBlock(uint16_t* audio_buf) 
{
    /* 1. Calculate the dynamic DC Offset (Average) of this specific block */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        sum += audio_buf[i];
    }
    float dc_mean = (float)sum / FFT_SIZE;

    /* 2. Remove DC, Convert to Float, and Apply Hanning Window */
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        /* Subtract mean to center the wave at 0.0f, then apply window */
        fft_input[i] = ((float)audio_buf[i] - dc_mean) * window[i];
    }

    /* 3. Execute the Real FFT */
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

    /* 4. Calculate Magnitudes from Complex Output */
    arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2);

    //TODO: tune noise gate with guitar
    /* --- TEMPORARY NOISE GATE CALIBRATION --- */
    float temp_max_mag = 0.0f;
    for (uint32_t i = SEARCH_START_BIN; i < SEARCH_END_BIN; i++) {
        if (fft_mag[i] > temp_max_mag) {
            temp_max_mag = fft_mag[i];
        }
    }
    /* Print the absolute loudest thing the ADC hears right now */
    printf("[DSP Debug] Peak Magnitude: %.1f\r\n", temp_max_mag);
    /* ---------------------------------------- */



    /* 5. Polyphonic Peak Detection & Noise Gate */
    active_string_count = 0;
    fundamental_hz = 0.0f; /* Reset to 0.0 Hz (Silence) */

    for (uint32_t i = SEARCH_START_BIN; i < SEARCH_END_BIN; i++) {
        
        /* Check 1: Does this bin break through the noise floor? */
        if (fft_mag[i] > NOISE_GATE_THRESHOLD) {
            
            /* Check 2: Is it a Local Maximum? (Taller than left and right neighbors) */
            if (fft_mag[i] > fft_mag[i - 1] && fft_mag[i] > fft_mag[i + 1]) {
                
                /* We found an active string! Apply Parabolic Interpolation */
                float alpha = fft_mag[i - 1];
                float beta  = fft_mag[i];
                float gamma = fft_mag[i + 1];
                
                /* --- 1. THE FATNESS CHECK --- */
                /* Reject peaks that are smeared by nearby resonant frequencies */
                float peak_fatness = (alpha + gamma) / beta;
                if (peak_fatness > 1.3f) { 
                    continue; /* Skip this corrupted peak */
                }
                
                /* --- 2. PARABOLIC INTERPOLATION --- */
                float p = 0.5f * (alpha - gamma) / (alpha - 2.0f * beta + gamma);
                float true_peak_idx = (float)i + p;
                float exact_freq = true_peak_idx * ((float)AUDIO_SAMPLE_RATE_HZ / FFT_SIZE) * TIM6_CALIBRATION_FACTOR;

                /* --- 3. HARMONIC REJECTION --- */
                uint8_t is_harmonic = 0;
                for (uint8_t j = 0; j < active_string_count; j++) {
                    float ratio = exact_freq / active_frequencies[j];
                    float nearest_int = roundf(ratio);
                    
                    /* Using 5% tolerance (0.05f) for physical string inharmonicity */
                    if (fabsf(ratio - nearest_int) < 0.05f) {
                        is_harmonic = 1;
                        break;
                    }
                }

                /* --- 4. STORE VALID STRING --- */
                if (is_harmonic == 0) {
                    if (active_string_count < NUM_STRINGS) {
                        active_frequencies[active_string_count] = exact_freq;
                        active_string_count++;
                    }
                }
            }
        }
    }

    /* 6. Update the legacy 'fundamental_hz' variable for backward compatibility.
     * We'll just set it to the lowest frequency detected (usually the root note). */
    if (active_string_count > 0) {
        fundamental_hz = active_frequencies[0]; 
    }
}



/* =======================
   Accessors
   ======================= */

float FFT_GetFundamental(void) {
    return fundamental_hz;
}

void FFT_GetHarmonics(float *out) {
    memcpy(out, harmonics, sizeof(harmonics));
}

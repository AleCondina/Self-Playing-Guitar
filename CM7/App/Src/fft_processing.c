#include "fft_processing.h"
#include "app_cm7.h"
#include "application_config.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/* FFT_RESOLUTION = 6000/2048 = 2 (integer division, actual 2.929 Hz/bin).
 * E2 (82.41 Hz) is at bin ~28 — SEARCH_START_BIN must be < 28.
 * Starting at 40 Hz → bin 13 gives safe margin below E2.
 * Ending at 1200 Hz → bin 409 covers all 6 open strings + 2nd harmonics. */
#define SEARCH_START_BIN (40   / FFT_RESOLUTION) // ~40 Hz  (below E2 at 82 Hz)
#define SEARCH_END_BIN   (302 / FFT_RESOLUTION) // ~1200 Hz (2nd harmonic of E4)

/* =======================
   Static buffers
   ======================= */

static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t fft_mag[FFT_HALF_SIZE];
static float32_t window[FFT_SIZE];

static arm_rfft_fast_instance_f32 fft_instance;


/* Adaptive per-bin noise floor (exponential moving average) */
static float noise_floor[FFT_HALF_SIZE];
static uint8_t noise_floor_initialized = 0;

float active_frequencies[NUM_STRINGS];
float active_magnitudes[MAX_HARMONICS];
uint8_t active_string_count;
uint8_t fft_print_enabled = 0;
uint8_t fft_sensing_enabled = 1;  /* cleared by calibration controller during solenoid burst */

/* Debug snapshot — updated every frame, printed by app_cm7.c when debug nf is on */
float fft_dbg_mag[3];
float fft_dbg_nf[3];
float fft_dbg_peak;



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
    /* Gate: calibration controller clears this during solenoid bursts to
     * prevent solenoid magnetic-field crosstalk from poisoning the output. */
    if (!fft_sensing_enabled) {
        active_string_count = 0;
        return;
    }

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

        /* Snapshot debug values only when noise floor debug is active */
        if (app_debug_flags & DEBUG_FLAG_NOISE_FLOOR) {
            fft_dbg_mag[0] = fft_mag[28];  fft_dbg_nf[0] = noise_floor[28];
            fft_dbg_mag[1] = fft_mag[67];  fft_dbg_nf[1] = noise_floor[67];
            fft_dbg_mag[2] = fft_mag[112]; fft_dbg_nf[2] = noise_floor[112];
            float peak = 0.0f;
            for (uint32_t i = SEARCH_START_BIN; i < SEARCH_END_BIN; i++) {
                if (fft_mag[i] > peak) peak = fft_mag[i];
            }
            fft_dbg_peak = peak;
        }
        


    /* 5. Update adaptive noise floor (EMA per bin) */
    if (!noise_floor_initialized) {
        /* Seed the noise floor with the first frame's magnitudes */
        for (uint32_t i = 0; i < FFT_HALF_SIZE; i++) {
            noise_floor[i] = fft_mag[i] > NOISE_FLOOR_MIN ? fft_mag[i] : NOISE_FLOOR_MIN;
        }
        noise_floor_initialized = 1;
    } else {
        for (uint32_t i = SEARCH_START_BIN; i < SEARCH_END_BIN; i++) {
            /* Only update noise floor from bins that are NOT currently peaking.
             * A bin is "quiet" if it's below GUARD × the current noise estimate.
             * (Was 2×, raised to 6× to prevent signal dips from ratcheting up
             *  the floor during sustained solenoid-driven playing.) */
            if (fft_mag[i] < noise_floor[i] * NOISE_FLOOR_GUARD) {
                noise_floor[i] = NOISE_FLOOR_ALPHA * fft_mag[i]
                               + (1.0f - NOISE_FLOOR_ALPHA) * noise_floor[i];
            }

            /* Constant decay toward MIN — prevents permanent ratcheting.
             * At 0.997/frame and ~5.86 Hz hop rate, half-life ≈ 40 s. */
            noise_floor[i] *= NOISE_FLOOR_DECAY;

            /* Clamp to [MIN, MAX] */
            if (noise_floor[i] < NOISE_FLOOR_MIN) {
                noise_floor[i] = NOISE_FLOOR_MIN;
            } else if (noise_floor[i] > NOISE_FLOOR_MAX) {
                noise_floor[i] = NOISE_FLOOR_MAX;
            }
        }
    }

    /* 6. Polyphonic Peak Detection with Adaptive Noise Floor
     *    Collect ALL candidates first, then sort by magnitude and keep top NUM_STRINGS. */
    uint8_t candidate_count = 0;
    float cand_freq[MAX_HARMONICS];
    float cand_mag[MAX_HARMONICS];

    for (uint32_t i = SEARCH_START_BIN; i < SEARCH_END_BIN; i++) {

        /* Check 1: Does this bin exceed the LOCAL noise floor by SNR ratio? */
        if (fft_mag[i] > noise_floor[i] * NOISE_FLOOR_SNR) {

            /* Check 2: Is it a Local Maximum? (Taller than left and right neighbors) */
            if (fft_mag[i] > fft_mag[i - 1] && fft_mag[i] > fft_mag[i + 1]) {

                float alpha = fft_mag[i - 1];
                float beta  = fft_mag[i];
                float gamma = fft_mag[i + 1];

                /* --- 1. THE FATNESS CHECK --- */
                float peak_fatness = (alpha + gamma) / beta;
                if (peak_fatness > 1.3f) {
                    continue;
                }

                /* --- 2. PARABOLIC INTERPOLATION --- */
                float p = 0.5f * (alpha - gamma) / (alpha - 2.0f * beta + gamma);
                float true_peak_idx = (float)i + p;
                float exact_freq = true_peak_idx * ((float)AUDIO_SAMPLE_RATE_HZ / FFT_SIZE) * TIMER_CALIBRATION_FACTOR;

                /* --- 3. HARMONIC REJECTION (against already-accepted candidates) --- */
                uint8_t is_harmonic = 0;
                for (uint8_t j = 0; j < candidate_count; j++) {
                    float ratio = exact_freq / cand_freq[j];
                    float nearest_int = roundf(ratio);
                    if (fabsf(ratio - nearest_int) < 0.05f) {
                        is_harmonic = 1;
                        break;
                    }
                }

                /* --- 4. STORE CANDIDATE --- */
                if (is_harmonic == 0 && candidate_count < MAX_HARMONICS) {
                    cand_freq[candidate_count] = exact_freq;
                    cand_mag[candidate_count] = beta;
                    candidate_count++;
                }
            }
        }
    }

    /* 6b. Sort candidates by magnitude (descending) — simple insertion sort, max 8 elements */
    for (uint8_t i = 1; i < candidate_count; i++) {
        float key_mag  = cand_mag[i];
        float key_freq = cand_freq[i];
        int8_t j = i - 1;
        while (j >= 0 && cand_mag[j] < key_mag) {
            cand_mag[j + 1]  = cand_mag[j];
            cand_freq[j + 1] = cand_freq[j];
            j--;
        }
        cand_mag[j + 1]  = key_mag;
        cand_freq[j + 1] = key_freq;
    }

    /* 6c. Keep only the top NUM_STRINGS candidates */
    active_string_count = (candidate_count < NUM_STRINGS) ? candidate_count : NUM_STRINGS;
    for (uint8_t i = 0; i < active_string_count; i++) {
        active_frequencies[i] = cand_freq[i];
        active_magnitudes[i]  = cand_mag[i];
    }
}

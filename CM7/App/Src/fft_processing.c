#include "fft_processing.h"
#include "application_config.h"
#include <string.h>

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

/* =======================
   Initialization
   ======================= */

void FFT_Init(void)
{
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    /* Hanning window */
    arm_hanning_f32(window, FFT_SIZE);
}

/* =======================
   FFT processing
   ======================= */

void FFT_ProcessBlock(uint16_t *adc_buffer)
{
    /* 1️ Convert ADC -> float (-1 to +1) */
    for (uint32_t i = 0; i < FFT_SIZE; i++)
    {
        fft_input[i] = ((float32_t)adc_buffer[i] - 2048.0f) / 2048.0f;
    }

    /* 2️ Apply window */
    arm_mult_f32(fft_input, window, fft_input, FFT_SIZE);

    /* 3️ FFT */
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

    /* 4️ Magnitude */
    arm_cmplx_mag_f32(fft_output, fft_mag, FFT_HALF_SIZE);

    /* 5️ Find fundamental */
    uint32_t peak_bin = 1;
    float32_t peak_val = 0.0f;

    // FFT_HALF_SIZE is the Nyquist frequency (Fs / 2), dividing by 2 again searches for bins half of that
    // For efficiency since our frequencies won't exceed 500 Hz
    // also note: starts at index 1 because arm_rfft_fast_f32 has index 0(real DC component)
    // index 1(real nyquist component)
    // index 2 and 3(real and imaginary parts)
    for (uint32_t i = 1; i < FFT_HALF_SIZE / 2; i++)    
    {
        if (fft_mag[i] > peak_val)
        {
            peak_val = fft_mag[i];
            peak_bin = i;
        }
    }

    fundamental_hz = ((float)peak_bin * AUDIO_SAMPLE_RATE_HZ) / FFT_SIZE;

    /* 6️⃣ Extract harmonics */
    memset(harmonics, 0, sizeof(harmonics));

    for (uint32_t h = 1; h <= MAX_HARMONICS; h++)
    {
        uint32_t bin = (uint32_t)((h * fundamental_hz * FFT_SIZE) / AUDIO_SAMPLE_RATE_HZ);

        if (bin >= FFT_HALF_SIZE)
            break;

        float32_t energy = 0.0f;

        /* Small bin neighborhood to reduce leakage */
        for (int32_t k = -2; k <= 2; k++)
        {
            if ((int32_t)bin + k > 0 && (bin + k) < FFT_HALF_SIZE)
                energy += fft_mag[bin + k];
        }

        harmonics[h - 1] = energy;
    }
}

/* =======================
   Accessors
   ======================= */

float FFT_GetFundamental(void)
{
    return fundamental_hz;
}

void FFT_GetHarmonics(float *out)
{
    memcpy(out, harmonics, sizeof(harmonics));
}

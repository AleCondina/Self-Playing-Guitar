/**
 ******************************************************************************
 * @file    audio_input.c
 * @brief   Audio input implementation (ADC + DMA for guitar pickup)
 * @note    Implements circular DMA with ping-pong buffering for audio acquisition
 ******************************************************************************
 */

#include "audio_input.h"
#include "application_config.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ADC and DMA Handle definitions */
extern ADC_HandleTypeDef hadc3;           /* ADC3 for audio input */
extern DMA_HandleTypeDef hdma_adc3;       /* DMA for ADC3 */


#if defined(__ICCARM__)
#pragma location = ".RAM_d3"

/* Audio buffer: Circular DMA buffer (2x half-buffers for ping-pong) */
static uint16_t audio_buffer[AUDIO_BUFFER_SIZE] = {0};

#elif defined(__CC_ARM) || defined(__GNUC__)
__attribute__((section(".audio_buffer"))) static uint16_t audio_buffer[AUDIO_BUFFER_SIZE];
#endif

// managed buffer window that will take in half of the DMA samples at a time
static uint16_t fft_window_buffer[FFT_SIZE] = {0};

/* Flag to indicate which half-buffer has new data */
static volatile uint8_t new_samples_ready = 0;

/* Flag to indicate if data has been read (for detecting new data) */
static volatile uint8_t data_processed = 1;



/**
 * @brief DMA half-transfer complete callback (first half of buffer ready)
 * @param hdma: DMA handle pointer
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC3) {
        new_samples_ready = 0;  /* First half-buffer ready */
        data_processed = 0;     /* Mark that new data is available */
    }
}

/**
 * @brief DMA transfer complete callback (second half of buffer ready)
 * @param hdma: DMA handle pointer
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC3) {
        new_samples_ready = 1;  /* Second half-buffer ready */
        data_processed = 0;     /* Mark that new data is available */
    }
}

int32_t Audio_Init(void) {
    /* ADC3 is initialized in MX_ADC3_Init (main.c). Nothing to do here. */
    return 0;
}

int32_t Audio_Start(void) {

    printf("[CM7] Audio buffer address: %p\r\n", (void*)audio_buffer);
    /* Start ADC3 in DMA circular mode */
    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)audio_buffer, AUDIO_BUFFER_SIZE) != HAL_OK) {
        return -1;  /* Failed to start ADC with DMA */
    }
    
    new_samples_ready = 0;
    return 0;  /* Success */
}

void Audio_Stop(void) {
    /* Stop ADC3 and DMA */
    HAL_ADC_Stop_DMA(&hadc3);
    new_samples_ready = 0;
}

uint8_t Audio_HasNewSamples(void) {
    /* Return 1 if new samples are available (not yet processed), 0 otherwise */
    return !data_processed;
}

uint16_t* Audio_GetBuffer(void) {
    // Sliding the window buffer to the left, so second half to the [0]. New elements replacing the old
    memmove(&fft_window_buffer[0], &fft_window_buffer[FFT_HALF_SIZE], FFT_HALF_SIZE * sizeof(uint16_t));
    
    /* Return pointer to the ready half-buffer */
    if (new_samples_ready == 0) {
        // DMA filled up first half of the audio_buffer
        memcpy(&fft_window_buffer[FFT_HALF_SIZE], &audio_buffer[0], FFT_HALF_SIZE * sizeof(uint16_t));
    } else {
        // DMA filled up second half of the audio_buffer
        memcpy(&fft_window_buffer[FFT_HALF_SIZE], &audio_buffer[FFT_HALF_SIZE], FFT_HALF_SIZE * sizeof(uint16_t));
    }

    // returning fully assembled, FFT_SIZE sample, 50% overlapped block
    return fft_window_buffer;
}

void Audio_MarkProcessed(void) {
    /* Mark the current buffer as processed */
    data_processed = 1;
}

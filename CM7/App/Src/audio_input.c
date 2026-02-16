/**
 ******************************************************************************
 * @file    audio_input.c
 * @brief   Audio input implementation (ADC + DMA for guitar pickup)
 * @note    Implements circular DMA with ping-pong buffering for audio acquisition
 ******************************************************************************
 */

#include "audio_input.h"
#include <stddef.h>
#include <string.h>

/* ADC and DMA Handle definitions */
extern ADC_HandleTypeDef hadc3;           /* ADC3 for audio input */
extern DMA_HandleTypeDef hdma_adc3;       /* DMA for ADC3 */

/* Audio buffer: Circular DMA buffer (2x half-buffers for ping-pong) */
static uint16_t audio_buffer[AUDIO_BUFFER_SIZE] = {0};

/* Flag to indicate which half-buffer has new data */
static volatile uint8_t new_samples_ready = 0;

/* Flag to indicate if data has been read (for detecting new data) */
static volatile uint8_t data_processed = 1;



/**
 * @brief DMA half-transfer complete callback (first half of buffer ready)
 * @param hdma: DMA handle pointer
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3) {
        new_samples_ready = 0;  /* First half-buffer ready */
        data_processed = 0;     /* Mark that new data is available */
    }
}

/**
 * @brief DMA transfer complete callback (second half of buffer ready)
 * @param hdma: DMA handle pointer
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC3) {
        new_samples_ready = 1;  /* Second half-buffer ready */
        data_processed = 0;     /* Mark that new data is available */
    }
}

int32_t Audio_Init(void)
{
    /* ADC3 is initialized in MX_ADC3_Init (main.c). Nothing to do here. */
    return 0;
}

int32_t Audio_Start(void)
{
    /* Start ADC3 in DMA circular mode */
    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*)audio_buffer, AUDIO_BUFFER_SIZE) != HAL_OK) {
        return -1;  /* Failed to start ADC with DMA */
    }
    
    new_samples_ready = 0;
    return 0;  /* Success */
}

void Audio_Stop(void)
{
    /* Stop ADC3 and DMA */
    HAL_ADC_Stop_DMA(&hadc3);
    new_samples_ready = 0;
}

uint8_t Audio_HasNewSamples(void)
{
    /* Return 1 if new samples are available (not yet processed), 0 otherwise */
    return !data_processed;
}

uint16_t* Audio_GetBuffer(void)
{
    /* Return pointer to the ready half-buffer */
    if (new_samples_ready == 0) {
        /* First half: samples [0 ... AUDIO_BUFFER_SIZE/2) */
        return &audio_buffer[0];
    } else {
        /* Second half: samples [AUDIO_BUFFER_SIZE/2 ... AUDIO_BUFFER_SIZE) */
        return &audio_buffer[AUDIO_BUFFER_SIZE / 2];
    }
}

void Audio_MarkProcessed(void)
{
    /* Mark the current buffer as processed */
    data_processed = 1;
}

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
ADC_HandleTypeDef hadc3;           /* ADC3 for audio input */
DMA_HandleTypeDef hdma_adc3;       /* DMA for ADC3 */

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
    /* Initialize ADC3 with DMA in circular mode */
    ADC_ChannelConfTypeDef sConfig = {0};
    
    /* ADC3 Configuration:
     * - Resolution: 12-bit
     * - Sample time: ~13.5 cycles for accuracy
     * - Clock: APB2 (ADC clock), max 72 MHz
     * - Sampling at ~44.1 kHz
     */
    
    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;      /* 480MHz/4 = 120MHz → 60MHz after internal ÷2 = 60MHz */
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode          = ADC_SCAN_DISABLE;          /* Single channel */
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait      = DISABLE;
    hadc3.Init.ContinuousConvMode    = ENABLE;                    /* Continuous conversion for DMA */
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        /* Software trigger */
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;  /* Circular DMA */
    hadc3.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        return -1;  /* Initialization failed */
    }
    
    /* Configure ADC3 regular channel */
    /* Assuming guitar pickup is connected to ADC3 channel (to be determined from hardware) */
    /* For now, using a placeholder channel - adjust based on actual hardware setup */
    sConfig.Channel       = ADC_CHANNEL_0;       /* TODO: Change to actual input channel */
    sConfig.Rank          = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime  = ADC_SAMPLETIME_2CYCLES_5;  /* ~0.4 µs per sample @ 60 MHz */
    sConfig.SingleDiff    = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber  = ADC_OFFSET_NONE;
    sConfig.Offset        = 0;
    
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        return -1;  /* Channel configuration failed */
    }
    
    return 0;  /* Success */
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

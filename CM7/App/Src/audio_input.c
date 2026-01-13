/**
 ******************************************************************************
 * @file    audio_input.c
 * @brief   Audio input implementation (placeholder)
 * @note    To be implemented when ADC peripheral is configured
 ******************************************************************************
 */

#include "audio_input.h"

/* Placeholder implementation */

int32_t Audio_Init(void)
{
    /* TODO: Initialize ADC3 + DMA */
    return 0;
}

int32_t Audio_Start(void)
{
    /* TODO: Start ADC in circular DMA mode */
    return 0;
}

void Audio_Stop(void)
{
    /* TODO: Stop ADC */
}

uint8_t Audio_HasNewSamples(void)
{
    /* TODO: Check DMA callback flags */
    return 0;
}

uint16_t* Audio_GetBuffer(uint8_t buffer_half)
{
    /* TODO: Return pointer to audio buffer */
    (void)buffer_half;
    return NULL;
}

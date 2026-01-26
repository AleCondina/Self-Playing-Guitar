/**
 ******************************************************************************
 * @file    audio_input.c
 * @brief   Audio input implementation (placeholder)
 * @note    To be implemented when ADC peripheral is configured
 ******************************************************************************
 */

#include "audio_input.h"
#include <stddef.h>

/* Placeholder implementation */

// Checking if DSP fft library is working
void test(void) {
    float32_t a[4] = {1,2,3,4};
    float32_t b[4] = {1,1,1,1};
    float32_t c[4];

    arm_add_f32(a, b, c, 4);
}

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

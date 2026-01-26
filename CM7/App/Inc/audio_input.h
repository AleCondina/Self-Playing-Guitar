/**
 ******************************************************************************
 * @file    audio_input.h
 * @brief   Audio input (ADC + DMA) API
 * @note    Placeholder for future ADC/DMA audio acquisition
 ******************************************************************************
 */

#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <stdint.h>
#include "application_config.h"
#include "arm_math.h"


// TODO: remove
void test(void);

/**
 * @brief Initialize audio input (ADC + DMA)
 * @return 0 on success, -1 on error
 * @note  Placeholder - to be implemented later
 */
int32_t Audio_Init(void);

/**
 * @brief Start audio acquisition
 * @return 0 on success, -1 on error
 */
int32_t Audio_Start(void);

/**
 * @brief Stop audio acquisition
 */
void Audio_Stop(void);

/**
 * @brief Check if new audio samples are available
 * @return 1 if new samples ready, 0 otherwise
 */
uint8_t Audio_HasNewSamples(void);

/**
 * @brief Get pointer to audio buffer
 * @param buffer_half 0 for first half, 1 for second half
 * @return Pointer to audio buffer
 */
uint16_t* Audio_GetBuffer(uint8_t buffer_half);

#endif /* AUDIO_INPUT_H */

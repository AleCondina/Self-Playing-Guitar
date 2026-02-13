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
#include "stm32h7xx_hal.h"


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
 * @brief Mark the current audio buffer as processed
 * @note Call this after processing a buffer to allow new data to be flagged
 */
void Audio_MarkProcessed(void);

/**
 * @brief Get pointer to the ready audio buffer
 * @return Pointer to the ready half-buffer (automatically determined internally)
 */
uint16_t* Audio_GetBuffer(void);

#endif /* AUDIO_INPUT_H */

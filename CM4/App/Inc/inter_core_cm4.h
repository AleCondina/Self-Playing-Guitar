/**
 ******************************************************************************
 * @file    inter_core_cm4.h
 * @brief   CM4 inter-core communication API
 * @note    This file provides functions for CM4 to read commands from CM7
 *          and write status updates to CM7 via shared memory.
 ******************************************************************************
 */

#ifndef INTER_CORE_CM4_H
#define INTER_CORE_CM4_H

#include <stdint.h>
#include "shared_memory.h"

/**
 * @brief Initialize CM4 inter-core communication
 * @note  Must be called after HAL_Init()
 * @return 0 on success, -1 on error
 */
int32_t InterCore_CM4_Init(void);

/**
 * @brief Read command flags from CM7
 * @param commands Pointer to receive command flags
 * @return Number of pending commands (0 if none)
 */
uint32_t InterCore_ReadCommands(uint32_t *commands);

/**
 * @brief Acknowledge command processing
 * @param command_flag Command flag to clear
 * @note  Call this after processing a command to clear the flag
 */
void InterCore_AckCommand(uint32_t command_flag);

/**
 * @brief Write motor status to CM7
 * @param status Status flags (MOTOR_STATUS_*)
 * @param positions Array of current positions (or NULL)
 * @param errors Error flags (or NULL)
 */
void InterCore_WriteStatus(uint32_t status, const int32_t positions[NUM_MOTORS], uint32_t errors);

/**
 * @brief Notify CM7 that status update is available
 * @note  Internal function - called by InterCore_WriteStatus()
 */
void InterCore_NotifyCM7(void);

/**
 * @brief Update CM4 heartbeat counter
 * @note  Call this in main control loop
 */
void InterCore_UpdateHeartbeat(void);

#endif /* INTER_CORE_CM4_H */

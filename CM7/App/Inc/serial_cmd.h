/**
 ******************************************************************************
 * @file    serial_cmd.h
 * @brief   Serial command interface for motor control
 * @note    Parses text commands from UART and dispatches inter-core calls
 ******************************************************************************
 */

#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

#include <stdint.h>

/**
 * @brief Initialize serial command processor
 * @note  Call after Debug_UART_StartRx()
 */
void SerialCmd_Init(void);

/**
 * @brief Process one pending command if available
 * @note  Call from main loop. Non-blocking.
 * @return 1 if a command was processed, 0 if none pending
 */
uint8_t SerialCmd_Process(void);

#endif /* SERIAL_CMD_H */

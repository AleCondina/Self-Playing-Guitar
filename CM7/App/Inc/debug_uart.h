/**
 ******************************************************************************
 * @file    debug_uart.h
 * @brief   Debug UART utilities for CM7
 * @note    Provides printf redirection and debug helpers
 ******************************************************************************
 */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdint.h>

/**
 * @brief Initialize debug UART
 * @note  Must be called after UART peripheral is initialized
 * @return 0 on success, -1 on error
 */
int32_t Debug_Init(void);

/**
 * @brief Print debug message
 * @param fmt Printf-style format string
 * @note  Uses printf internally (redirected to UART)
 */
void Debug_Printf(const char *fmt, ...);

/**
 * @brief Print shared memory diagnostics
 * @note  Displays CM7/CM4 heartbeats, motor status, errors
 */
void Debug_PrintSharedMemory(void);

/**
 * @brief Print motor status
 * @param motor_id Motor index (0-5)
 */
void Debug_PrintMotorStatus(uint8_t motor_id);

/**
 * @brief Start UART RX interrupt reception
 * @note  Call once after BSP_COM_Init() to begin receiving characters
 */
void Debug_UART_StartRx(void);

/**
 * @brief Check if a complete command line is available
 * @return 1 if line ready, 0 if not
 */
uint8_t Debug_UART_IsLineReady(void);

/**
 * @brief Get the received command line
 * @param buffer  Destination buffer (caller-provided)
 * @param max_len Maximum bytes to copy
 * @return Length of string copied (excluding null terminator)
 */
uint16_t Debug_UART_GetLine(char *buffer, uint16_t max_len);

/**
 * @brief Drain echo buffer (call frequently from main loop)
 */
void Debug_UART_FlushEcho(void);

#endif /* DEBUG_UART_H */

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

#endif /* DEBUG_UART_H */

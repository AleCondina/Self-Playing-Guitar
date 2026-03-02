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
 * @brief Start UART receive (call once before polling)
 */
void Debug_UART_StartRx(void);

/**
 * @brief Check if a complete line is ready
 * @return 1 if a line is buffered and ready to read, 0 otherwise
 * @note  Polls UART internally (non-blocking)
 */
uint8_t Debug_UART_IsLineReady(void);

/**
 * @brief Copy the buffered line into caller's buffer and reset
 * @param buf Destination buffer
 * @param size Buffer size
 */
void Debug_UART_GetLine(char *buf, uint16_t size);

/**
 * @brief Process UART input (non-blocking)
 * @note  Call this periodically from main loop to handle serial commands
 */
void Debug_ProcessInput(void);

/**
 * @brief Print available commands
 */
void Debug_PrintHelp(void);

/**
 * @brief Print current motor tuning status
 */
void Debug_PrintTuningStatus(void);

/**
 * @brief Initialize motor tuning defaults in shared memory
 */
void Debug_InitMotorDefaults(void);

/**
 * @brief Print solenoid help
 */
void Debug_PrintSolenoidHelp(void);

/**
 * @brief Print solenoid status
 */
void Debug_PrintSolenoidStatus(void);

#endif /* DEBUG_UART_H */

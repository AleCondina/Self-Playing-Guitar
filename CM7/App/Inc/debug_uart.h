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
#include "stm32h7xx_hal.h"

/* USART2 handle — defined in debug_uart.c */
extern UART_HandleTypeDef hdebug_uart;

/**
 * @brief Initialize USART2 for debug (PD5=TX, PD6=RX, 115200 8N1)
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
 * @note  Call once after Debug_Init() to begin receiving characters
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

/**
 * @brief Reset UART RX state and re-arm interrupt
 * @note  Call after heavy printf output (calibration, LUT build) to recover
 *        from any overrun errors that left the HAL RX state machine stuck
 */
void Debug_UART_ResetRx(void);

#endif /* DEBUG_UART_H */

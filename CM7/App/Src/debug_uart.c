/**
 ******************************************************************************
 * @file    debug_uart.c
 * @brief   Debug UART utilities implementation
 ******************************************************************************
 */

#include "debug_uart.h"
#include "shared_memory.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

/* External UART handle (defined in main.c) */
extern UART_HandleTypeDef huart3;

/**
 * @brief Initialize debug UART
 */
int32_t Debug_Init(void)
{
    /* UART is initialized in main.c via CubeMX */
    /* This function is here for future expansion (e.g., DMA mode) */
    return 0;
}

/**
 * @brief Printf redirection to UART
 * @note  This is called by printf() - implements _write syscall
 */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Unused parameter */

    if (HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    } else {
        return -1;
    }
}

/**
 * @brief Print debug message
 */
void Debug_Printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    printf("%s", buffer);
}

/**
 * @brief Print shared memory diagnostics
 */
void Debug_PrintSharedMemory(void)
{
    printf("\r\n===== Shared Memory Diagnostics =====\r\n");
    printf("CM7 Heartbeat: %lu\r\n", g_shared_memory.cm7_heartbeat);
    printf("CM4 Heartbeat: %lu\r\n", g_shared_memory.cm4_heartbeat);
    printf("CM4 Loop Hz:   %lu\r\n", g_shared_memory.cm4_loop_hz);
    printf("CM4 Overruns:  %lu\r\n", g_shared_memory.cm4_loop_overruns);
    printf("CM4 Max Time:  %lu us\r\n", g_shared_memory.cm4_max_loop_time_us);
    printf("Command Flags: 0x%08lX\r\n", g_shared_memory.motor_command_flags);
    printf("Status Flags:  0x%08lX\r\n", g_shared_memory.motor_status_flags);
    printf("Motor Errors:  0x%08lX\r\n", g_shared_memory.motor_errors);
    printf("CM7->CM4 Msgs: %lu\r\n", g_shared_memory.cm7_to_cm4_msg_count);
    printf("CM4->CM7 Msgs: %lu\r\n", g_shared_memory.cm4_to_cm7_msg_count);
    printf("=====================================\r\n\r\n");
}

/**
 * @brief Print motor status
 */
void Debug_PrintMotorStatus(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) {
        printf("Invalid motor ID: %u\r\n", motor_id);
        return;
    }

    printf("Motor %u: Speed=%ld steps/s, Pos=%ld steps\r\n",
           motor_id,
           g_shared_memory.current_speeds[motor_id],
           g_shared_memory.current_positions[motor_id]);
}

/**
 ******************************************************************************
 * @file    debug_uart.c
 * @brief   Debug UART utilities implementation
 ******************************************************************************
 */

#include "debug_uart.h"
#include "shared_memory.h"
#include "stm32h747xx.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* ===== USART2 handle (PD5=TX, PD6=RX) ===== */
UART_HandleTypeDef hdebug_uart;

/* ===== UART RX State ===== */
#define UART_RX_BUFFER_SIZE  128

static uint8_t rx_byte;                              /* Single byte for HAL_UART_Receive_IT */
static char    rx_line_buffer[UART_RX_BUFFER_SIZE];  /* Accumulated line */
static uint16_t rx_line_pos = 0;                     /* Current position in line buffer */
static volatile uint8_t rx_line_ready = 0;           /* Flag: complete line available */

/**
 * @brief Initialize USART2 for debug (PD5=TX, PD6=RX, 115200 8N1)
 */
int32_t Debug_Init(void)
{
    /* --- GPIO & clock setup (MspInit equivalent) --- */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6;   /* PD5=TX, PD6=RX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* --- UART peripheral setup --- */
    hdebug_uart.Instance          = USART2;
    hdebug_uart.Init.BaudRate     = 115200;
    hdebug_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    hdebug_uart.Init.StopBits     = UART_STOPBITS_1;
    hdebug_uart.Init.Parity       = UART_PARITY_NONE;
    hdebug_uart.Init.Mode         = UART_MODE_TX_RX;
    hdebug_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    hdebug_uart.Init.OverSampling = UART_OVERSAMPLING_8;

    if (HAL_UART_Init(&hdebug_uart) != HAL_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Printf redirection to UART
 * @note  This is called by printf() - implements _write syscall
 */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Unused parameter */

    /* Use USART2 debug handle */
    if (HAL_UART_Transmit(&hdebug_uart, (uint8_t*)ptr, len, HAL_MAX_DELAY) == HAL_OK) {
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

/* ===== UART RX Functions ===== */

/**
 * @brief Start UART RX interrupt reception
 */
void Debug_UART_StartRx(void)
{
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&hdebug_uart, &rx_byte, 1);
}

/**
 * @brief Check if a complete command line is available
 */
uint8_t Debug_UART_IsLineReady(void)
{
    return rx_line_ready;
}

/**
 * @brief Get the received command line
 */
uint16_t Debug_UART_GetLine(char *buffer, uint16_t max_len)
{
    if (!rx_line_ready) return 0;

    uint16_t len = rx_line_pos;
    if (len >= max_len) len = max_len - 1;

    memcpy(buffer, rx_line_buffer, len);
    buffer[len] = '\0';

    /* Reset for next line */
    rx_line_pos = 0;
    rx_line_ready = 0;

    return len;
}

/**
 * @brief HAL UART RX complete callback (called from ISR context)
 */
/* Echo ring buffer — ISR stores chars here, main loop drains them via UART TX */
#define ECHO_BUF_SIZE 64
static volatile char    echo_buf[ECHO_BUF_SIZE];
static volatile uint8_t echo_head = 0;
static volatile uint8_t echo_tail = 0;

static inline void echo_put(char c)
{
    uint8_t next = (echo_head + 1) % ECHO_BUF_SIZE;
    if (next != echo_tail) {          /* drop if full */
        echo_buf[echo_head] = c;
        echo_head = next;
    }
}

/**
 * @brief Drain echo buffer from main loop (call from SerialCmd_Process or main)
 *
 * Also re-arms the RX interrupt if it was lost.  This happens when
 * HAL_UART_RxCpltCallback calls HAL_UART_Receive_IT while a blocking
 * HAL_UART_Transmit (used by printf) holds huart->Lock — the re-arm
 * returns HAL_BUSY and the UART goes permanently deaf.  Checking
 * RxState here, outside any TX call, lets us recover.
 */
void Debug_UART_FlushEcho(void)
{
    /* Re-arm RX if it was lost due to HAL lock conflict with TX */
    if (hdebug_uart.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&hdebug_uart, &rx_byte, 1);
    }

    while (echo_tail != echo_head) {
        char c = echo_buf[echo_tail];
        HAL_UART_Transmit(&hdebug_uart, (uint8_t*)&c, 1, 2);
        echo_tail = (echo_tail + 1) % ECHO_BUF_SIZE;
    }
}

/**
 * @brief Reset UART RX state and re-arm interrupt.
 *
 * Call after any long-running operation with heavy printf output (e.g.
 * calibration) to recover from UART overrun errors that may have left
 * the HAL RX state machine stuck.
 */
void Debug_UART_ResetRx(void)
{
    /* Disable UART IRQ to prevent re-entrance while we reset state */
    HAL_NVIC_DisableIRQ(USART2_IRQn);

    /* Clear all UART error flags at register level */
    __HAL_UART_CLEAR_FLAG(&hdebug_uart,
        UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

    /* Force HAL RxState to READY — AbortReceive_IT is async and leaves
     * RxState as BUSY_RX, causing the subsequent Receive_IT to fail.
     * Directly resetting the internal state is the only reliable way. */
    hdebug_uart.RxState = HAL_UART_STATE_READY;

    /* Reset application-level buffers */
    rx_line_pos   = 0;
    rx_line_ready = 0;
    echo_head     = 0;
    echo_tail     = 0;

    /* Re-enable IRQ and re-arm RX */
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&hdebug_uart, &rx_byte, 1);
}

/**
 * @brief UART error callback — re-arms RX after overrun or framing errors.
 * @note  Without this, a UART overrun (ORE) during heavy printf output
 *        permanently disables RX interrupts and the firmware goes deaf.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        /* Clear all UART error flags */
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF  | UART_CLEAR_FEF);
        /* Re-arm the RX interrupt so the firmware can receive again */
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        char c = (char)rx_byte;

        if (c == '\r' || c == '\n') {
            if (rx_line_pos > 0) {
                rx_line_buffer[rx_line_pos] = '\0';
                rx_line_ready = 1;
            }
            echo_put('\r');
            echo_put('\n');
        }
        else if (c == '\b' || c == 0x7F) {  /* Backspace / DEL */
            if (rx_line_pos > 0) {
                rx_line_pos--;
                echo_put('\b');
                echo_put(' ');
                echo_put('\b');
            }
        }
        else if (rx_line_pos < UART_RX_BUFFER_SIZE - 1 && !rx_line_ready) {
            rx_line_buffer[rx_line_pos++] = c;
            echo_put(c);
        }

        /* Always re-arm RX — never skip this */
        HAL_UART_Receive_IT(&hdebug_uart, &rx_byte, 1);
    }
}

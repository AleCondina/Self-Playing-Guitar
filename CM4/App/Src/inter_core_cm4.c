/**
 ******************************************************************************
 * @file    inter_core_cm4.c
 * @brief   CM4 inter-core communication implementation
 ******************************************************************************
 */

#include "inter_core_cm4.h"
#include "stm32h7xx_hal.h"

/**
 * @brief Initialize CM4 inter-core communication
 */
int32_t InterCore_CM4_Init(void)
{
    /* Enable HSEM clock */
    __HAL_RCC_HSEM_CLK_ENABLE();

    /* Enable notification for CM7 -> CM4 communication */
    HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_CM7_TO_CM4));

    return 0;
}

/**
 * @brief Read command flags from CM7
 */
uint32_t InterCore_ReadCommands(uint32_t *commands)
{
    /* Read command flags from shared memory */
    *commands = g_shared_memory.motor_command_flags;

    /* Count number of pending commands */
    uint32_t count = 0;
    uint32_t flags = *commands;
    while (flags) {
        count += (flags & 1);
        flags >>= 1;
    }

    return count;
}

/**
 * @brief Acknowledge command processing
 */
void InterCore_AckCommand(uint32_t command_flag)
{
    /* Clear the command flag */
    g_shared_memory.motor_command_flags &= ~command_flag;

    /* Data synchronization barrier (CM4 has no cache, but good practice) */
    __DSB();
}

/**
 * @brief Write motor status to CM7
 */
void InterCore_WriteStatus(uint32_t status, const int32_t positions[NUM_MOTORS], uint32_t errors)
{
    /* Update status flags */
    g_shared_memory.motor_status_flags = status;

    /* Update positions if provided */
    if (positions != NULL) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            g_shared_memory.current_positions[i] = positions[i];
        }
    }

    /* Update errors */
    g_shared_memory.motor_errors = errors;

    /* Data synchronization barrier */
    __DSB();

    /* Notify CM7 */
    InterCore_NotifyCM7();
}

/**
 * @brief Notify CM7 via HSEM
 */
void InterCore_NotifyCM7(void)
{
    /* Take and release HSEM to trigger CM7 notification */
    HAL_HSEM_FastTake(HSEM_ID_CM4_TO_CM7);
    HAL_HSEM_Release(HSEM_ID_CM4_TO_CM7, 0);

    /* Increment message counter */
    g_shared_memory.cm4_to_cm7_msg_count++;
}

/**
 * @brief Update CM4 heartbeat counter
 */
void InterCore_UpdateHeartbeat(void)
{
    g_shared_memory.cm4_heartbeat++;
}

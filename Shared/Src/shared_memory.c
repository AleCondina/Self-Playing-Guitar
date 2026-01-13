/**
 ******************************************************************************
 * @file    shared_memory.c
 * @brief   Global shared memory instance definition
 * @note    This file defines the actual shared memory instance that is
 *          placed in AXI SRAM (0x24000000) via linker script.
 ******************************************************************************
 */

#include "shared_memory.h"

/**
 * @brief Global shared memory instance
 * @note  Placed in AXI SRAM shared region via SHARED_DATA attribute
 * @note  Initialized to zero by startup code (.bss section behavior)
 * @note  Accessible by both CM7 and CM4 cores
 * @note  CM7 initializes this in main() before releasing CM4
 */
SHARED_DATA SharedMemory_t g_shared_memory = {0};

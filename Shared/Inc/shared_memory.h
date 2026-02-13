/**
 ******************************************************************************
 * @file    shared_memory.h
 * @brief   Shared memory structure for inter-core communication (CM7 <-> CM4)
 * @note    This file defines the shared data structure placed in AXI SRAM
 *          for high-speed, low-latency communication between cores.
 *
 *          Memory coherency is ensured via MPU configuration (non-cacheable)
 *          in CM7 main.c. CM4 has no cache, so direct access is coherent.
 ******************************************************************************
 */

#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include <stdint.h>

/* Linker section attributes for shared memory placement */
#define SHARED_DATA __attribute__((section(".shared_data"))) __attribute__((aligned(32)))
#define SHARED_BSS  __attribute__((section(".shared_bss"))) __attribute__((aligned(32)))

/* DMA buffer attribute (placed in D3 SRAM for DMA accessibility) */
#define DMA_BUFFER  __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))

/* Hardware Semaphore IDs for inter-core synchronization */
#define HSEM_ID_BOOT            0   /* Boot synchronization (existing pattern) */
#define HSEM_ID_CM7_TO_CM4      1   /* CM7 signals CM4 (new command available) */
#define HSEM_ID_CM4_TO_CM7      2   /* CM4 signals CM7 (status update ready) */

/* Application constants */
#define NUM_MOTORS              6   /* 6 guitar strings */
/* NUM_SOLENOIDS defined in application_config.h (currently 3, expandable to 6) */

/* Command flags: CM7 → CM4 (bit field) */
#define CMD_UPDATE_MOTOR_SPEED  (1U << 0)   /* Update motor target speeds */
#define CMD_TRIGGER_SOLENOID    (1U << 1)   /* Trigger solenoid(s) */
#define CMD_HOME_MOTORS         (1U << 2)   /* Home all motors to reference */
#define CMD_EMERGENCY_STOP      (1U << 3)   /* Emergency stop all motion */
#define CMD_RESET_ERRORS        (1U << 4)   /* Clear error flags */
#define CMD_STOP_SOLENOID       (1U << 5)   /* Stop solenoid(s) */

/* Status flags: CM4 → CM7 (bit field) */
#define MOTOR_STATUS_READY      (1U << 0)   /* Motors initialized and ready */
#define MOTOR_STATUS_MOVING     (1U << 1)   /* At least one motor is moving */
#define MOTOR_STATUS_ERROR      (1U << 2)   /* Error condition detected */
#define MOTOR_STATUS_LIMIT_HIT  (1U << 3)   /* Limit switch triggered */
#define MOTOR_STATUS_HOMING     (1U << 4)   /* Homing sequence in progress */

/**
 * @brief Shared memory structure for inter-core communication
 * @note  Size: ~200 bytes, fits comfortably in 256KB shared region
 * @note  32-byte alignment matches STM32H7 L1 cache line size
 * @note  All fields are volatile to prevent compiler optimization
 */
typedef struct {
    /* ===== CM7 → CM4 Command Channel ===== */
    volatile uint32_t motor_command_flags;              /* Command bit field */
    volatile int32_t motor_speeds[NUM_MOTORS];          /* Target speeds (steps/sec) */
    volatile uint32_t solenoid_triggers;                /* Solenoid bit field */
    volatile uint32_t solenoid_duration_ms;             /* Solenoid hold time (ms) */

    /* ===== Solenoid Frequency Control (PWM-based) ===== */
    volatile uint8_t solenoid_id;                       /* Solenoid index (0-2 for 3 solenoids) */
    volatile uint8_t solenoid_string;                   /* String index (0-2: E2, A2, D3) */
    volatile uint8_t solenoid_fret;                     /* Fret index (0-7: open to fret 7) */
    volatile uint8_t solenoid_duty_cycle;               /* Duty cycle 1-99% (0 = use default 50%) */
    volatile uint8_t solenoid_active_mask;              /* Bit field: which solenoids are running */

    /* ===== Motor Tuning Parameters (CM7 → CM4, adjustable via serial) ===== */
    volatile int32_t motor_max_speed;                   /* Max speed (steps/sec), 0 = use default */
    volatile int32_t motor_max_accel;                   /* Max acceleration, 0 = use default */
    volatile int32_t motor_steps_per_move;              /* Steps per oscillation move, 0 = use default */
    volatile uint32_t motor_settle_time_ms;             /* Settling time (ms) between moves */
    volatile uint8_t motor_test_enabled;                /* 0 = stopped, 1 = running oscillation test */
    volatile uint8_t motor_microstep_mode;              /* Microstep mode: 0=full, 1=1/2, 2=1/4, 3=1/8, 4=1/16, 5=1/32 */
    volatile uint8_t motor_microstep_changed;           /* Flag: 1 = CM4 should apply new microstep mode */
    volatile uint8_t motor_test_padding;                /* Padding for alignment */

    /* ===== CM4 → CM7 Status Channel ===== */
    volatile uint32_t motor_status_flags;               /* Status bit field */
    volatile int32_t current_positions[NUM_MOTORS];     /* Current motor positions (steps) */
    volatile int32_t current_speeds[NUM_MOTORS];        /* Current motor speeds (steps/sec) */
    volatile uint32_t motor_errors;                     /* Error code bit field */

    /* ===== Diagnostics & Health Monitoring ===== */
    volatile uint32_t cm7_heartbeat;                    /* CM7 loop counter */
    volatile uint32_t cm4_heartbeat;                    /* CM4 loop counter */
    volatile uint32_t cm4_loop_hz;                      /* Actual CM4 loop rate (Hz) */
    volatile uint32_t cm4_loop_overruns;                /* Timing violation count */
    volatile uint32_t cm4_max_loop_time_us;             /* Worst-case loop time (µs) */

    /* ===== Inter-Core Synchronization ===== */
    volatile uint32_t cm7_to_cm4_msg_count;             /* Messages sent by CM7 */
    volatile uint32_t cm4_to_cm7_msg_count;             /* Messages sent by CM4 */

} __attribute__((aligned(32))) SharedMemory_t;

/* Global shared memory instance (defined in shared_memory.c) */
extern SHARED_DATA SharedMemory_t g_shared_memory;

/**
 * @brief Helper macro to calculate shared memory size
 */
#define SHARED_MEM_SIZE sizeof(SharedMemory_t)

#endif /* SHARED_MEMORY_H */

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
#define CMD_SET_MOTOR_ACCEL     (1U << 6)   /* Set motor acceleration -----*/ 
#define CMD_SET_MOTOR_MAXSPEED  (1U << 7)   /* Set motor max speed -----*/
#define CMD_MOTOR_MOVE_REL      (1U << 8)   /* Relative position move -----*/
#define CMD_MOTOR_MOVE_ABS      (1U << 9)   /* Absolute position move -----*/
#define CMD_SET_MICROSTEP       (1U << 10)  /* Set microstepping mode -----*/
#define CMD_MOTOR_ENABLE        (1U << 11)  /* Enable/disable motor -----*/
#define CMD_MOTOR_ZERO          (1U << 12)  /* Zero motor position -----*/
#define CMD_TRIGGER_SOLENOID_HZ (1U << 13)  /* Trigger solenoid at exact Hz (tuning controller) */
#define CMD_MOTOR_ENABLE_MASK   (1U << 14)  /* Batch enable/disable motors via bitmask */
#define CMD_MOTOR_MOVE_ABS_BATCH    (1U << 15) /* Batch absolute move: positions[] + mask  */
#define CMD_TRIGGER_SOLENOID_HZ_BATCH (1U << 16) /* Batch Hz trigger: freq[]+mask, one shot */
#define CMD_SET_TUNING              (1U << 17)  /* Set tuning mode (standard/drop D)       */

/* Status flags: CM4 → CM7 (bit field) */
#define MOTOR_STATUS_READY      (1U << 0)   /* Motors initialized and ready */
#define MOTOR_STATUS_MOVING     (1U << 1)   /* At least one motor is moving */
#define MOTOR_STATUS_ERROR      (1U << 2)   /* Error condition detected */
#define MOTOR_STATUS_LIMIT_HIT  (1U << 3)   /* Limit switch triggered */
#define MOTOR_STATUS_HOMING     (1U << 4)   /* Homing sequence in progress */

/* Solenoid operation modes */
#define SOLENOID_MODE_INFINITE  0           /* Run until explicitly stopped */
#define SOLENOID_MODE_TIMED     1           /* Auto-stop after time_ms */
#define SOLENOID_MODE_BURST     2           /* Auto-stop after N cycles */

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
    volatile uint8_t solenoid_id;                       /* Solenoid index (0-5 for 6 solenoids) */
    volatile uint8_t solenoid_string;                   /* String index (0-5: E2, A2, D3, G3, B3, E4) */
    volatile uint8_t solenoid_fret;                     /* Fret index (0-7: open to fret 7) */
    volatile uint8_t solenoid_duty_cycle;               /* Duty cycle 1-99% (0 = use default 50%) */
    volatile uint8_t solenoid_active_mask;              /* Bit field: which solenoids are running */

    /* ===== Motor Extended Commands (CM7 → CM4) ===== */
    volatile uint8_t  motor_cmd_id;                    /* Motor ID for current command */
    volatile uint8_t  motor_cmd_microstep_mode;        /* Microstep mode (0-5) */
    volatile uint8_t  motor_cmd_enable;                /* 1=enable, 0=disable */
    volatile uint8_t  motor_cmd_enable_mask;            /* Bitmask for CMD_MOTOR_ENABLE_MASK: bit i → motor i */
    volatile int32_t  motor_cmd_position;              /* Target position (absolute or relative) */
    volatile uint8_t  motor_cmd_batch_mask;                  /* Bitmask for CMD_MOTOR_MOVE_ABS_BATCH */
    volatile int32_t  motor_cmd_batch_positions[NUM_MOTORS]; /* Target positions for batch move      */

    /* Batch solenoid Hz trigger (CMD_TRIGGER_SOLENOID_HZ_BATCH) */
    volatile uint8_t  solenoid_batch_mask;               /* Bit i = trigger solenoid i           */
    volatile float    solenoid_batch_freq_hz[6];         /* Per-solenoid drive frequency (Hz)    */
    volatile uint8_t  solenoid_batch_duty;               /* Shared duty cycle (1-99%)            */
    volatile uint8_t  solenoid_batch_mode;               /* Shared mode (TIMED/INFINITE/BURST)   */
    volatile uint16_t solenoid_batch_time_ms;            /* Shared auto-stop time for TIMED mode */
    volatile int32_t  motor_cmd_accel;                 /* Acceleration value */
    volatile int32_t  motor_cmd_max_speed;             /* Max speed value */

    /* Extended solenoid timing control */
    volatile uint16_t solenoid_activation_time_ms;      /* Auto-stop time (0=infinite, 10-5000ms) */
    volatile uint16_t solenoid_burst_cycles;            /* Auto-stop cycles (0=disabled, 1-1000) */
    volatile uint8_t solenoid_mode;                     /* 0=infinite, 1=timed, 2=burst */
    volatile uint8_t solenoid_padding;                  /* Alignment */
    volatile float solenoid_freq_hz;                 /* Direct Hz drive (CMD_TRIGGER_SOLENOID_HZ) */

    /* ===== Tuning Mode (CM7 → CM4) ===== */
    volatile uint8_t tuning_id;                      /* 0=standard, 1=drop D (see application_config.h) */

    /* ===== CM4 → CM7 Status Channel ===== */
    volatile uint32_t motor_status_flags;               /* Status bit field */
    volatile int32_t current_positions[NUM_MOTORS];     /* Current motor positions (steps) */
    volatile int32_t current_speeds[NUM_MOTORS];        /* Current speed (steps/sec) */
    volatile int32_t target_speeds_status[NUM_MOTORS];  /* Target speed (steps/sec) */
    volatile int32_t max_speeds_status[NUM_MOTORS];     /* Max speed limit (steps/sec) */
    volatile int32_t max_accels_status[NUM_MOTORS];     /* Max accel limit (steps/sec^2) */
    volatile uint8_t motor_enabled[NUM_MOTORS];         /* 1=enabled, 0=disabled */
    volatile uint8_t motor_microstep_status[NUM_MOTORS]; /* Current microstep mode (0-5) */
    volatile uint8_t motor_is_moving[NUM_MOTORS];       /* 1=moving, 0=stopped */
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

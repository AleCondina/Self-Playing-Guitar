/**
 ******************************************************************************
 * @file    inter_core_cm7.c
 * @brief   CM7 inter-core communication implementation
 ******************************************************************************
 */

#include "inter_core_cm7.h"
#include "application_config.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Private variables */
static uint32_t last_cm4_heartbeat = 0;

/**
 * @brief Initialize CM7 inter-core communication
 */
int32_t InterCore_CM7_Init(void)
{
    /* Enable HSEM clock */
    __HAL_RCC_HSEM_CLK_ENABLE();

    /* Configure MPU for shared memory (non-cacheable) */
    InterCore_ConfigMPU();

    /* Initialize shared memory (CM7 is responsible for this) */
    memset((void*)&g_shared_memory, 0, sizeof(SharedMemory_t));

    /* Data synchronization barrier to ensure write completes */
    __DSB();

    /* Initialize heartbeat tracking */
    last_cm4_heartbeat = 0;

    return 0;
}

/**
 * @brief Configure MPU for shared memory region
 */
void InterCore_ConfigMPU(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disable MPU during configuration */
    HAL_MPU_Disable();

    /* Configure shared AXI SRAM region (0x24000000 - 0x24040000) as non-cacheable */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;      /* KEY: Non-cacheable */
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Enable MPU with privileged default access */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief Send motor speed command to CM4
 */
void InterCore_SendMotorSpeed(uint8_t motor_id, int32_t speed_steps_per_sec)
{
    if (motor_id >= NUM_MOTORS) {
        return; /* Invalid motor ID */
    }

    /* Write target speed to shared memory */
    g_shared_memory.motor_speeds[motor_id] = speed_steps_per_sec;

    /* Set command flag */
    g_shared_memory.motor_command_flags |= CMD_UPDATE_MOTOR_SPEED;

    /* Data synchronization barrier */
    __DSB();

    /* Notify CM4 */
    InterCore_NotifyCM4();
}

/**
 * @brief Send all motor speeds at once
 */
void InterCore_SendAllMotorSpeeds(const int32_t speeds[NUM_MOTORS])
{
    /* Copy all speeds to shared memory */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        g_shared_memory.motor_speeds[i] = speeds[i];
    }

    /* Set command flag */
    g_shared_memory.motor_command_flags |= CMD_UPDATE_MOTOR_SPEED;

    /* Data synchronization barrier */
    __DSB();

    /* Notify CM4 */
    InterCore_NotifyCM4();
}

/**
 * @brief Trigger solenoid(s)
 */
void InterCore_TriggerSolenoids(uint8_t solenoid_mask, uint32_t duration_ms)
{
    /* Clamp duration to valid range */
    if (duration_ms < SOLENOID_MIN_DURATION_MS) {
        duration_ms = SOLENOID_MIN_DURATION_MS;
    }
    if (duration_ms > SOLENOID_MAX_DURATION_MS) {
        duration_ms = SOLENOID_MAX_DURATION_MS;
    }

     /* Write to shared memory */
    // g_shared_memory.solenoid_triggers = solenoid_mask & 0x3F; /* Mask to 6 bits */
    // g_shared_memory.solenoid_duration_ms = duration_ms;

    /* Write to shared memory.
     * CM4 CMD_TRIGGER_SOLENOID handler reads solenoid_id (single index), not
     * solenoid_triggers (bitmask).  InterCore_TriggerSolenoids only supports
     * single-solenoid triggering; use the lowest set bit as the solenoid_id. */
    uint8_t sol_id = 0;
    uint8_t mask = solenoid_mask & 0x3Fu;
    while (sol_id < 6u && !((mask >> sol_id) & 1u)) { sol_id++; }
    g_shared_memory.solenoid_id           = sol_id;
    g_shared_memory.solenoid_string       = sol_id;   /* default: string == solenoid */
    g_shared_memory.solenoid_fret         = 0u;
    g_shared_memory.solenoid_mode         = SOLENOID_MODE_TIMED;
    g_shared_memory.solenoid_activation_time_ms = (uint16_t)duration_ms;
    g_shared_memory.solenoid_triggers     = mask;     /* kept for legacy readers */
    g_shared_memory.solenoid_duration_ms  = duration_ms;

    /* Set command flag */
    g_shared_memory.motor_command_flags |= CMD_TRIGGER_SOLENOID;

    /* Data synchronization barrier */
    __DSB();

    /* Notify CM4 */
    InterCore_NotifyCM4();
}

/**
 * @brief Emergency stop
 */
void InterCore_EmergencyStop(void)
{
    /* Set emergency stop flag */
    g_shared_memory.motor_command_flags = CMD_EMERGENCY_STOP;

    /* Data synchronization barrier */
    __DSB();

    /* Notify CM4 (critical priority) */
    InterCore_NotifyCM4();
}

/**
 * @brief Home motors
 */
void InterCore_HomeMotors(void)
{
    /* Set home command flag */
    g_shared_memory.motor_command_flags |= CMD_HOME_MOTORS;

    /* Data synchronization barrier */
    __DSB();

    /* Notify CM4 */
    InterCore_NotifyCM4();
}

/**
 * @brief Send motor acceleration command to CM4
 */
void InterCore_SendMotorAccel(uint8_t motor_id, int32_t accel)
{
    if (motor_id >= NUM_MOTORS) return;
    g_shared_memory.motor_cmd_id = motor_id;
    g_shared_memory.motor_cmd_accel = accel;
    g_shared_memory.motor_command_flags |= CMD_SET_MOTOR_ACCEL;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Send motor max speed command to CM4
 */
void InterCore_SendMotorMaxSpeed(uint8_t motor_id, int32_t max_speed)
{
    if (motor_id >= NUM_MOTORS) return;
    g_shared_memory.motor_cmd_id = motor_id;
    g_shared_memory.motor_cmd_max_speed = max_speed;
    g_shared_memory.motor_command_flags |= CMD_SET_MOTOR_MAXSPEED;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Send relative move command to CM4
 */
void InterCore_SendMotorMoveRelative(uint8_t motor_id, int32_t steps)
{
    if (motor_id >= NUM_MOTORS) return;
    g_shared_memory.motor_cmd_id = motor_id;
    g_shared_memory.motor_cmd_position = steps;
    g_shared_memory.motor_command_flags |= CMD_MOTOR_MOVE_REL;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Send absolute position move command to CM4
 */
void InterCore_SendMotorGoto(uint8_t motor_id, int32_t position)
{
    if (motor_id >= NUM_MOTORS) return;
    g_shared_memory.motor_cmd_id = motor_id;
    g_shared_memory.motor_cmd_position = position;
    g_shared_memory.motor_command_flags |= CMD_MOTOR_MOVE_ABS;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Send microstep mode command to CM4
 */
void InterCore_SendMotorMicrostep(uint8_t motor_id, uint8_t mode)
{
    if (motor_id >= NUM_MOTORS) return;
    g_shared_memory.motor_cmd_id = motor_id;
    g_shared_memory.motor_cmd_microstep_mode = mode;
    g_shared_memory.motor_command_flags |= CMD_SET_MICROSTEP;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Send motor enable/disable command to CM4
 */
void InterCore_SendMotorEnable(uint8_t motor_id, uint8_t enable)
{
    if (motor_id >= NUM_MOTORS) return;
    g_shared_memory.motor_cmd_id = motor_id;
    g_shared_memory.motor_cmd_enable = enable;
    g_shared_memory.motor_command_flags |= CMD_MOTOR_ENABLE;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Send zero position command to CM4
 */
void InterCore_SendMotorZero(uint8_t motor_id)
{
    if (motor_id >= NUM_MOTORS) return;
    g_shared_memory.motor_cmd_id = motor_id;
    g_shared_memory.motor_command_flags |= CMD_MOTOR_ZERO;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Read motor status from CM4
 */
int32_t InterCore_ReadMotorStatus(uint32_t *status, int32_t positions[NUM_MOTORS], uint32_t *errors)
{
    /* Read status flags */
    if (status != NULL) {
        *status = g_shared_memory.motor_status_flags;
    }

    /* Read positions if requested */
    if (positions != NULL) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            positions[i] = g_shared_memory.current_positions[i];
        }
    }

    /* Read errors if requested */
    if (errors != NULL) {
        *errors = g_shared_memory.motor_errors;
    }

    return 0;
}

/**
 * @brief Check if CM4 is alive
 */
uint8_t InterCore_IsCM4Alive(void)
{
    uint32_t current_heartbeat = g_shared_memory.cm4_heartbeat;

    /* Check if heartbeat changed since last check */
    if (current_heartbeat != last_cm4_heartbeat) {
        last_cm4_heartbeat = current_heartbeat;
        return 1; /* CM4 is alive */
    } else {
        return 0; /* CM4 appears stalled */
    }
}

/**
 * @brief Get CM4 diagnostics
 */
void InterCore_GetCM4Diagnostics(uint32_t *loop_hz, uint32_t *overruns, uint32_t *max_time_us)
{
    if (loop_hz != NULL) {
        *loop_hz = g_shared_memory.cm4_loop_hz;
    }
    if (overruns != NULL) {
        *overruns = g_shared_memory.cm4_loop_overruns;
    }
    if (max_time_us != NULL) {
        *max_time_us = g_shared_memory.cm4_max_loop_time_us;
    }
}

/**
 * @brief Trigger a solenoid at an exact frequency in Hz (tuning controller use)
 *
 * Uses CMD_TRIGGER_SOLENOID_HZ so CM4 calls Solenoid_SetFrequencyHz() instead
 * of the string/fret lookup table.  Allows the tuning controller to drive the
 * solenoid at the precise target note frequency once the string is locked.
 *
 * @param solenoid_id  Solenoid index 0–5
 * @param freq_hz      Drive frequency in Hz (40–800 Hz)
 * @param duty_percent Duty cycle 1–99 % (0 = use driver default 50 %)
 * @param mode         SOLENOID_MODE_INFINITE / TIMED / BURST
 * @param time_ms      Auto-stop time for TIMED mode (0 = ignored)
 * @param burst_cycles Auto-stop cycles for BURST mode (0 = ignored)
 */
void InterCore_TriggerSolenoidHz(uint8_t  solenoid_id,
                                  float freq_hz,
                                  uint8_t  duty_percent,
                                  uint8_t  mode,
                                  uint16_t time_ms,
                                  uint16_t burst_cycles)
{
    if (solenoid_id >= NUM_MOTORS) return;

    g_shared_memory.solenoid_id                 = solenoid_id;
    g_shared_memory.solenoid_freq_hz            = freq_hz;
    g_shared_memory.solenoid_duty_cycle         = duty_percent;
    g_shared_memory.solenoid_mode               = mode;
    g_shared_memory.solenoid_activation_time_ms = time_ms;
    g_shared_memory.solenoid_burst_cycles       = burst_cycles;
    g_shared_memory.motor_command_flags        |= CMD_TRIGGER_SOLENOID_HZ;
    __DSB();
    InterCore_NotifyCM4();
}

/**
 * @brief Notify CM4 via HSEM
 */
void InterCore_NotifyCM4(void)
{
    /* Take and release HSEM to trigger CM4 notification */
    HAL_HSEM_FastTake(HSEM_ID_CM7_TO_CM4);
    HAL_HSEM_Release(HSEM_ID_CM7_TO_CM4, 0);

    /* Increment message counter */
    g_shared_memory.cm7_to_cm4_msg_count++;

    /* Update CM7 heartbeat */
    g_shared_memory.cm7_heartbeat++;
}

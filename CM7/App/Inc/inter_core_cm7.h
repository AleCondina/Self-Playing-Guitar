/**
 ******************************************************************************
 * @file    inter_core_cm7.h
 * @brief   CM7 inter-core communication API
 * @note    This file provides functions for CM7 to send commands to CM4
 *          and read status updates from CM4 via shared memory.
 *
 *          Cache coherency is handled via MPU configuration (non-cacheable)
 *          for the shared memory region. No explicit cache maintenance needed.
 ******************************************************************************
 */

#ifndef INTER_CORE_CM7_H
#define INTER_CORE_CM7_H

#include <stdint.h>
#include "shared_memory.h"

/**
 * @brief Initialize CM7 inter-core communication
 * @note  Must be called after HAL_Init() and before accessing shared memory
 * @note  Configures MPU to mark shared memory region as non-cacheable
 * @return 0 on success, -1 on error
 */
int32_t InterCore_CM7_Init(void);

/**
 * @brief Trigger solenoid(s)
 * @param solenoid_mask Bit field of solenoids to trigger (bit 0 = solenoid 0, etc.)
 * @param duration_ms Duration in milliseconds (10-500ms)
 * @note  Example: solenoid_mask = 0x05 triggers solenoids 0 and 2
 */
void InterCore_TriggerSolenoids(uint8_t solenoid_mask, uint32_t duration_ms);

/**
 * @brief Send emergency stop command to CM4
 * @note  Immediately stops all motors and deactivates all solenoids
 * @note  Critical safety function - bypasses normal command queue
 */
void InterCore_EmergencyStop(void);


/**
 * @brief Check if CM4 is alive (heartbeat check)
 * @return 1 if CM4 is running, 0 if CM4 appears stalled
 * @note  Call this periodically (e.g., 100ms) to detect CM4 crashes
 */
uint8_t InterCore_IsCM4Alive(void);

/**
 * @brief Get CM4 diagnostics (loop rate, overruns, etc.)
 * @param loop_hz Pointer to receive actual CM4 loop rate (Hz)
 * @param overruns Pointer to receive timing overrun count
 * @param max_time_us Pointer to receive worst-case loop time (µs)
 */
void InterCore_GetCM4Diagnostics(uint32_t *loop_hz, uint32_t *overruns, uint32_t *max_time_us);

/**
 * @brief Send motor acceleration command to CM4
 */


//-----------------------------START OF MOTOR EXTENDED FUNCS -----------------
/**
 * @brief Send home command to CM4 (calibration sequence)
 * @note  CM4 will move all motors to home position using limit switches
 * @note  This is a blocking operation - monitor status with InterCore_ReadMotorStatus()
 */
void InterCore_HomeMotors(void);

/**
 * @brief Read motor status from CM4
 * @param status Pointer to receive status flags
 * @param positions Pointer to array of 6 current positions (or NULL if not needed)
 * @param errors Pointer to receive error flags (or NULL if not needed)
 * @return 0 on success, -1 if CM4 not responding
 */
int32_t InterCore_ReadMotorStatus(uint32_t *status, int32_t positions[NUM_MOTORS], uint32_t *errors);

/**
 * @brief Send motor speed command to CM4
 * @param motor_id Motor index (0-5)
 * @param speed_steps_per_sec Target speed in steps/sec (signed for direction)
 * @note  Positive = forward, negative = reverse, 0 = stop
 * @note  Notifies CM4 via HSEM after writing command
 */
void InterCore_SendMotorSpeed(uint8_t motor_id, int32_t speed_steps_per_sec);

/**
 * @brief Send multiple motor speeds at once (atomic update)
 * @param speeds Array of 6 motor speeds (steps/sec)
 * @note  More efficient than calling InterCore_SendMotorSpeed() 6 times
 */
void InterCore_SendAllMotorSpeeds(const int32_t speeds[NUM_MOTORS]);


void InterCore_SendMotorAccel(uint8_t motor_id, int32_t accel);

/**
 * @brief Send motor max speed command to CM4
 */
void InterCore_SendMotorMaxSpeed(uint8_t motor_id, int32_t max_speed);

/**
 * @brief Send relative move command to CM4
 */
void InterCore_SendMotorMoveRelative(uint8_t motor_id, int32_t steps);

/**
 * @brief Send absolute position move command to CM4
 */
void InterCore_SendMotorGoto(uint8_t motor_id, int32_t position);

/**
 * @brief Send microstep mode command to CM4
 * @param mode 0=full, 1=half, 2=quarter, 3=eighth, 4=16th, 5=32nd
 */
void InterCore_SendMotorMicrostep(uint8_t motor_id, uint8_t mode);

/**
 * @brief Send motor enable/disable command to CM4
 * @param enable 1=enable, 0=disable
 */
void InterCore_SendMotorEnable(uint8_t motor_id, uint8_t enable);

/**
 * @brief Send zero position command to CM4
 */
void InterCore_SendMotorZero(uint8_t motor_id);


/**
 * @brief Trigger a solenoid at an exact frequency in Hz (tuning controller)
 * @param solenoid_id  Solenoid index 0–5
 * @param freq_hz      Drive frequency 40–800 Hz
 * @param duty_percent Duty cycle 1–99 % (0 = default 50 %)
 * @param mode         SOLENOID_MODE_INFINITE / TIMED / BURST
 * @param time_ms      Auto-stop time for TIMED mode (0 = N/A)
 * @param burst_cycles Auto-stop cycles for BURST mode (0 = N/A)
 */
void InterCore_TriggerSolenoidHz(uint8_t  solenoid_id,
                                  float freq_hz,
                                  uint8_t  duty_percent,
                                  uint8_t  mode,
                                  uint16_t time_ms,
                                  uint16_t burst_cycles);

//----------------END OF MOTOR FUNCS -----------------



/**
 * @brief Notify CM4 that new command is available
 * @note  Internal function - called by InterCore_Send*() functions
 * @note  Releases HSEM to trigger CM4 notification interrupt
 */
void InterCore_NotifyCM4(void);

/**
 * @brief MPU configuration for shared memory (non-cacheable)
 * @note  Called by InterCore_CM7_Init()
 * @note  Marks 0x24000000 - 0x24040000 (256KB) as non-cacheable
 */
void InterCore_ConfigMPU(void);

int InterCore_PlayNote_Tune(uint8_t string_id,
                            float target_freq_hz,
                            uint16_t duration_ms,
                            uint8_t velocity,
                            uint32_t start_time_ms);

int InterCore_StopAll(void);

#endif /* INTER_CORE_CM7_H */

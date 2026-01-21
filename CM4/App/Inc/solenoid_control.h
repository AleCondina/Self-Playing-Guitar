/**
 ******************************************************************************
 * @file    solenoid_control.h
 * @brief   Solenoid control driver for guitar string vibration
 * @note    Provides PWM-based frequency control for up to 3 solenoids
 *
 *          Hardware:
 *          - Solenoid 0: TIM12 CH1 (PB14) → String E2 (82-123 Hz)
 *          - Solenoid 1: TIM12 CH2 (PB15) → String A2 (110-165 Hz)
 *          - Solenoid 2: TIM13 CH1 (PA6)  → String D3 (147-220 Hz)
 *
 *          Each solenoid is driven via N-channel MOSFET (logic-level, 3.3V)
 *          Timer clock: 64 MHz (APB1), Prescaler: 99 (÷100) → 640 kHz count freq
 ******************************************************************************
 */

#ifndef SOLENOID_CONTROL_H
#define SOLENOID_CONTROL_H

#include <stdint.h>

/**
 * @brief Solenoid IDs for 3-solenoid system
 * @note  Each solenoid corresponds to one of the low guitar strings
 */
typedef enum {
    SOLENOID_STRING_E2 = 0,  /* Low E string (82-123 Hz, frets 0-7) */
    SOLENOID_STRING_A2 = 1,  /* A string (110-165 Hz, frets 0-7) */
    SOLENOID_STRING_D3 = 2,  /* D string (147-220 Hz, frets 0-7) */
    SOLENOID_COUNT = 3       /* Total number of solenoids */
} SolenoidID_t;

/**
 * @brief Initialize all solenoid PWM channels
 * @note  Must be called after MX_TIM12_Init() and MX_TIM13_Init()
 * @return 0 on success, -1 on error (timer not initialized)
 */
int32_t Solenoid_Init(void);

/**
 * @brief Set frequency for a specific solenoid by string and fret
 * @param solenoid_id Solenoid index (0-2)
 * @param string String index (0-2: E2, A2, D3)
 * @param fret Fret index (0-7: open string to fret 7)
 * @return 0 on success, -1 if invalid parameters
 * @note  Does NOT start PWM - call Solenoid_Start() to activate
 * @note  Updates ARR and CCR registers atomically to avoid glitches
 */
int32_t Solenoid_SetFrequency(uint8_t solenoid_id, uint8_t string, uint8_t fret);

/**
 * @brief Set duty cycle for a specific solenoid
 * @param solenoid_id Solenoid index (0-2)
 * @param duty_percent Duty cycle 1-99% (0 = use default 50%)
 * @return 0 on success, -1 if invalid solenoid_id or duty_percent
 * @note  Default 50% duty cycle has been validated to work well at 82 Hz
 */
int32_t Solenoid_SetDutyCycle(uint8_t solenoid_id, uint8_t duty_percent);

/**
 * @brief Start PWM output for a specific solenoid
 * @param solenoid_id Solenoid index (0-2)
 * @note  Safe to call multiple times (idempotent)
 * @note  Solenoid will vibrate at last set frequency/duty cycle
 */
void Solenoid_Start(uint8_t solenoid_id);

/**
 * @brief Stop PWM output for a specific solenoid
 * @param solenoid_id Solenoid index (0-2)
 * @note  Safe to call multiple times (idempotent)
 * @note  Turns off MOSFET gate signal, stopping solenoid
 */
void Solenoid_Stop(uint8_t solenoid_id);

/**
 * @brief Stop all solenoids immediately (emergency stop)
 * @note  Equivalent to calling Solenoid_Stop() for all 3 solenoids
 */
void Solenoid_StopAll(void);

/**
 * @brief Get active solenoid mask (bit field)
 * @return Bit field where bit N = 1 if solenoid N is active
 * @note  Bit 0 = solenoid 0, bit 1 = solenoid 1, bit 2 = solenoid 2
 */
uint8_t Solenoid_GetActiveMask(void);

/**
 * @brief Get current frequency for a solenoid (diagnostics)
 * @param solenoid_id Solenoid index (0-2)
 * @return Frequency in Hz, or 0 if invalid solenoid_id
 * @note  Calculated from current ARR value: freq = 640000 / (ARR + 1)
 */
uint32_t Solenoid_GetFrequency(uint8_t solenoid_id);

#endif /* SOLENOID_CONTROL_H */

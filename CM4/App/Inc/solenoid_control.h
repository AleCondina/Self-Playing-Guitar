/**
 ******************************************************************************
 * @file    solenoid_control.h
 * @brief   Solenoid control driver for guitar string vibration
 * @note    Provides PWM-based frequency control for 6 solenoids (all strings)
 *
 *          Hardware (6 independent timers for independent frequencies):
 *          - Solenoid 0: TIM12 CH1 (PB14) → String E2 (82-123 Hz)
 *          - Solenoid 1: TIM14 CH1 (PA7)  → String A2 (110-165 Hz)
 *          - Solenoid 2: TIM13 CH1 (PA6)  → String D3 (147-220 Hz)
 *          - Solenoid 3: TIM15 CH1 (PE5)  → String G3 (196-294 Hz)
 *          - Solenoid 4: TIM16 CH1 (PB8)  → String B3 (247-370 Hz)
 *          - Solenoid 5: TIM17 CH1 (PB9)  → String E4 (330-494 Hz)
 *
 *          Each solenoid is driven via N-channel MOSFET (logic-level, 3.3V)
 *          Timer clock: 64 MHz (APB1), Prescaler: 99 (÷100) → 640 kHz count freq
 ******************************************************************************
 */

#ifndef SOLENOID_CONTROL_H
#define SOLENOID_CONTROL_H

#include <stdint.h>

/**
 * @brief Solenoid IDs for 6-solenoid system (all guitar strings)
 * @note  Each solenoid corresponds to one guitar string
 */
typedef enum {
    SOLENOID_STRING_E2 = 0,  /* Low E string (82-123 Hz, frets 0-7) */
    SOLENOID_STRING_A2 = 1,  /* A string (110-165 Hz, frets 0-7) */
    SOLENOID_STRING_D3 = 2,  /* D string (147-220 Hz, frets 0-7) */
    SOLENOID_STRING_G3 = 3,  /* G string (196-294 Hz, frets 0-7) */
    SOLENOID_STRING_B3 = 4,  /* B string (247-370 Hz, frets 0-7) */
    SOLENOID_STRING_E4 = 5,  /* High E string (330-494 Hz, frets 0-7) */
    SOLENOID_COUNT = 6       /* Total number of solenoids */
} SolenoidID_t;

/**
 * @brief Initialize all solenoid PWM channels
 * @note  Must be called after all MX_TIMxx_Init() functions
 * @return 0 on success, -1 on error (timer not initialized)
 */
int32_t Solenoid_Init(void);

/**
 * @brief Set frequency for a specific solenoid by string and fret
 * @param solenoid_id Solenoid index (0-5)
 * @param string String index (0-5: E2, A2, D3, G3, B3, E4)
 * @param fret Fret index (0-7: open string to fret 7)
 * @return 0 on success, -1 if invalid parameters
 * @note  Does NOT start PWM - call Solenoid_Start() to activate
 * @note  Updates ARR and CCR registers atomically to avoid glitches
 * @note  String mapping: 0=E2(low), 1=A2, 2=D3, 3=G3, 4=B3, 5=E4(high)
 */
int32_t Solenoid_SetFrequency(uint8_t solenoid_id, uint8_t string, uint8_t fret);

/**
 * @brief Set duty cycle for a specific solenoid
 * @param solenoid_id Solenoid index (0-5)
 * @param duty_percent Duty cycle 1-99% (0 = use default 50%)
 * @return 0 on success, -1 if invalid solenoid_id or duty_percent
 * @note  Default 50% duty cycle has been validated to work well at 82 Hz
 */
int32_t Solenoid_SetDutyCycle(uint8_t solenoid_id, uint8_t duty_percent);

/**
 * @brief Start PWM output for a specific solenoid
 * @param solenoid_id Solenoid index (0-5)
 * @note  Safe to call multiple times (idempotent)
 * @note  Solenoid will vibrate at last set frequency/duty cycle
 */
void Solenoid_Start(uint8_t solenoid_id);

/**
 * @brief Stop PWM output for a specific solenoid
 * @param solenoid_id Solenoid index (0-5)
 * @note  Safe to call multiple times (idempotent)
 * @note  Turns off MOSFET gate signal, stopping solenoid
 */
void Solenoid_Stop(uint8_t solenoid_id);

/**
 * @brief Stop all solenoids immediately (emergency stop)
 * @note  Equivalent to calling Solenoid_Stop() for all 6 solenoids
 */
void Solenoid_StopAll(void);

/**
 * @brief Get active solenoid mask (bit field)
 * @return Bit field where bit N = 1 if solenoid N is active
 * @note  Bits 0-5 correspond to solenoids 0-5
 */
uint8_t Solenoid_GetActiveMask(void);

/**
 * @brief Get current frequency for a solenoid (diagnostics)
 * @param solenoid_id Solenoid index (0-5)
 * @return Frequency in Hz, or 0 if invalid solenoid_id
 * @note  Calculated from current ARR value: freq = 640000 / (ARR + 1)
 */
uint32_t Solenoid_GetFrequency(uint8_t solenoid_id);

#endif /* SOLENOID_CONTROL_H */

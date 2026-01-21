/**
 ******************************************************************************
 * @file    application_config.h
 * @brief   Global application configuration constants
 * @note    Hardware-agnostic configuration shared by both cores
 ******************************************************************************
 */

#ifndef APPLICATION_CONFIG_H
#define APPLICATION_CONFIG_H

#include <stdint.h>

/* ===== Guitar Controller Configuration ===== */
#define NUM_STRINGS             6       /* Number of guitar strings */
#define NUM_MOTORS              6       /* One motor per string */
#define NUM_SOLENOIDS           3       /* Initial: 3 solenoids (expandable to 6) */

/* ===== Motor Control Parameters ===== */
#define MOTOR_MIN_STEP_FREQ     200     /* Minimum steps/sec */
#define MOTOR_MAX_STEP_FREQ     10000   /* Maximum steps/sec */
#define MOTOR_MAX_ACCEL         1000    /* Max acceleration (steps/sec²) */
#define MOTOR_STEPS_PER_REV     200     /* Stepper motor steps per revolution */

/* ===== Solenoid Control Parameters ===== */
#define SOLENOID_MIN_DURATION_MS    10  /* Minimum solenoid on-time */
#define SOLENOID_MAX_DURATION_MS    500 /* Maximum solenoid on-time */
#define SOLENOID_DEFAULT_DURATION_MS 50 /* Default solenoid duration */
#define SOLENOID_DEFAULT_DUTY_CYCLE  50 /* Default duty cycle (%) - validated at 82 Hz */
#define SOLENOID_MIN_FREQ_HZ         82 /* Minimum frequency: E2 (low E string) */
#define SOLENOID_MAX_FREQ_HZ        220 /* Maximum frequency: D3 fret 7 */

/* ===== Audio Processing Parameters ===== */
#define AUDIO_SAMPLE_RATE_HZ    44100   /* ADC sample rate */
#define AUDIO_BUFFER_SIZE       2048    /* Power of 2 for FFT */
#define AUDIO_FFT_SIZE          2048    /* FFT size (must match buffer) */

/* ===== Timing Parameters ===== */
#define CM4_CONTROL_LOOP_FREQ_HZ    1000    /* CM4 control loop: 1kHz (1ms) */
#define CM7_AUDIO_PROCESS_FREQ_HZ   100     /* CM7 pitch detection: 100Hz (10ms) */

/* ===== System Clock Configuration ===== */
#define SYSTEM_CLOCK_MHZ        480     /* CM7 system clock */
#define CM4_CLOCK_MHZ           240     /* CM4 clock */
#define TIM_CLOCK_MHZ           64      /* Timer peripheral clock (ACTUAL measured: 64 MHz, not 240) */

/* ===== Error Codes (bit field) ===== */
#define ERROR_MOTOR_STALL       (1U << 0)   /* Motor stall detected */
#define ERROR_LIMIT_SWITCH      (1U << 1)   /* Limit switch hit */
#define ERROR_OVER_TEMP         (1U << 2)   /* Over-temperature */
#define ERROR_COMM_TIMEOUT      (1U << 3)   /* Inter-core comm timeout */
#define ERROR_ADC_OVERRUN       (1U << 4)   /* ADC DMA overrun */
#define ERROR_INVALID_COMMAND   (1U << 5)   /* Invalid command received */

/* ===== Debug Configuration ===== */
#define DEBUG_UART_ENABLED      1       /* Enable UART debug output */
#define DEBUG_UART_BAUD         115200  /* UART baud rate */

/* ===== Version Info ===== */
#define FIRMWARE_VERSION_MAJOR  0
#define FIRMWARE_VERSION_MINOR  1
#define FIRMWARE_VERSION_PATCH  0

#endif /* APPLICATION_CONFIG_H */

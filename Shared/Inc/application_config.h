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
#define MOTOR_MAX_ACCEL         100000    /* Max acceleration (steps/sec²) */
#define MOTOR_STEPS_PER_REV     200     /* Stepper motor steps per revolution */

/* ===== Solenoid Control Parameters ===== */
#define SOLENOID_MIN_DURATION_MS    10  /* Minimum solenoid on-time */
#define SOLENOID_MAX_DURATION_MS    500 /* Maximum solenoid on-time */
#define SOLENOID_DEFAULT_DURATION_MS 50 /* Default solenoid duration */
#define SOLENOID_DEFAULT_DUTY_CYCLE  50 /* Default duty cycle (%) - validated at 82 Hz */
#define SOLENOID_MIN_FREQ_HZ         82 /* Minimum frequency: E2 (low E string) */
#define SOLENOID_MAX_FREQ_HZ        220 /* Maximum frequency: D3 fret 7 */ //TODO: modify possibly

/* ===== Audio Processing Parameters ===== */   //TODO: check and modify for FFT
#define AUDIO_SAMPLE_RATE_HZ    6000   /* ADC sample rate, technically its at 5999.812 since the register counts to 10666+1*/ 
#define FFT_SIZE                2048    //2048 for 341ms sampling time, halved by using an overlapping array to 170ms
#define FFT_HALF_SIZE           (FFT_SIZE / 2)
#define AUDIO_BUFFER_SIZE       FFT_SIZE   // same size as FFT for overlapping samples at every half-buffer
#define MAX_HARMONICS           8
#define FFT_RESOLUTION          (AUDIO_SAMPLE_RATE_HZ / FFT_SIZE)

/* ===== Timing Parameters ===== */
#define CM4_CONTROL_LOOP_FREQ_HZ    1000    /* CM4 control loop: 1kHz (1ms) */
#define CM7_AUDIO_PROCESS_FREQ_HZ   100     /* CM7 pitch detection: 100Hz (10ms) */

/* ===== System Clock Configuration ===== */
#define SYSTEM_CLOCK_MHZ        396     /* CM7 system clock */
#define CM4_CLOCK_MHZ           198     /* CM4 clock */
#define TIM_CLOCK_MHZ           CM4_CLOCK_MHZ      /* Timer peripheral clock (fixed to cm4 speed 240 Hz) */
#define TIM_PRESCALED_CLK_HZ    2200000  /*Timer clk prescaled by /(1+89) */
#define TIMER_CALIBRATION_FACTOR   1.005245182f  /* Corrects timer clock drift (~0.52% fast) */
 
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

/* ===== Universal Guitar Frequencies (Hz) ===== */
#define E2_HZ   82.41f
#define F2_HZ   87.31f
#define Fs2_HZ  92.50f  /* F#2 / Gb2 */
#define G2_HZ   98.00f
#define Gs2_HZ  103.83f /* G#2 / Ab2 */

#define A2_HZ   110.00f
#define As2_HZ  116.54f /* A#2 / Bb2 */
#define B2_HZ   123.47f
#define C3_HZ   130.81f
#define Cs3_HZ  138.59f /* C#3 / Db3 */

#define D3_HZ   146.83f
#define Ds3_HZ  155.56f /* D#3 / Eb3 */
#define E3_HZ   164.81f
#define F3_HZ   174.61f
#define Fs3_HZ  185.00f /* F#3 / Gb3 */

#define G3_HZ   196.00f
#define Gs3_HZ  207.65f /* G#3 / Ab3 */
#define A3_HZ   220.00f
#define As3_HZ  233.08f /* A#3 / Bb3 */

#define B3_HZ   246.94f
#define C4_HZ   261.63f
#define Cs4_HZ  277.18f /* C#4 / Db4 */
#define D4_HZ   293.66f
#define Ds4_HZ  311.13f /* D#4 / Eb4 */

#define E4_HZ   329.63f
#define F4_HZ   349.23f

#endif /* APPLICATION_CONFIG_H */

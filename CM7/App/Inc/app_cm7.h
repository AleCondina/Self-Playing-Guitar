/**
 ******************************************************************************
 * @file    app_cm7.h
 * @brief   CM7 application main header
 * @note    Top-level application entry point for CM7
 ******************************************************************************
 */

#ifndef APP_CM7_H
#define APP_CM7_H

#include <stdint.h>

/**
 * @brief CM7 application main function
 * @note  Called from main.c after all initialization is complete
 * @note  This function never returns (infinite loop)
 */
void App_CM7_Main(void);
void App_RequestCalibration(uint8_t string_mask);
void App_RequestPlay(void);
void App_RequestPlaySong(uint8_t song_id);
void App_RequestLutBuild(uint8_t string_mask);
void App_RequestTune(uint8_t string_mask);

/* Debug flags — toggled via 'debug' serial command */
#define DEBUG_FLAG_HOP_TIMER   (1u << 0)  /* Print hop interval & proc time every 5s */
#define DEBUG_FLAG_NOISE_FLOOR (1u << 1)  /* Print per-bin noise floor samples */
extern uint8_t app_debug_flags;

#endif /* APP_CM7_H */

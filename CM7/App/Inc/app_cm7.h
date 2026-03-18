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
void App_RequestLutBuild(uint8_t string_mask);
void App_RequestTune(uint8_t string_mask);

#endif /* APP_CM7_H */

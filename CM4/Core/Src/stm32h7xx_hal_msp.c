/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief TIM PWM MSP Initialization
 * @note  Enables timer clocks and configures GPIO pins for PWM output
 * @note  Called automatically by HAL_TIM_PWM_Init()
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(htim->Instance == TIM12)
    {
        /* Enable TIM12 clock (APB1 domain) */
        __HAL_RCC_TIM12_CLK_ENABLE();

        /* Enable GPIOB clock for PWM output pins */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**
         * TIM12 GPIO Configuration:
         * PB14 -> TIM12_CH1 (Solenoid 0: E2 string)
         * PB15 -> TIM12_CH2 (Solenoid 1: A2 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* Alternate function push-pull */
        GPIO_InitStruct.Pull = GPIO_NOPULL;             /* No pull-up/pull-down */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (82-165 Hz PWM) */
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;     /* AF2 = TIM12 */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM13)
    {
        /* Enable TIM13 clock (APB1 domain) */
        __HAL_RCC_TIM13_CLK_ENABLE();

        /* Enable GPIOA clock for PWM output pin */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**
         * TIM13 GPIO Configuration:
         * PA6 -> TIM13_CH1 (Solenoid 2: D3 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* Alternate function push-pull */
        GPIO_InitStruct.Pull = GPIO_NOPULL;             /* No pull-up/pull-down */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (147-220 Hz PWM) */
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;     /* AF9 = TIM13 */
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
 * @brief TIM PWM MSP De-Initialization
 * @note  Disables timer clocks and de-initializes GPIO pins
 * @note  Called automatically by HAL_TIM_PWM_DeInit()
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM12)
    {
        /* Disable TIM12 clock */
        __HAL_RCC_TIM12_CLK_DISABLE();

        /* De-initialize GPIO pins */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14 | GPIO_PIN_15);
    }
    else if(htim->Instance == TIM13)
    {
        /* Disable TIM13 clock */
        __HAL_RCC_TIM13_CLK_DISABLE();

        /* De-initialize GPIO pin */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    }
}

/* USER CODE END 1 */

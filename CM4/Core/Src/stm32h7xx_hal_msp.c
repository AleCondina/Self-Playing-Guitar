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

        /* Enable GPIOB clock for PWM output pin */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**
         * TIM12 GPIO Configuration:
         * PB14 -> TIM12_CH1 (Solenoid 0: E2 string - Low E)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* Alternate function push-pull */
        GPIO_InitStruct.Pull = GPIO_NOPULL;             /* No pull-up/pull-down */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (82 Hz PWM) */
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
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (147 Hz PWM) */
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;     /* AF9 = TIM13 */
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM14)
    {
        /* Enable TIM14 clock (APB1 domain) */
        __HAL_RCC_TIM14_CLK_ENABLE();

        /* Enable GPIOA clock for PWM output pin */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**
         * TIM14 GPIO Configuration:
         * PA7 -> TIM14_CH1 (Solenoid 1: A2 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* Alternate function push-pull */
        GPIO_InitStruct.Pull = GPIO_NOPULL;             /* No pull-up/pull-down */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (110 Hz PWM) */
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;     /* AF9 = TIM14 */
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM15)
    {
        /* Enable TIM15 clock (APB2 domain) */
        __HAL_RCC_TIM15_CLK_ENABLE();

        /* Enable GPIOE clock for PWM output pin */
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /**
         * TIM15 GPIO Configuration:
         * PE5 -> TIM15_CH1 (Solenoid 3: G3 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* Alternate function push-pull */
        GPIO_InitStruct.Pull = GPIO_NOPULL;             /* No pull-up/pull-down */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (196 Hz PWM) */
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;     /* AF4 = TIM15 */
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM16)
    {
        /* Enable TIM16 clock (APB2 domain) */
        __HAL_RCC_TIM16_CLK_ENABLE();

        /* Enable GPIOB clock for PWM output pin */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**
         * TIM16 GPIO Configuration:
         * PB8 -> TIM16_CH1 (Solenoid 4: B3 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* Alternate function push-pull */
        GPIO_InitStruct.Pull = GPIO_NOPULL;             /* No pull-up/pull-down */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (247 Hz PWM) */
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;     /* AF1 = TIM16 */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM17)
    {
        /* Enable TIM17 clock (APB2 domain) */
        __HAL_RCC_TIM17_CLK_ENABLE();

        /* Enable GPIOB clock for PWM output pin */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**
         * TIM17 GPIO Configuration:
         * PB9 -> TIM17_CH1 (Solenoid 5: E4 string - High E)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;         /* Alternate function push-pull */
        GPIO_InitStruct.Pull = GPIO_NOPULL;             /* No pull-up/pull-down */
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    /* Low speed (330 Hz PWM) */
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM17;     /* AF1 = TIM17 */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

        /* De-initialize GPIO pin */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);
    }
    else if(htim->Instance == TIM13)
    {
        /* Disable TIM13 clock */
        __HAL_RCC_TIM13_CLK_DISABLE();

        /* De-initialize GPIO pin (PA6) */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    }
    else if(htim->Instance == TIM14)
    {
        /* Disable TIM14 clock */
        __HAL_RCC_TIM14_CLK_DISABLE();

        /* De-initialize GPIO pin (PA7) */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
    }
    else if(htim->Instance == TIM15)
    {
        /* Disable TIM15 clock */
        __HAL_RCC_TIM15_CLK_DISABLE();

        /* De-initialize GPIO pin */
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_5);
    }
    else if(htim->Instance == TIM16)
    {
        /* Disable TIM16 clock */
        __HAL_RCC_TIM16_CLK_DISABLE();

        /* De-initialize GPIO pin (PB8) */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
    }
    else if(htim->Instance == TIM17)
    {
        /* Disable TIM17 clock */
        __HAL_RCC_TIM17_CLK_DISABLE();

        /* De-initialize GPIO pin (PB9) */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
    }
}

/* USER CODE END 1 */

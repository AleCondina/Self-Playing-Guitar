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
 *
 * PCB solenoid timer assignments:
 *   TIM13 CH1 → PF8  (Sol 0: E2) AF9, D2 domain
 *   TIM15 CH1 → PE5  (Sol 1: A2) AF4, D1 domain
 *   TIM17 CH1 → PB9  (Sol 2: D3) AF1, D1 domain
 *   TIM16 CH1 → PB8  (Sol 3: G3) AF1, D1 domain
 *   TIM4  CH2 → PB7  (Sol 4: B3) AF2, D2 domain
 *   TIM5  CH4 → PI0  (Sol 5: E4) AF2, D2 domain
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(htim->Instance == TIM4)
    {
        /* Enable TIM4 clock (APB1 / D2 domain) */
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**
         * TIM4 GPIO Configuration:
         * PB7 -> TIM4_CH2 (Solenoid 4: B3 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM5)
    {
        /* Enable TIM5 clock (APB1 / D2 domain) */
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOI_CLK_ENABLE();

        /**
         * TIM5 GPIO Configuration:
         * PI0 -> TIM5_CH4 (Solenoid 5: E4 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM13)
    {
        /* Enable TIM13 clock (APB1 / D2 domain) */
        __HAL_RCC_TIM13_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();

        /**
         * TIM13 GPIO Configuration:
         * PF8 -> TIM13_CH1 (Solenoid 0: E2 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM15)
    {
        /* Enable TIM15 clock (APB2 / D1 domain — accessible from CM4) */
        __HAL_RCC_TIM15_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /**
         * TIM15 GPIO Configuration:
         * PE5 -> TIM15_CH1 (Solenoid 1: A2 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM16)
    {
        /* Enable TIM16 clock (APB2 / D1 domain — accessible from CM4) */
        __HAL_RCC_TIM16_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**
         * TIM16 GPIO Configuration:
         * PB8 -> TIM16_CH1 (Solenoid 3: G3 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if(htim->Instance == TIM17)
    {
        /* Enable TIM17 clock (APB2 / D1 domain — accessible from CM4) */
        __HAL_RCC_TIM17_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**
         * TIM17 GPIO Configuration:
         * PB9 -> TIM17_CH1 (Solenoid 2: D3 string)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM17;
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
    if(htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
    }
    else if(htim->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOI, GPIO_PIN_0);
    }
    else if(htim->Instance == TIM13)
    {
        __HAL_RCC_TIM13_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_8);
    }
    else if(htim->Instance == TIM15)
    {
        __HAL_RCC_TIM15_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_5);
    }
    else if(htim->Instance == TIM16)
    {
        __HAL_RCC_TIM16_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
    }
    else if(htim->Instance == TIM17)
    {
        __HAL_RCC_TIM17_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
    }
}

/* USER CODE END 1 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "application_config.h"
#include "shared_memory.h"
#include "inter_core_cm4.h"
#include "app_cm4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Timer handles for solenoid PWM control (6 timers, one per string) */
TIM_HandleTypeDef htim4;    /* Solenoid 4: B3 string (PB7, TIM4_CH2)  — D2 domain */
TIM_HandleTypeDef htim5;    /* Solenoid 5: E4 string (PI0, TIM5_CH4)  — D2 domain */
TIM_HandleTypeDef htim13;   /* Solenoid 0: E2 string (PF8, TIM13_CH1) — D2 domain */
TIM_HandleTypeDef htim15;   /* Solenoid 1: A2 string (PE5, TIM15_CH1) — D1 domain */
TIM_HandleTypeDef htim16;   /* Solenoid 3: G3 string (PB8, TIM16_CH1) — D1 domain */
TIM_HandleTypeDef htim17;   /* Solenoid 2: D3 string (PB9, TIM17_CH1) — D1 domain */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_BDMA_Init(void);
/* USER CODE BEGIN PFP */

static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Initialize inter-core communication */
  InterCore_CM4_Init();

  /* Verify shared memory is accessible */
  g_shared_memory.cm4_heartbeat = 0;
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_BDMA_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize timers for solenoid PWM control (6 timers, one per string) */
  MX_TIM13_Init();  /* Solenoid 0: E2 (PF8)  */
  MX_TIM15_Init();  /* Solenoid 1: A2 (PE5)  */
  MX_TIM17_Init();  /* Solenoid 2: D3 (PB9)  */
  MX_TIM16_Init();  /* Solenoid 3: G3 (PB8)  */
  MX_TIM4_Init();   /* Solenoid 4: B3 (PB7)  */
  MX_TIM5_Init();   /* Solenoid 5: E4 (PI0)  */

  /* Call application main (never returns) */
  App_CM4_Main();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
 * @brief TIM13 Initialization Function
 * @note  Solenoid 0: E2 string on PF8 (TIM13_CH1, AF9, D2 domain)
 */
static void MX_TIM13_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim13.Instance = TIM13;
    htim13.Init.Prescaler = 89;  /* 198 MHz / 90 = 2.2 MHz count frequency */
    htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim13.Init.Period = (uint16_t)(((TIM_PRESCALED_CLK_HZ / E2_HZ) - 1.0f) * TIMER_CALIBRATION_FACTOR + 0.5f);
    htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim13.Init.Period + 1) / 2;  /* 50% duty cycle */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM15 Initialization Function
 * @note  Solenoid 1: A2 string on PE5 (TIM15_CH1, AF4, D1 domain)
 */
static void MX_TIM15_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 89;  /* 198 MHz / 90 = 2.2 MHz count frequency */
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Period = (uint16_t)(((TIM_PRESCALED_CLK_HZ / A2_HZ) - 1.0f) * TIMER_CALIBRATION_FACTOR + 0.5f);
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim15.Init.Period + 1) / 2;  /* 50% duty cycle */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM17 Initialization Function
 * @note  Solenoid 2: D3 string on PB9 (TIM17_CH1, AF1, D1 domain)
 */
static void MX_TIM17_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 89;  /* 198 MHz / 90 = 2.2 MHz count frequency */
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = (uint16_t)(((TIM_PRESCALED_CLK_HZ / D3_HZ) - 1.0f) * TIMER_CALIBRATION_FACTOR + 0.5f);
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim17.Init.Period + 1) / 2;  /* 50% duty cycle */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM16 Initialization Function
 * @note  Solenoid 3: G3 string on PB8 (TIM16_CH1, AF1, D1 domain)
 */
static void MX_TIM16_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 89;  /* 198 MHz / 90 = 2.2 MHz count frequency */
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = (uint16_t)(((TIM_PRESCALED_CLK_HZ / G3_HZ) - 1.0f) * TIMER_CALIBRATION_FACTOR + 0.5f);
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim16.Init.Period + 1) / 2;  /* 50% duty cycle */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM4 Initialization Function
 * @note  Solenoid 4: B3 string on PB7 (TIM4_CH2, AF2, D2 domain)
 */
static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 89;  /* 198 MHz / 90 = 2.2 MHz count frequency */
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = (uint16_t)(((TIM_PRESCALED_CLK_HZ / B3_HZ) - 1.0f) * TIMER_CALIBRATION_FACTOR + 0.5f);
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim4.Init.Period + 1) / 2;  /* 50% duty cycle */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM5 Initialization Function
 * @note  Solenoid 5: E4 string on PI0 (TIM5_CH4, AF2, D2 domain)
 */
static void MX_TIM5_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 89;  /* 198 MHz / 90 = 2.2 MHz count frequency */
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = (uint16_t)(((TIM_PRESCALED_CLK_HZ / E4_HZ) - 1.0f) * TIMER_CALIBRATION_FACTOR + 0.5f);
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim5.Init.Period + 1) / 2;  /* 50% duty cycle */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

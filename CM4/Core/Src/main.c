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

/* Timer handles for solenoid PWM control (6 independent timers) */
TIM_HandleTypeDef htim1;   /* Solenoid 0: E2 string (PE9) */
TIM_HandleTypeDef htim2;   /* Solenoid 2: D3 string (PA5) */
TIM_HandleTypeDef htim3;   /* Solenoid 1: A2 string (PB5) */
TIM_HandleTypeDef htim4;   /* Solenoid 3: G3 string (PB6) */
TIM_HandleTypeDef htim16;  /* Solenoid 4: B3 string (PB8) */
TIM_HandleTypeDef htim17;  /* Solenoid 5: E4 string (PB9) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_BDMA_Init(void);
/* USER CODE BEGIN PFP */

static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
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

  /* Initialize timers for solenoid PWM control (6 independent timers) */
  MX_TIM1_Init();   /* Solenoid 0: E2 */
  MX_TIM2_Init();   /* Solenoid 2: D3 */
  MX_TIM3_Init();   /* Solenoid 1: A2 */
  MX_TIM4_Init();   /* Solenoid 3: G3 */
  MX_TIM16_Init();  /* Solenoid 4: B3 */
  MX_TIM17_Init();  /* Solenoid 5: E4 */

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
 * @brief TIM1 Initialization Function
 * @note  Solenoid 0: E2 string on PE9
 * @note  Uses TIM1 channel 1 for an independent PWM output
 */
static void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 99;  /* 200 MHz / 100 = 2.0 MHz count frequency */
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = (uint16_t)(((TIM_CLOCK_MHZ * (10000)) / E2_HZ) - 0.5f);   /* Default: 82.41 Hz (E2 open string) */
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure PWM Channel 1: Solenoid 0 (E2 string) */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim1.Init.Period / 2;  /* 50% duty cycle (Period / 2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 * @note  Solenoid 2: D3 string on PA5
 * @note  Uses TIM2 channel 1 for an independent PWM output
 */
static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 99;  /* 200 MHz / 100 = 2.0 MHz count frequency */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (uint16_t)(((TIM_CLOCK_MHZ * (10000)) / D3_HZ) - 0.5f);   /* Default: 146.83 Hz (D3 open string) */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure PWM Channel 1: Solenoid 2 (D3 string) */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim2.Init.Period / 2;  /* 50% duty cycle (Period / 2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM3 Initialization Function
 * @note  Solenoid 1: A2 string on PB5
 * @note  Uses TIM3 channel 2 for an independent PWM output on the requested pin
 */
static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 99;  /* 200 MHz / 100 = 2.0 MHz count frequency */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = (uint16_t)(((TIM_CLOCK_MHZ * (10000)) / A2_HZ) - 0.5f);   /* Default: 110.00 Hz (A2 open string) */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure PWM Channel 2: Solenoid 1 (A2 string) */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim3.Init.Period / 2;  /* 50% duty cycle (Period / 2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM4 Initialization Function
 * @note  Solenoid 3: G3 string on PB6
 * @note  Uses TIM4 channel 1 for an independent PWM output
 */
static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 99;  /* 200 MHz / 100 = 2.0 MHz count frequency */
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = (uint16_t)(((TIM_CLOCK_MHZ * (10000)) / G3_HZ) - 0.5f);   /* Default: 196.00 Hz (G3 open string) */
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure PWM Channel 1: Solenoid 3 (G3 string) */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim4.Init.Period / 2;  /* 50% duty cycle (Period / 2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM16 Initialization Function
 * @note  Solenoid 4: B3 string on PB8
 * @note  PWM frequency controlled by ARR, duty cycle by CCR
 */
static void MX_TIM16_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 99;  /* 200 MHz / 100 = 2.0 MHz count frequency */
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = (uint16_t)(((TIM_CLOCK_MHZ * (10000)) / B3_HZ) - 0.5f);   /* Default: 246.94 Hz (B3 open string) */
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure PWM Channel 1: Solenoid 4 (B3 string) */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim16.Init.Period / 2;  /* 50% duty cycle (Period / 2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM17 Initialization Function
 * @note  Solenoid 5: E4 string (High E) on PB9
 * @note  PWM frequency controlled by ARR, duty cycle by CCR
 */
static void MX_TIM17_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 99;  /* 200 MHz / 100 = 2.0 MHz count frequency */
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = (uint16_t)(((TIM_CLOCK_MHZ * (10000)) / E4_HZ) - 0.5f);   /* Default: 329.63 Hz (E4 open string) */
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure PWM Channel 1: Solenoid 5 (E4 string - High E) */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim17.Init.Period / 2;  /* 50% duty cycle (Period / 2) */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

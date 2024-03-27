/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ExtiConfig(void);
static void APP_PWR_EnterHibernateMode(void);

/**
  * @brief  Application Entry Function.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();
  
  /* System clock configuration */
  APP_SystemClockConfig(); 

  /* Initialize LED */
  BSP_LED_Init(LED_TK1);

  /* Initialization button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
  
  /* Configure external events */
  APP_ExtiConfig();
  
  /* Turn on the LED */
  BSP_LED_On(LED_TK1);

  /*Wait for the button to be pressed*/
  while (BSP_PB_GetState(BUTTON_KEY) == 0)
  {
  }
  
  /* Turn off LED */
  BSP_LED_Off(LED_TK1);
  
  /* Systick interrupt shutdown to prevent systick interrupt wake-up */
  HAL_SuspendTick();
  
  /* Configure and enter hibernate mode */
  APP_PWR_EnterHibernateMode(); 
  
  /* Systick interrupt enable */
  HAL_ResumeTick();
  
  /* infinite loop */
  while (1)
  {
    /* LED flashing */
    BSP_LED_Toggle(LED_TK1);
    HAL_Delay(200);
  }
}

/**
  * @brief  System clock configuration function
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Oscillator configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE; /* Select oscillator HSE, HSI, LSI, LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                           /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                           /* HSI 1 frequency division */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* Configure HSI clock 24MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                          /* Close HSE */
  /* RCC_OscInitStruct.HSEFreq  = RCC_HSE_6_8MHz; */                 /* HSE select 6-8MHz */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                          /* Close LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                          /* Close LSE */
  /* RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/            /* LSE medium drive capability */
  /* Configure oscillator */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Choose to configure clock HCLK, SYSCLK, PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS; /* Select HSISYS as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* AHB clock 1 division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         /* APB clock 1 division */
  /* Configure clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  Event Configuration Function.
  * @param  None
  * @retval None
  */
static void APP_ExtiConfig(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();                  /* Enable GPIOA clock */
  GPIO_InitStruct.Mode  = GPIO_MODE_EVT_FALLING; /* GPIO mode is a falling edge event */
  GPIO_InitStruct.Pull  = GPIO_PULLUP;           /* pull up */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* The speed is high */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  Configuration function for hibernate mode.
  * @param  None
  * @retval None
  */
static void APP_PWR_EnterHibernateMode(void)
{
  PWR_StopModeConfigTypeDef PWR_StopModeConfigStruct = {0};
  PWR_StopModeConfigStruct.WakeUpHsiEnableTime = PWR_WAKEUP_HSIEN_AFTER_MR;       /* Wait for MR to become stable */
  PWR_StopModeConfigStruct.SramRetentionVolt = PWR_SRAM_RETENTION_VOLT_HIBERNATE; /* Setting the SRAM voltage in hibernate mode */
  PWR_StopModeConfigStruct.FlashDelay = PWR_WAKEUP_FLASH_DELAY_5US;               /* Delay 5us */
  /* Configure hibernate mode */
  if (HAL_PWR_ConfigStopMode(&PWR_StopModeConfigStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  HAL_PWR_EnterSTOPMode(PWR_DEEPLOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);         /* Entering hibernate mode */
}

/**
  * @brief  Error executing function.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* Users can add their own printing information as needed,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

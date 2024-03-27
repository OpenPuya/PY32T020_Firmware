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
RTC_HandleTypeDef RTCinit;
RTC_TimeTypeDef RTCtime;
RTC_AlarmTypeDef RTC_AlarmStruct;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_RtcInit(void);
static void APP_RtcSetAlarm_IT(void);
static void APP_PWR_EnterStopMode(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* Enable LSE clock */
  APP_SystemClockConfig();
  
  /* Initialize LED */
  BSP_LED_Init(LED_TK1);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);

  /* Initialize the debug UART (used for printf) */
  BSP_UART_Config();
  
  /* RTC initialization */
  APP_RtcInit();
  
  /* Turn on LED */
  BSP_LED_On(LED_TK1);
  
  /* Wait for the button to be pressed */
  while (BSP_PB_GetState(BUTTON_KEY) == 0)
  {
  }
  
  /* Get current RTC time in binary format */
  HAL_RTC_GetTime(&RTCinit,&RTCtime,RTC_FORMAT_BIN);
  
  /* Configure the time of the first alarm in binary format */
  RTC_AlarmStruct.AlarmTime = RTCtime;
  RTC_AlarmStruct.AlarmTime.Seconds += 1;
  HAL_RTC_SetAlarm_IT(&RTCinit, &RTC_AlarmStruct, RTC_FORMAT_BIN);
  
   /* Turn off LED */
  BSP_LED_Off(LED_TK1);
  
  while (1)
  {
    /* Configure and enter stop mode */
    APP_PWR_EnterStopMode();
    
    /* Wait for synchronization */
    HAL_RTC_WaitForSynchro(&RTCinit);
    
    /* Set RTC alarm interrupt */
    APP_RtcSetAlarm_IT();
  }
}

/**
  * @brief  RTC initialization function, sets RTC to January 1, 2022, Saturday, 00:00:00
  * @param  None
  * @retval None
  */
static void APP_RtcInit(void)
{
  RTC_TimeTypeDef Timeinit;
  
  /* RTC initialization */
  RTCinit.Instance = RTC;                               /* Select RTC */
  RTCinit.Init.AsynchPrediv = RTC_AUTO_1_SECOND;        /* Automatic calculation of RTC's 1-second time base */
  /*2022-1-1-00:00:00*/
  RTCinit.DateToUpdate.Year = 22;                       /* Year 22 */
  RTCinit.DateToUpdate.Month = RTC_MONTH_JANUARY;       /* January */
  RTCinit.DateToUpdate.Date = 1;                        /* 1st day */
  RTCinit.DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;  /* Saturday */
  Timeinit.Hours = 0x00;                                /* 0 hours */
  Timeinit.Minutes = 0x00;                              /* 0 minutes */
  Timeinit.Seconds = 0x00;                              /* 0 seconds */
  
  /* RTC deinitialization */
  HAL_RTC_DeInit(&RTCinit);
  
  /* RTC initialization */
  HAL_RTC_Init(&RTCinit);
  
  /* Set RTC current time in binary format */
  HAL_RTC_SetTime(&RTCinit, &Timeinit, RTC_FORMAT_BIN);
}

/**
  * @brief  Set RTC alarm interrupt
  * @retval None
  */
static void APP_RtcSetAlarm_IT(void)
{
  /* Get current alarm time in binary format */
  HAL_RTC_GetAlarm(&RTCinit, &RTC_AlarmStruct, RTC_FORMAT_BIN);
  
  /* Update alarm time in binary format */
  RTC_AlarmStruct.AlarmTime.Seconds += 1;
  HAL_RTC_SetAlarm_IT(&RTCinit, &RTC_AlarmStruct, RTC_FORMAT_BIN);
}

/**
  * @brief  RTC alarm event callback function, LED turns on when exiting low-power mode
  * @param  hrtc：RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Toggle LED */
  BSP_LED_Toggle(LED_TK1);
  
  /* Print the current alarm time */
  printf("%02d:%02d:%02d\r\n", RTC_AlarmStruct.AlarmTime.Hours , RTC_AlarmStruct.AlarmTime.Minutes, RTC_AlarmStruct.AlarmTime.Seconds);
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
  /* RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/            /* LSE  drive capability */
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
  * @brief  Configuration function for stop mode.
  * @param  None
  * @retval None
  */
static void APP_PWR_EnterStopMode(void)
{
  PWR_StopModeConfigTypeDef PWR_StopModeConfigStruct = {0};
  PWR_StopModeConfigStruct.WakeUpHsiEnableTime = PWR_WAKEUP_HSIEN_AFTER_MR;  /* Wait for MR to become stable */
  PWR_StopModeConfigStruct.SramRetentionVolt = PWR_SRAM_RETENTION_VOLT_STOP; /* Setting the SRAM voltage in stop mode */
  PWR_StopModeConfigStruct.FlashDelay = PWR_WAKEUP_FLASH_DELAY_5US;          /* Delay 5us */
  /* Configure stop mode */
  if (HAL_PWR_ConfigStopMode(&PWR_StopModeConfigStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);        /* Entering stop mode */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
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
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

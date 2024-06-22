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
ADC_HandleTypeDef             ADCHandle;
ADC_ChannelConfTypeDef        sConfig;
uint32_t                      adc_value;
uint32_t                      VrefBuf_Value;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ADCConfig(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */ 
  HAL_Init();
  
  APP_SystemClockConfig();

  /* Initialize LED */
  BSP_LED_Init(LED_TK1);

  /* Initialize UART */
  BSP_UART_Config();

  /* Configure ADC */
  APP_ADCConfig();
  
  while (1)
  {
    HAL_ADC_Start(&ADCHandle);                             /* Start ADC Conversion */
    HAL_ADC_PollForConversion(&ADCHandle, 1000000);        /* Polling for ADC Conversion */
    adc_value = HAL_ADC_GetValue(&ADCHandle);              /* Get ADC Value */
    VrefBuf_Value = (4095 * 1200) / adc_value;                /* Calculate the data */
    
    /* print VrefBuf value */
    printf("VrefBuf:%umV\r\n",(unsigned int)VrefBuf_Value);
    
    HAL_Delay(1000);
  }
}

/**
  * @brief  ADC configuration function
  * @param  None
  * @retval None
  */
static void APP_ADCConfig(void)
{
  ADCHandle.Instance = ADC1;
  
  ADCHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;                /* Set ADC clock*/
  ADCHandle.Init.Resolution            = ADC_RESOLUTION_12B;                      /* 12-bit resolution for converted data */
  ADCHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;                     /* Right-alignment for converted data */
  ADCHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;              /* Scan sequence direction: forward */
  ADCHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;                     /* Single Conversion*/
  ADCHandle.Init.LowPowerAutoWait      = DISABLE;                                 /* Auto-delayed conversion feature disabled */
  ADCHandle.Init.ContinuousConvMode    = DISABLE;                                 /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  ADCHandle.Init.DiscontinuousConvMode = DISABLE;                                 /* Disable discontinuous mode */
  ADCHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;                      /* Software start to trig the 1st conversion manually, without external event */
  ADCHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;           /* Parameter discarded because software trigger chosen */
  ADCHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;                /* DR register is overwritten with the last conversion result in case of overrun */
  ADCHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_239CYCLES_5;              /* Channel sampling time is 239.5 ADC clock cycles */
  if (HAL_ADC_Init(&ADCHandle) != HAL_OK)                                         /* ADC initialization */
  {
    APP_ErrorHandler();
  }

  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;                            /* Set the rank for the ADC channel order */
  sConfig.Channel      = ADC_CHANNEL_VREFINT;                                /* ADC channel selection */
  if (HAL_ADC_ConfigChannel(&ADCHandle, &sConfig) != HAL_OK)                      /* Configure ADC channels */
  {
    APP_ErrorHandler();
  }
  
  /* Configure VrefBuf 2.5V */  
  HAL_ADC_ConfigVrefBuf(&ADCHandle,ADC_VREFBUF_2P5V);
  
  if (HAL_ADCEx_Calibration_Start(&ADCHandle) != HAL_OK)                          /* ADC Calibration */
  {
    APP_ErrorHandler();
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


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                           /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                           /* HSI 1 division */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* Configure HSI output clock as 24MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                          /* Close HSE */
  /* RCC_OscInitStruct.HSEFreq  = RCC_HSE_6_8MHz; */                 /* HSE select 6-8MHz */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                          /* Disable LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                          /* Disable LSE */
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
  /* Configure OSC */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Configure Clock */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS; /* Select HSISYS as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* AHB clock 1 division */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         /* APB clock 1 division */
  /* Configure Clock */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
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

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
COMP_HandleTypeDef  hcomp1;
PWR_StopModeConfigTypeDef PWRStopModeConfig = {0};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_RccInit(void);
static void APP_CompInit(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();       
  
  /* System clock configuration */
  APP_SystemClockConfig();
  
  /* Initialization button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
  
  /* Initialize LED */
  BSP_LED_Init(LED_TK1);  

  /* Clock setting initialization */
  APP_RccInit();

  /* Initialize COMP */
  APP_CompInit();
  
  /* COMP1 Start */
  HAL_COMP_Start(&hcomp1);
  
  BSP_LED_On(LED_TK1);

  /* Wait for the button to be pressed */
  while(BSP_PB_GetState(BUTTON_USER) != 1)
  {
  }
  
  BSP_LED_Off(LED_TK1);
  
  /* Turn off Systick interrupt */
  HAL_SuspendTick(); 
  
  PWRStopModeConfig.WakeUpHsiEnableTime = PWR_WAKEUP_HSIEN_AFTER_MR;  /* Wait for MR to become stable */
  PWRStopModeConfig.SramRetentionVolt = PWR_SRAM_RETENTION_VOLT_STOP; /* Setting the SRAM voltage in stop mode */
  PWRStopModeConfig.FlashDelay = PWR_WAKEUP_FLASH_DELAY_5US;          /* Delay 5us */
  /* Configure stop mode */
  if (HAL_PWR_ConfigStopMode(&PWRStopModeConfig) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Entering STOP mode */
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
  
  /* Enable Systick interrupt */
  HAL_ResumeTick();

  BSP_LED_On(LED_TK1);
  HAL_Delay(1000);
  while (1)
  {
    BSP_LED_Toggle(LED_TK1); 
    HAL_Delay(200);    
  }
}

/**
  * @brief  Comparator clock switching function.
  * @param  None
  * @retval None
  */
static void APP_RccInit(void)
{                    
  RCC_OscInitTypeDef RCCCONF={0};
  RCC_PeriphCLKInitTypeDef COMPRCC={0};
  
  RCCCONF.OscillatorType = RCC_OSCILLATORTYPE_LSI;        /* RCC uses internal LSI */
  RCCCONF.LSIState = RCC_LSI_ON;                          /* Enable LSI */
  HAL_RCC_OscConfig(&RCCCONF);                            /* Clock initialization */
    
  HAL_RCCEx_SetLSCSource(RCC_LSCSOURCE_LSI);              /* LSC Select LSI */
  
  COMPRCC.PeriphClockSelection = RCC_PERIPHCLK_COMP1;     /* Peripheral selection COMP1 */
  COMPRCC.Comp1ClockSelection = RCC_COMP1CLKSOURCE_LSC;   /* Peripheral independent clock source selection LSC */
  HAL_RCCEx_PeriphCLKConfig(&COMPRCC);                    /* RCC extension peripheral clock initialization */
}

/**
  * @brief  Comparator initialization function
  * @param  None
  * @retval None
  */
static void APP_CompInit(void)
{
  hcomp1.Instance = COMP1;                                              /* COMP1 */
  hcomp1.Init.InputMinus      = COMP_INPUT_MINUS_IO1;                   /* Negative input is PA0  */
  hcomp1.Init.InputPlus       = COMP_INPUT_PLUS_IO3;                    /* Positive input selection is Vrefcmp */
  hcomp1.Init.OutputPol       = COMP_OUTPUTPOL_NONINVERTED;             /* COMP1 polarity Sexual selection is not reverse */
  hcomp1.Init.Mode            = COMP_POWERMODE_MEDIUMSPEED;             /* COMP1 power consumption mode is selected as Medium speed mode */
  hcomp1.Init.WindowMode      = COMP_WINDOWMODE_DISABLE;                /* Window Mode Off */
  hcomp1.Init.TriggerMode     = COMP_TRIGGERMODE_IT_RISING;             /* COMP1 external Trigger on Rise */
  hcomp1.Init.VrefDiv         = COMP_VREFCMP_DIV_32_64VREFCMP;          /* Vrefcmp div 32/64 */
  hcomp1.Init.VrefSrc         = COMP_VREFCMP_SOURCE_VCC;                /* Vrefcmp select VCC */
  hcomp1.Init.DigitalFilter   = 0;                                      /* Disable DigitalFilter */
  hcomp1.Init.TimAndExtiOutSel= COMP_TIM_EXTI_OUT_FILTER;               /* The comparator output to TIM or EXTI is filtered */
  /* COMP1 initialization */
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)                                 
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

  /* Oscillator configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE; /* Select oscillator HSE, HSI, LSI, LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                           /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                           /* HSI 1 frequency division */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* Configure HSI clock 24MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                          /* Close HSE */
  /* RCC_OscInitStruct.HSEFreq  = RCC_HSE_6_8MHz; */                 /* HSE select 6-8MHz */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                          /* Close LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                          /* Close LSE */
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
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

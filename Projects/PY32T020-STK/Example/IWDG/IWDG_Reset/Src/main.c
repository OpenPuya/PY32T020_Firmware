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
IWDG_HandleTypeDef   IwdgHandle;
   
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
    
  /* Configure Systemclock */
  APP_SystemClockConfig(); 

  BSP_LED_Init(LED_TK1);

  IwdgHandle.Instance = IWDG;                     /* Select IWDG */
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;  /* Configure prescaler as 32 */
  IwdgHandle.Init.Reload = (1024);                /* IWDG counter reload value is 1024 */

  /* Initialize IWDG */
  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)       
  {
    APP_ErrorHandler();
  }

  /* Infinite loop */
  while (1)
  {  
    HAL_Delay(900);                               /* Feed the dog every 900ms, the system can run normally */

    /* HAL_Delay(1100); */                        /* Feed the dog every 1.1s, the system will not run properly */
       
    /* Toggle LED */
    BSP_LED_Toggle(LED_TK1);
    
    /* Refresh IWDG */
    if (HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
    {
      APP_ErrorHandler();
    }
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

  /* Oscillator configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;         /* Select oscillator LSI */
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;                           /* Enable LSI */

  /* Configure oscillator */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
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

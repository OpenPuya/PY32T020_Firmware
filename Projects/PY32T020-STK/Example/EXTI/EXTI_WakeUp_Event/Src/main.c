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
EXTI_HandleTypeDef exti_handle;

/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_ConfigureEXTI(void);
static void APP_PWR_EnterStopMode(void);
  
/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();
  
  /* LED initialization */
  BSP_LED_Init(LED_TK1);

  /* Initialization button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
  
  /* Configure external events */
  APP_ConfigureEXTI();
  
  /* Turn on LED */
  BSP_LED_On(LED_TK1);
  
  /* Waiting for the user to press the button */
  while (BSP_PB_GetState(BUTTON_KEY) == 0)
  {
  }
  
  /* Turn off LED */
  BSP_LED_Off(LED_TK1);
  
  /* Pause systick */
  HAL_SuspendTick();
  
  /* Configure and enter stop mode */
  APP_PWR_EnterStopMode();
  
  /* Restore systick */
  HAL_ResumeTick();
  
  while (1)
  {
    BSP_LED_Toggle(LED_TK1);
    HAL_Delay(500);
  }
}

/**
  * @brief  Configure event pins
  * @param  None
  * @retval None
  */
static void APP_ConfigureEXTI(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();                  /* Enable GPIOA clock */
  GPIO_InitStruct.Mode  = GPIO_MODE_EVT_FALLING; /* GPIO mode is a falling edge event */
  GPIO_InitStruct.Pull  = GPIO_PULLUP;           /* pull up */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* The speed is high */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);        /* Entering stop mode */
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

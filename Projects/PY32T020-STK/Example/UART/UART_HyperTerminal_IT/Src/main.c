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
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define TXSTARTMESSAGESIZE    (COUNTOF(aTxStartMessage) - 1)
#define TXENDMESSAGESIZE      (COUNTOF(aTxEndMessage) - 1)

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle = {0};
uint8_t aRxBuffer[12] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t aTxStartMessage[] = "\n\r UART-Hyperterminal communication based on IT \n\r Enter 12 characters using keyboard :\n\r";
uint8_t aTxEndMessage[] = "\n\r Example Finished\n\r";
__IO uint8_t uErrorState   = 0;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Main program
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick */
  HAL_Init();
  
  /* Initialize LED */
  BSP_LED_Init(LED_TK1);
  
  /* Initialize UART */
  UartHandle.Instance          = UART3;
  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.MsbFirst     = UART_MSB_FIRST_DISABLE;
  HAL_UART_Init(&UartHandle);
  
  /* Start the transmission process */
  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxStartMessage, TXSTARTMESSAGESIZE)!= HAL_OK)
  {
    APP_ErrorHandler();
  }
  while(HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY);
  
  /* Put UART peripheral in reception process */
  if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, 12) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  while(HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY);
  
  /* Sends an amount of data in non blocking mode. */
  if (HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)aRxBuffer, 12) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY);

  /* Send the End Message */
  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxEndMessage, TXENDMESSAGESIZE)!= HAL_OK)
  {
    APP_ErrorHandler();
  }
  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY);
  
  BSP_LED_On(LED_TK1);

  while (1)
  {
    if( uErrorState == 1)
    {
      BSP_LED_Toggle(LED_TK1);
      HAL_Delay(200);
    }
  }
}

/**
  * @brief  UART error handler callback
  * @param  huart：UART handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uErrorState = 1;
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

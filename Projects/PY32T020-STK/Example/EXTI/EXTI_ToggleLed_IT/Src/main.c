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
EXTI_HandleTypeDef exti_handle;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_ConfigureExti(void);

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

  /* Configure external interrupts */
  APP_ConfigureExti();

  while (1)
  {
  }
}

/**
  * @brief  Configure EXTI
  * @param  None
  * @retval None
  */
static void APP_ConfigureExti(void)
{
  /* Configuration pins */
  GPIO_InitTypeDef  GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();                  /* Enable GPIOA clock */
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;   /* GPIO mode is a rising edge interrupt */
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;         /* pull down */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  /* The speed is high */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable EXTI interrupt */
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* Configure interrupt priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
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

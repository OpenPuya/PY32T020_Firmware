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
static void APP_ConfigTIM1Count(void);
static void APP_ConfigTIM1ExternalClock(void);
static void APP_GPIOConfig(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Enable TIM1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* Configure system clock */
  APP_SystemClockConfig();
 
  /* Initialize GPIO output */
  APP_GPIOConfig();
  
  /* Initialize LED */
  BSP_LED_Init(LED3);

  /* MCO (Microcontroller Clock Output) clock and divider initialization */
  LL_RCC_ConfigMCO(LL_RCC_MCOSOURCE_HSI,LL_RCC_MCO_DIV_4);
  
  /* Configure external clock mode 1 */
  APP_ConfigTIM1ExternalClock();
  
  /* Configure and enable TIM1 counter mode */
  APP_ConfigTIM1Count();
  
  while (1)
  {
  }
}

/**
  * @brief  Configure TIM external clock input
  * @param  None
  * @retval None
  */
static void APP_ConfigTIM1ExternalClock(void)
{  
  /* Configure PA8 pin as TI1 input */
  LL_GPIO_InitTypeDef ETRGPIOinit={0};
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  ETRGPIOinit.Pin        = LL_GPIO_PIN_3;
  ETRGPIOinit.Mode       = LL_GPIO_MODE_ALTERNATE;
  ETRGPIOinit.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  ETRGPIOinit.Pull       = LL_GPIO_PULL_UP;
  ETRGPIOinit.Alternate  = LL_GPIO_AF_2;
  
  LL_GPIO_Init(GPIOA,&ETRGPIOinit);
  
  /* Configure TIM1 external clock source mode 1 */
  LL_TIM_SetClockSource(TIM1,LL_TIM_CLOCKSOURCE_EXT_MODE1);
  
  /* Configure trigger selection as TI1F_ED */
  LL_TIM_SetTriggerInput(TIM1,LL_TIM_TS_TI1F_ED);
  
}

/**
  * @brief  Configure TIM count mode
  * @param  None
  * @retval None
  */
static void APP_ConfigTIM1Count(void)
{
  /* Configure TIM1 */
  LL_TIM_InitTypeDef TIM1CountInit = {0};
  
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1; /* No clock division */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;     /* Up counting mode */
  TIM1CountInit.Prescaler           = 1-1;                       /* Prescaler value: 1-1 */
  TIM1CountInit.Autoreload          = 800-1;                     /* Autoreload value: 800-1 */
  TIM1CountInit.RepetitionCounter   = 0;                         /* Repetition counter value: 0 */
  
  /* Initialize TIM1 */
  LL_TIM_Init(TIM1,&TIM1CountInit);
  
  /* Enable UPDATE interrupt */
  LL_TIM_EnableIT_UPDATE(TIM1);
  
  /* Enable TIM1 counter */
  LL_TIM_EnableCounter(TIM1);
  
  /* Enable UPDATE interrupt request */
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,0);
}

/**
  * @brief  TIM update interrupt callback function
  * @param  None
  * @retval None
  */
void APP_UpdateCallback(void)
{
  /* Toggle LED */
  BSP_LED_Toggle(LED3);
}

/**
  * @brief  System clock configuration function
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  /* Enable HSI */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB prescaler */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Configure HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);
  
  /* Update system clock global variable SystemCoreClock (can also be updated by calling SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}

/**
  * @brief  Configure PA08 as MCO alternate function
  * @param  None
  * @retval None
  */
static void APP_GPIOConfig(void)
{
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure PA08 as alternate function and set it as MCO output pin */
  LL_GPIO_InitTypeDef GPIO_InitStruct;  
  /* Select pin 8 */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8; 
  /* Set mode as alternate function */
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE; 
  /* Select alternate function 6 (AF6) */
  GPIO_InitStruct.Alternate = LL_GPIO_AF6_MCO;     
  /* Set output speed as very high frequency */
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;   
  /* Set output type as push-pull */
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  /* Set no pull-up/pull-down */
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;                
  
  /* Initialize GPIOA */
  LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
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

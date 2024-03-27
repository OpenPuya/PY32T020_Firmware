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
LL_TIM_InitTypeDef TimHandle = {0};
uint32_t temp = 0;

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
  /* Configure Systemclock */
  APP_SystemClockConfig(); 

  /* Enable TIM1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* Initialize LED */
  BSP_LED_Init(LED_TK1);

  /* TIM Config */
  TimHandle.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;        /* Clock division: tDTS=tCK_INT */
  TimHandle.CounterMode       = LL_TIM_COUNTERMODE_UP;            /* CounterMode:Up */
  TimHandle.Prescaler         = 1000 - 1;                         /* Prescaler:1000-1 */
  TimHandle.Autoreload        = 2400 - 1;                         /* TIM1_ARR register is not buffered */
  TimHandle.RepetitionCounter = 0;                                /* repetition counter value:1-1 */

  /* Initialize TIM1 */
  LL_TIM_Init(TIM1,&TimHandle);

  /* Enable the TIM Update interrupt */
  LL_TIM_EnableIT_UPDATE(TIM1);

  /* Enable TIM1  */
  LL_TIM_EnableCounter(TIM1);

  /* Set COM interrupt priority */
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn,0);
  /* Enable COM interrupt in NVIC */
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Update interrupt callback
  * @param  None
  * @retval None
  */
void APP_UpdattCallback(void)
{
  temp++;
  /* Toggle LED */
  BSP_LED_Toggle(LED_TK1);
  
  if (temp == 3)
  {
    /* Modify TIM auto reload value */
    LL_TIM_SetAutoReload(TIM1, 4800 - 1); 
  }
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

  /* Set AHB divider: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* HSISYS used as SYSCLK clock source  */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
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

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
#include "py32t020xx_ll_Start_Kit.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ExtiConfig(void);
static void APP_EnterHibernate(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Enable SYSCFG and PWR clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  
  /* Configure systemclock */
  APP_SystemClockConfig();
  
  /* LED initialization */
  BSP_LED_Init(LED_TK1);

  /* Initialize button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* Initialize external interrupt */
  APP_ExtiConfig();

  /* Turn on LED */
  BSP_LED_On(LED_TK1);

  /* Wait for button press */
  while (BSP_PB_GetState(BUTTON_USER) == 0)
  {
  }

  /* Turn off LED */
  BSP_LED_Off(LED_TK1);
    
  /* Enter hibernate mode */
  APP_EnterHibernate();
  
  while (1)
  {
    /* Toggle LED */
    BSP_LED_Toggle(LED_TK1);
    
    /* Delay for 200ms */
    LL_mDelay(200);
  }
}

/**
  * @brief  System clock configuration function
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  /* Enable and initialize HSI */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
  
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Configure HSISYS as system clock and initialize it */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }
  
  /* Set APB1 prescaler and initialize it */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);
  /* Update system clock global variable SystemCoreClock (can also be updated by calling SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}

/**
  * @brief  Configure external interrupt
  * @param  None
  * @retval None
  */
static void APP_ExtiConfig(void)
{
   /* Enable GPIOA clock */
   LL_IOP_GRP1_EnableClock (LL_IOP_GRP1_PERIPH_GPIOA);
  
   LL_GPIO_InitTypeDef GPIO_InitStruct;
   /* Select PA06 pin */
   GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
   /* Select input mode */
   GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
   /* Select pull-up */
   GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
   /* Initialize GPIOA */
   LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   LL_EXTI_InitTypeDef EXTI_InitStruct;
   /* Select EXTI6 */
   EXTI_InitStruct.Line = LL_EXTI_LINE_6;
   /* Enable */
   EXTI_InitStruct.LineCommand = ENABLE;
   /* Select interrupt mode */
   EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
   /* Select falling edge trigger */
   EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
   /* Initialize external interrupt */
   LL_EXTI_Init(&EXTI_InitStruct);
}

/**
  * @brief  Enter hibernate mode
  * @param  None
  * @retval None
  */
static void APP_EnterHibernate(void)
{
  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* hibernate mode with deep low power regulator ON */
  LL_PWR_SetLprMode(LL_PWR_LPR_MODE_DLPR);

  /* Set SRAM voltage in hibernate mode */
  LL_PWR_SetSramRetentionVolt(LL_PWR_SRAM_RETENTION_VOLT_HIBERNATE);
  
  /* Enter DeepSleep mode */
  LL_LPM_EnableDeepSleep();

  /* Request Wait For event */
  __SEV();
  __WFE();
  __WFE();

  LL_LPM_EnableSleep();
}

/**
  * @brief  Error handling function
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
  * @param  file:Pointer to the source file name
  * @param  line:assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add His own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

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
static void APP_CompInit(void);
static void APP_ComparatorTrigger(void);
static void APP_GpioInit(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Enable SYSCFG clock and PWR clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure Systemclock */
  APP_SystemClockConfig();

  /* Initialize LED */
  BSP_LED_Init(LED_TK1);
  
  /* Initialize PA4 */
  APP_GpioInit();
  
  /* Initialize COMP */
  APP_CompInit();

  while (1)
  {
    APP_ComparatorTrigger();
  }
}

/**
  * @brief  Initialize GPIO
  * @param  None
  * @retval None
  */
static void APP_GpioInit(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_4,LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinOutputType(GPIOA,LL_GPIO_PIN_4,LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOA,LL_GPIO_PIN_4,LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOA,LL_GPIO_PIN_4,LL_GPIO_PULL_UP);
}

/**
  * @brief  Initialize COMP
  * @param  None
  * @retval None
  */
static void APP_CompInit(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_GPIO_SetPinMode(GPIOB,LL_GPIO_PIN_1,LL_GPIO_MODE_ANALOG);
  LL_SYSCFG_EnableGPIOAnalog2(LL_SYSCFG_GPIO_PORTB,LL_SYSCFG_GPIO_PIN_1);
  
  LL_GPIO_SetPinMode(GPIOA,LL_GPIO_PIN_2,LL_GPIO_MODE_ANALOG);
  LL_SYSCFG_EnableGPIOAnalog2(LL_SYSCFG_GPIO_PORTA,LL_SYSCFG_GPIO_PIN_2);

  /* Enable COMP2 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_COMP2);

  /* Enable COMP1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_COMP1);

  /* Select VCC as VREFCMP reference source */
  LL_COMP_SetVrefCmpSource(COMP1,LL_COMP_VREFCMP_SOURCE_VCC);

  /* Set VREFCMP divider : VREFCMP=1/2 VCC */
  LL_COMP_SetVrefCmpDivider(COMP1,LL_COMP_VREFCMP_DIV_32_64VREFCMP);

  /* Enable Vrefcmp divider */
  LL_COMP_EnableVrefCmpDivider(COMP1);

  /* Select VREFCMP as COMP2 inputplus */
  LL_COMP_SetInputPlus(COMP2,LL_COMP_INPUT_PLUS_IO4);
  
  /* Select PA2 as COMP2 inputMinus */
  LL_COMP_SetInputMinus(COMP2,LL_COMP_INPUT_MINUS_IO1);

  /* Select PB1 as COMP1 inputMinus */
  LL_COMP_SetInputMinus(COMP1,LL_COMP_INPUT_MINUS_IO2);

  /* Enable WindowMode (COMP2 inputplus connect COMP1 inputplus ) */
  LL_COMP_SetCommonWindowMode(__LL_COMP_COMMON_INSTANCE(COMP1), LL_COMP_WINDOWMODE_COMP2_INPUT_PLUS_COMMON);

  /* Enable COMP1 */
  LL_COMP_Enable(COMP1);
  
  /* Enable COMP2 */
  LL_COMP_Enable(COMP2);

  __IO uint32_t wait_loop_index = 0;
  wait_loop_index = ((LL_COMP_DELAY_STARTUP_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
  while(wait_loop_index != 0UL)
  {
    wait_loop_index--;
  }
}

/**
  * @brief  Result processing functions for comparison
  * @param  None
  * @retval None
  */
void APP_ComparatorTrigger()
{
  if(LL_COMP_ReadOutputLevel(COMP1)==LL_COMP_OUTPUT_LEVEL_HIGH)
  {
    BSP_LED_On(LED_TK1);
  }
  else
  {
    BSP_LED_Off(LED_TK1);
  }
  
  if(LL_COMP_ReadOutputLevel(COMP2)==LL_COMP_OUTPUT_LEVEL_HIGH)
  {
    LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_4);
  }
  else
  {
    LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_4);
  }
}

/**
  * @brief  Configure systemclock
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

  /* Set AHB prescaler: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Select HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB prescaler: PCLK = HCLK */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);

  /* Update the SystemCoreClock global variable(which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
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

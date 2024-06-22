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
#define  PERIOD_VALUE       (uint32_t)(700 - 1)               /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */

/* Private variables ---------------------------------------------------------*/
LL_GPIO_InitTypeDef    GPIOxCH1MapInit = {0};
LL_TIM_InitTypeDef     TimHandle       = {0};
LL_TIM_OC_InitTypeDef  sPWMConfig      = {0};

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigTIM1(void);

/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Configure Systemclock */
  APP_SystemClockConfig(); 

  /* Enable TIM1 peripheral clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  
  /* Enabel GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Enabel GPIOB clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /* PWM Config */
  sPWMConfig.OCMode       = LL_TIM_OCMODE_PWM1;                     /* Set as PWM1 mode */
  sPWMConfig.OCState      = LL_TIM_OCSTATE_ENABLE;                  /* Set as OCMODE state */
  sPWMConfig.OCNState     = LL_TIM_OCSTATE_ENABLE;                  /* Set as OCMODE state */
  sPWMConfig.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;                 /* Compare output polarity: HIGH */
  sPWMConfig.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;                 /* Compare complementary output polarity: HIGH */
  sPWMConfig.OCIdleState  = LL_TIM_OCIDLESTATE_HIGH;                /* Output Idle state: HIGH */
  sPWMConfig.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;                 /* Complementary output Idle state: LOW */

  sPWMConfig.CompareValue = PULSE1_VALUE;                           /* Duty cycle of channel 1 */
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &sPWMConfig);            /* Configure Channel 1/1N  */

  sPWMConfig.CompareValue = PULSE2_VALUE;                           /* Duty cycle of channel 2 */
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &sPWMConfig);            /* Configure Channel 2/2N  */

  sPWMConfig.CompareValue = PULSE3_VALUE;                           /* Duty cycle of channel 3 */
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &sPWMConfig);            /* Configure Channel 3/3N  */

  /* Set TIM1 in PWM mode */
  APP_ConfigTIM1();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Configure TIM1.
  * @param  None
  * @retval None
  */
static void APP_ConfigTIM1(void)
{
  /* Configure TIM1 */
  LL_TIM_InitTypeDef TIM1CountInit = {0};
  
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;/* Set divider:tDTS=tCK_INT  */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;    /* count mode：up count      */
  TIM1CountInit.Prescaler           = 2400-1;                   /* clock prescaler：2400     */
  TIM1CountInit.Autoreload          = 1000-1;                   /* auto-reload value：1000   */
  TIM1CountInit.RepetitionCounter   = 0;                        /* recount：0                */

  /* Set PA3/PA7/PA5/PA4 as TIM1_CH1/TIM1_CH1N/TIM1_CH2/TIM1_CH3 */
  GPIOxCH1MapInit.Pin        = LL_GPIO_PIN_3|LL_GPIO_PIN_7|LL_GPIO_PIN_5|LL_GPIO_PIN_4;
  GPIOxCH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  GPIOxCH1MapInit.Alternate  = LL_GPIO_AF_2;
  GPIOxCH1MapInit.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
  GPIOxCH1MapInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIOxCH1MapInit.Pull       = LL_GPIO_PULL_DOWN;  
  LL_GPIO_Init(GPIOA,&GPIOxCH1MapInit);

  /* Set PB0/PB1 as TIM1_CH2N/TIM1_CH3N */
  GPIOxCH1MapInit.Pin        = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIOxCH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  GPIOxCH1MapInit.Alternate  = LL_GPIO_AF_2; 
  LL_GPIO_Init(GPIOB,&GPIOxCH1MapInit);
  
  /* Initialize TIM1 */
  LL_TIM_Init(TIM1,&TIM1CountInit);
  
  /* Enable output */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 */
  LL_TIM_EnableCounter(TIM1);
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

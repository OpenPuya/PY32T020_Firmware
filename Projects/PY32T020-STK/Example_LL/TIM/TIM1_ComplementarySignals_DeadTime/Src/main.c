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
#define  PULSE1_VALUE       (uint32_t)(400)        /* Capture Compare 1 Value  */

/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigPWMChannel(void);
static void APP_ConfigBDTR(void);
static void APP_ConfigTIM1Base(void);

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

  /* Configure TIM1 PWM channels */
  APP_ConfigPWMChannel();

  /* Configure brake and dead time */
  APP_ConfigBDTR();
  
  /* Configure and enable TIM1 PWM mode */
  APP_ConfigTIM1Base();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Configure TIM base
  * @param  None
  * @retval None
  */
static void APP_ConfigTIM1Base(void)
{
  /* Configure TIM1 */
  LL_TIM_InitTypeDef TIM1CountInit = {0};

  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;/* Set divider:tDTS=tCK_INT  */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;    /* count mode：up count      */
  TIM1CountInit.Prescaler           = 2400-1;                   /* clock prescaler：2400     */
  TIM1CountInit.Autoreload          = 1000-1;                   /* auto-reload value：1000   */
  TIM1CountInit.RepetitionCounter   = 0;                        /* recount：0                */

  /* Initialize TIM1 */
  LL_TIM_Init(TIM1,&TIM1CountInit);
  
  /* Enable main output */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 counter */
  LL_TIM_EnableCounter(TIM1);
}

/**
  * @brief  Configure dead time and brake
  * @param  None
  * @retval None
  */
static void APP_ConfigBDTR(void)
{
  LL_TIM_BDTR_InitTypeDef TIM1BDTRInit = {0};
  LL_GPIO_InitTypeDef     TIM1BreakMapInit = {0};

  /* Configures the Break feature, dead time */
  TIM1BDTRInit.BreakState       = LL_TIM_BREAK_ENABLE;                    /* Break input BRK is enabled */
  TIM1BDTRInit.DeadTime         = 160;                                    /* Set the dead time */
  TIM1BDTRInit.OSSRState        = LL_TIM_OSSR_ENABLE;                     /* Off state in run mode: output enable */
  TIM1BDTRInit.OSSIState        = LL_TIM_OSSI_ENABLE;                     /* IDLE state in run mode: output enable */
  TIM1BDTRInit.LockLevel        = LL_TIM_LOCKLEVEL_OFF;                   /* LOCK off */
  TIM1BDTRInit.BreakPolarity    = LL_TIM_BREAK_POLARITY_LOW;              /* Break input BRK is active LOW */
  TIM1BDTRInit.AutomaticOutput  = LL_TIM_AUTOMATICOUTPUT_ENABLE;          /* Automatic output enable */

  /* Set PA6 BKIN */
  TIM1BreakMapInit.Pin        = LL_GPIO_PIN_6;
  TIM1BreakMapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1BreakMapInit.Alternate  = LL_GPIO_AF_5; 
  LL_GPIO_Init(GPIOA,&TIM1BreakMapInit);

  /* Initialize dead time and brake configuration */
  LL_TIM_BDTR_Init(TIM1,&TIM1BDTRInit);
}

/**
  * @brief  Configure TIM1 PWM related GPIO
  * @param  None
  * @retval None
  */
static void APP_ConfigPWMChannel(void)
{
  LL_GPIO_InitTypeDef TIM1CH1MapInit= {0};
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct ={0};

  /* Set PA3/PA7 as TIM1_CH1/TIM1_CH1N */
  TIM1CH1MapInit.Pin        = LL_GPIO_PIN_3|LL_GPIO_PIN_7;
  TIM1CH1MapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CH1MapInit.Alternate  = LL_GPIO_AF_2; 
  LL_GPIO_Init(GPIOA,&TIM1CH1MapInit);

  /* PWM Config */
  TIM_OC_Initstruct.OCMode       = LL_TIM_OCMODE_PWM1;                    /* Set as PWM1 mode */
  TIM_OC_Initstruct.OCState      = LL_TIM_OCSTATE_ENABLE;                 /* Set as OCMODE state */
  TIM_OC_Initstruct.OCNState     = LL_TIM_OCSTATE_ENABLE;                 /* Set as OCMODE state */
  TIM_OC_Initstruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;                /* Compare output polarity: HIGH */
  TIM_OC_Initstruct.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;                /* Compare complementary output polarity: HIGH */
  TIM_OC_Initstruct.OCIdleState  = LL_TIM_OCIDLESTATE_HIGH;               /* Output Idle state: HIGH */
  TIM_OC_Initstruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;                /* Complementary output Idle state: LOW */
  TIM_OC_Initstruct.CompareValue = PULSE1_VALUE;                          /* Duty cycle of channel 1 */

  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_Initstruct);           /* Configure Channel 1/1N  */
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

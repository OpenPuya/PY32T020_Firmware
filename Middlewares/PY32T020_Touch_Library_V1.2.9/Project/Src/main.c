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
#if APP_TK_ENABLE
static void TK_Loop(void);
#endif
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/********************************************************
**	函数名	void SystemClockConfig(void)
**	描述	：时钟初始化，用于设置系统时钟
**	传入	：无
**	返回	：无
*********************************************************/
static void SystemClockConfig(void)
{
	#if (HSI_FREQUENCE == 48000000UL)
	/*	修改系统主频	*/
	SET_BIT(RCC->CR, RCC_CR_HSION);
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_1);
	MODIFY_REG(RCC->ICSCR, (RCC_ICSCR_HSI_FS | RCC_ICSCR_HSI_TRIM), LL_RCC_HSICALIBRATION_48MHz);
	while((READ_BIT(RCC->CR, RCC_CR_HSIRDY) != (RCC_CR_HSIRDY)))
	{
		;
	}
	#else
	SET_BIT(RCC->CR, RCC_CR_HSION);
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_0);
	MODIFY_REG(RCC->ICSCR, (RCC_ICSCR_HSI_FS | RCC_ICSCR_HSI_TRIM), LL_RCC_HSICALIBRATION_24MHz);
	while((READ_BIT(RCC->CR, RCC_CR_HSIRDY) != (RCC_CR_HSIRDY)))
	{
		;
	}
	#endif
	/* Set AHB divider: HCLK = SYSCLK */
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	/* HSISYS used as SYSCLK clock source  */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
	{
		;
	}
	/* Set APB1 divider */
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	SystemCoreClockUpdate();
	/* Enable SYSCFG and PWR clocks */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableBkUpAccess();
	/* Enable GPIOA clock */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/* Enable GPIOB clock */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	/* Enable GPIOF clock */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
	#if APP_SYSTICK_ENABLE
	/*	测试GPIO翻转		*/
	#if (SYSTICK_DEBUG_GPIO != NO_PIN && SYSTICK_DEBUG)
	GPIO_Init(SYSTICK_DEBUG_GPIO,OUTPUT|PUSHPULL);
	GPIO_ClearBit(SYSTICK_DEBUG_GPIO);
	#endif
	/*	初始Systick定时器*/
	SysTick_Config(SystemCoreClock / 1000000 * SYSTICK_TIMING_TIME);
	/* Set the interrupt priority */
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_IRQ_PRIORITY);
    /* Enable TIM Break, Update, Trigger and Commutation Interrupts */
    NVIC_EnableIRQ(SysTick_IRQn);
	#endif
}
/********************************************************
**	函数名	int main(void)
**	描述	：主函数入口
**	传入	：无
**	返回	：无
*********************************************************/
int main(void)
{
	SystemClockConfig();
    /*	用户初始化代码开始*/
	app_drivers_init();
    /*	用户初始化代码结束*/
	/*  触摸初始化开始 */
	#if APP_TK_ENABLE
    TK_Init();
	#endif
	/*  触摸初始化结束 */
	log_printf("start loop\r\n");
    while (1)
    {
		#if APP_TK_ENABLE
		TK_Loop();
		#endif
		app_drivers_loop();
    }
}
#if APP_SYSTICK_ENABLE
/********************************************************
**	函数名	void SysTick_Handler(void)
**	描述	：系统滴答定时器，目前定时时间为1ms
**	传入	：无
**	返回	：无
*********************************************************/
void SysTick_Handler(void)
{
	static uint16_t Prescaler = 0;
	#if (SYSTICK_DEBUG_GPIO != NO_PIN && SYSTICK_DEBUG)
	GPIO_ToggleBit(SYSTICK_DEBUG_GPIO);
	#endif
	Prescaler++;
	if(Prescaler >= (1000 / SYSTICK_TIMING_TIME))
	{
		Prescaler = 0;
		app_drivers_timer();
		#if APP_TK_ENABLE
		TK_TimerHandler(1);
		#endif
	}
}
#endif
#if APP_TK_ENABLE
/********************************************************
**	函数名	void TK_Loop(void)
**	描述	：触摸按键获取以及滑条位置获取
**	传入	：无
**	返回	：无
*********************************************************/
static void TK_Loop(void)
{
    static uint32_t KeyFlag = 0;
    uint32_t temp;
	uint8_t i;
	/*	触摸状态机处理	*/
	TK_MainFsm();
	/*************************************************************************************************
			 变量 TKCtr.KeyFlags是触摸库对外的数据接口，
	   TKCtr.KeyFlags的每一位对应一个触摸键的状态，为1表示触摸键触发。可多键同时触发。
	*************************************************************************************************/
	temp = TKCtr.KeyFlags ^ KeyFlag;
	if (temp != 0)
	{
		KeyFlag = TKCtr.KeyFlags;
		log_printf("KeyFlag:0X%X\r\n",KeyFlag);
		#if APP_SMG_ENABLE
		LED_Show(KeyFlag); 			// 显示对应的LED
		#endif
		for (i = 0; i < TKCtr.TouchKeyChCnt; i++)
		{
			if (temp & 0X01)
			{
				if (KeyFlag & ((0X01) << i)) 	// 按键按下
				{
					#if APP_BEEP_ENABLE
					BEEP_On(100);
					#endif
					#if APP_SMG_ENABLE
					SMG_Show(i + 1);
					#endif
					switch(i)
					{
						/*	KEY0按下*/
						case 0:
							
						break;
						/*	KEY1按下*/
						case 1:
						break;
					}
				}
				else
				{
					switch(i)
					{
						/*	KEY0松开*/
						case 0:
						break;
						/*	KEY1松开*/
						case 1:
						break;
					}
				}
			}
			temp >>= 1;
		}
	}
	#if (LIB_TYPE > 0)
	if (TKCtr.SliderOrWheelPosition[0] != -1)
	{

	}
	#endif
}
#endif
#ifdef USE_FULL_ASSERT
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
		;
    }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

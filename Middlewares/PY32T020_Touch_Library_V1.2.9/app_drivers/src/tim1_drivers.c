#include "tim1_drivers.h"
#if APP_TIM1_ENABLE
/********************************************************
**	函数名	void TIM1_Init(void)
**	描述	定时器1初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TIM1_Init(void)
{
	LL_TIM_InitTypeDef TimHandle = {0};
	uint32_t Prescale;
	uint32_t Autoreload;
	uint32_t Frequency = 1000000 / TIM1_TIMING_TIME;
	/*	测试GPIO翻转		*/
	#if (TIM1_DEBUG_GPIO != NO_PIN && TIM1_DEBUG)
	GPIO_Init(TIM1_DEBUG_GPIO,OUTPUT|PUSHPULL);
	GPIO_ClearBit(TIM1_DEBUG_GPIO);
	#endif
	for(Prescale = 1U; Prescale < 0xFFFFU; Prescale++)   /* prescale could be 0~0xFFFF */
    {
        Autoreload = (SystemCoreClock / Frequency) / Prescale;
        /* If target value is larger than CNR, need to use a larger prescaler */
        if(Autoreload < (0x10000U))
        {
            break;
        }
    }
    /* Enable TIM1 clock */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
	/* TIM Config */
	TimHandle.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;        /* Clock division: tDTS=tCK_INT */
	TimHandle.CounterMode       = LL_TIM_COUNTERMODE_UP;            /* CounterMode:Up */
	TimHandle.RepetitionCounter = 0;                                /* repetition counter value:1-1 */
    TimHandle.Autoreload = Autoreload - 1;                             /* Auto reload value： */
	TimHandle.Prescaler = Prescale - 1;                               	/* Prescaler:24 - 1 */
	/* Initialize TIM1 */
	LL_TIM_Init(TIM1,&TimHandle);
	/* Enable the TIM Update interrupt */
	LL_TIM_EnableIT_UPDATE(TIM1);
	/* Enable TIM1  */
	LL_TIM_EnableCounter(TIM1);
	#if TIM1_IRQ_ENABLE
	/* Set the interrupt priority */
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, TIM1_IRQ_PRIORITY);
    /* Enable TIM Break, Update, Trigger and Commutation Interrupts */
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	#endif
}
/********************************************************
**	函数名	void TIM1_PeriodElapsedCallback(void)
**	描述	定时器1中断回调函数
**	传入	：无
**	返回	：无
*********************************************************/
__weak void TIM1_PeriodElapsedCallback(void)
{


}
/********************************************************
**	函数名	void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
**	描述	定时器1中断入口
**	传入	：无
**	返回	：无
*********************************************************/
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	/* TIM Update event */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM1) != 0)
	{
		LL_TIM_ClearFlag_UPDATE(TIM1);
		/*	测试GPIO翻转		*/
		#if (TIM1_DEBUG_GPIO != NO_PIN && TIM1_DEBUG)
		GPIO_ToggleBit(TIM1_DEBUG_GPIO);
		#endif
		TIM1_PeriodElapsedCallback();
	}
}
#elif APP_TIM1_PWM_ENABLE
static LL_TIM_OC_InitTypeDef TIM_OC_Initstruct ={0};
static uint32_t TIM1_Autoreload;
/********************************************************
**	函数名	void TIM1_PWM_Init(void)
**	描述	定时器1用做PWM初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TIM1_PWM_Init(void)
{
	/* Configure TIM1 */
	LL_TIM_InitTypeDef TIM1CountInit = {0};
	uint32_t Prescale;
	for(Prescale = 1U; Prescale < 0xFFFFU; Prescale++)   /* prescale could be 0~0xFFFF */
    {
        TIM1_Autoreload = (SystemCoreClock / TIM1_PWM_FREQUENCE) / Prescale;
        /* If target value is larger than CNR, need to use a larger prescaler */
        if(TIM1_Autoreload < (0x10000U))
        {
            break;
        }
    }
	/* Enable TIM1 peripheral clock */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
	TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;/* Set divider:tDTS=tCK_INT  */
	TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_UP;    /* count mode：up count      */
	TIM1CountInit.Autoreload      	  = TIM1_Autoreload - 1;      /* auto-reload value：1000   */
	TIM1CountInit.Prescaler           = Prescale - 1;             /* clock prescaler：2400     */
	TIM1CountInit.RepetitionCounter   = 0;                        /* recount：0                */
	/* Initialize TIM1 */
	LL_TIM_Init(TIM1,&TIM1CountInit);
    /* Set PWM channel */
	TIM_OC_Initstruct.OCMode        = LL_TIM_OCMODE_PWM1;       /* channle mode：PWM2         */
	TIM_OC_Initstruct.OCState       = LL_TIM_OCSTATE_ENABLE;    /* Enable selected channle    */
	TIM_OC_Initstruct.OCPolarity    = LL_TIM_OCPOLARITY_HIGH;   /* channle active level：high */
	TIM_OC_Initstruct.OCIdleState   = LL_TIM_OCIDLESTATE_LOW;   /* channle idle polarity：low */
	#if TIM1_CH1_ENABLE
		#if (TIM1_CH1_PIN == 7)
		GPIO_Init(TIM1_CH1_PIN,ALTERNATE | GPIO_TIM1_AF5);
		#else
		GPIO_Init(TIM1_CH1_PIN,ALTERNATE | GPIO_TIM1_AF2);
		#endif
		TIM1_PWM_Pulse(LL_TIM_CHANNEL_CH1,TIM1_DUTY1_CYCLE);
	#endif
	#if TIM1_CH2_ENABLE
		#if (TIM1_CH2_PIN == 16)
		GPIO_Init(TIM1_CH2_PIN,ALTERNATE | GPIO_TIM1_AF5);
		#else
		GPIO_Init(TIM1_CH2_PIN,ALTERNATE | GPIO_TIM1_AF2);
		#endif
		TIM1_PWM_Pulse(LL_TIM_CHANNEL_CH2,TIM1_DUTY2_CYCLE);
	#endif
	#if TIM1_CH3_ENABLE
		GPIO_Init(TIM1_CH3_PIN,ALTERNATE | GPIO_TIM1_AF2);
		TIM1_PWM_Pulse(LL_TIM_CHANNEL_CH3,TIM1_DUTY3_CYCLE);
	#endif
	#if TIM1_CH4_ENABLE
		#if (TIM1_CH4_PIN == 15)
		GPIO_Init(TIM1_CH4_PIN,ALTERNATE | GPIO_TIM1_AF5);
		#else
		GPIO_Init(TIM1_CH4_PIN,ALTERNATE | GPIO_TIM1_AF2);
		#endif
		TIM1_PWM_Pulse(LL_TIM_CHANNEL_CH4,TIM1_DUTY4_CYCLE);
	#endif
	/* Enable output */
	LL_TIM_EnableAllOutputs(TIM1);
	/* Enable TIM1 */
	LL_TIM_EnableCounter(TIM1);
}
/********************************************************
**	函数名	void TIM1_PWM_Pulse(uint32_t CHx,uint16_t percent)
**	描述	定时器1-PWM占空比设置
**	传入	：	CHx 通道号	LL_TIM_CHANNEL_CH1~LL_TIM_CHANNEL_CH4，
				percent		输出百分比
**	返回	：无
*********************************************************/
void TIM1_PWM_Pulse(uint32_t CHx,uint16_t percent)
{
	/* CH1 compare value:250 */
	TIM_OC_Initstruct.CompareValue  = TIM1_Autoreload  * percent / TIM1_RESOLUTION;
	/* Configure CH1 */
	LL_TIM_OC_Init(TIM1,CHx,&TIM_OC_Initstruct);
}
#endif

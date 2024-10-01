#ifndef _TIM1_DRIVERS_H_
#define _TIM1_DRIVERS_H_

#include "app_config.h"
#if APP_TIM1_ENABLE
/********************************************************
**	函数名	void TIM1_Init(void)
**	描述	定时器1初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TIM1_Init(void);
/********************************************************
**	函数名	void TIM1_PeriodElapsedCallback(void)
**	描述	定时器1中断回调函数
**	传入	：无
**	返回	：无
*********************************************************/
void TIM1_PeriodElapsedCallback(void);
#elif APP_TIM1_PWM_ENABLE
/********************************************************
**	函数名	void TIM1_PWM_Init(void)
**	描述	定时器1用做PWM初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TIM1_PWM_Init(void);
/********************************************************
**	函数名	void TIM1_PWM_Pulse(uint32_t CHx,uint16_t percent)
**	描述	定时器1-PWM占空比设置
**	传入	：	CHx 通道号	LL_TIM_CHANNEL_CH1~LL_TIM_CHANNEL_CH4，
				percent		输出百分比
**	返回	：无
*********************************************************/
void TIM1_PWM_Pulse(uint32_t CHx,uint16_t percent);
#endif

#endif

#ifndef _TIM1_DRIVERS_H_
#define _TIM1_DRIVERS_H_

#include "app_config.h"
#if APP_TIM1_ENABLE
/********************************************************
**	������	void TIM1_Init(void)
**	����	��ʱ��1��ʼ��
**	����	����
**	����	����
*********************************************************/
void TIM1_Init(void);
/********************************************************
**	������	void TIM1_PeriodElapsedCallback(void)
**	����	��ʱ��1�жϻص�����
**	����	����
**	����	����
*********************************************************/
void TIM1_PeriodElapsedCallback(void);
#elif APP_TIM1_PWM_ENABLE
/********************************************************
**	������	void TIM1_PWM_Init(void)
**	����	��ʱ��1����PWM��ʼ��
**	����	����
**	����	����
*********************************************************/
void TIM1_PWM_Init(void);
/********************************************************
**	������	void TIM1_PWM_Pulse(uint32_t CHx,uint16_t percent)
**	����	��ʱ��1-PWMռ�ձ�����
**	����	��	CHx ͨ����	LL_TIM_CHANNEL_CH1~LL_TIM_CHANNEL_CH4��
				percent		����ٷֱ�
**	����	����
*********************************************************/
void TIM1_PWM_Pulse(uint32_t CHx,uint16_t percent);
#endif

#endif

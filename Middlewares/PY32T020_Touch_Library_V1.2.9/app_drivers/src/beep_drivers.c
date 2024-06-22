#include "beep_drivers.h"
#if APP_BEEP_ENABLE
static uint8_t beep_delay;
/********************************************************
**	函数名	void BEEP_Init(void)
**	描述	：蜂鸣器GPIO初始化
**	传入	：无
**	返回	：无
*********************************************************/
void BEEP_Init(void)
{
	GPIO_Init(BEEP_GPIO,OUTPUT|PUSHPULL);
	#if BEEP_GPIO_TYPE
    GPIO_ClearBit(BEEP_GPIO);
	#else
	GPIO_SetBit(BEEP_GPIO);
	#endif
}
/********************************************************
**	函数名	void BEEP_On(uint8_t timeout)
**	描述	：打开蜂鸣器
**	传入	：timeout 开启时间ms
**	返回	：无
*********************************************************/
void BEEP_On(uint8_t timeout)
{
    beep_delay = timeout;
	#if BEEP_GPIO_TYPE
    GPIO_SetBit(BEEP_GPIO);
	#else
	GPIO_ClearBit(BEEP_GPIO);
	#endif
}
/********************************************************
**	函数名	void BEEP_Timeout(void)
**	描述	：延时关闭蜂鸣器，系统定时器中断调用，调用周期1ms
**	传入	：t无
**	返回	：无
*********************************************************/
void BEEP_Timeout(void)
{
	if (beep_delay)
    {
        beep_delay--;
        if (beep_delay == 0)
        {
			#if BEEP_GPIO_TYPE
			GPIO_ClearBit(BEEP_GPIO);
			#else
			GPIO_SetBit(BEEP_GPIO);
			#endif
        }
    }
}
#endif

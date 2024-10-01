#include "beep_drivers.h"
#if APP_BEEP_ENABLE
static uint8_t beep_delay;
/********************************************************
**	������	void BEEP_Init(void)
**	����	��������GPIO��ʼ��
**	����	����
**	����	����
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
**	������	void BEEP_On(uint8_t timeout)
**	����	���򿪷�����
**	����	��timeout ����ʱ��ms
**	����	����
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
**	������	void BEEP_Timeout(void)
**	����	����ʱ�رշ�������ϵͳ��ʱ���жϵ��ã���������1ms
**	����	��t��
**	����	����
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

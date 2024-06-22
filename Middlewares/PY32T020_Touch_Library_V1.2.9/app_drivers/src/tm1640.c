#include "tm1640.h"
#if APP_TM1640_ENABLE
/********************************************************
**	������	void TM1640_Init(void)
**	����	TM1640��ʼ��
**	����	����
**	����	����
*********************************************************/
void TM1640_Init(void)
{
	GPIO_Init(TM1640_CLK,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1640_CLK);
	GPIO_Init(TM1640_DIN,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1640_DIN);
}
/********************************************************
**	������	void TM1640_Start(void)
**	����	TM1624���Ϳ�ʼ�ź�
**	����	����
**	����	����
*********************************************************/
static void TM1640_Start(void)
{
	GPIO_SetBit(TM1640_DIN);     
	GPIO_SetBit(TM1640_CLK);    
	nop_delay_xus(1);
	GPIO_ClearBit(TM1640_DIN);  
	nop_delay_xus(1);
}
/********************************************************
**	������	void TM1640_Stop(void)
**	����	TM1624����ֹͣ�ź�
**	����	����
**	����	����
*********************************************************/
static void TM1640_Stop(void)
{
	GPIO_ClearBit(TM1640_DIN);     
	GPIO_ClearBit(TM1640_CLK);  
	nop_delay_xus(1);
	GPIO_SetBit(TM1640_CLK);  
	nop_delay_xus(1);
	GPIO_SetBit(TM1640_DIN);     
	nop_delay_xus(1);
}
/********************************************************
**	������	void TM1640_SendByte(uint8_t data)
**	����	TM1624����1���ֽ�
**	����	��data �����͵�����
**	����	����
*********************************************************/
static void TM1640_SendByte(uint8_t data)
{
	for(uint8_t i = 0;i < 8;i++)
	{
		GPIO_ClearBit(TM1640_CLK); 
		if(data & 0X01)
			GPIO_SetBit(TM1640_DIN);     
		else
			GPIO_ClearBit(TM1640_DIN);   
		nop_delay_xus(1);		
		GPIO_SetBit(TM1640_CLK);  
		data >>= 1;
		nop_delay_xus(1);
	}
}
/********************************************************
**	������	void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
**	����	TM1640ˢ��
**	����	��display��ʾ������ָ�� num��ʾ������ Config����ָ��������ȵ����Լ�������ʾ
**	����	����
*********************************************************/
void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
{
	uint8_t i;
	TM1640_Start();
	TM1640_SendByte(0X40);				//���õ�ַ�Լ�ģʽ
	TM1640_Stop();
	
	TM1640_Start();
	TM1640_SendByte(0XC0);				//�����׵�ַ
	for(i = 0;i < num;i++,display++)
	{
		TM1640_SendByte(*display);
	}
	TM1640_Stop();
	
	TM1640_Start();
	TM1640_SendByte(0X80 | Config);		//��������
	TM1640_Stop();
}

#endif

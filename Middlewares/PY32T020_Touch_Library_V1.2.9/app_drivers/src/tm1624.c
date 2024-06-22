#include "tm1624.h"
#if APP_TM1624_ENABLE
/********************************************************
**	������	void TM1624_Init(void)
**	����	TM1624��ʼ��
**	����	����
**	����	����
*********************************************************/
void TM1624_Init(void)
{
	GPIO_Init(TM1624_CLK,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1624_CLK);
	GPIO_Init(TM1624_DIN,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1624_DIN);
	GPIO_Init(TM1624_STB,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1624_STB);
}
/********************************************************
**	������	void TM1624_SendByte(uint8_t dat)
**	����	TM1624����һ���ֽ�
**	����	������
**	����	����
*********************************************************/
static void TM1624_SendByte(uint8_t dat)
{
	uint8_t i;
	for(i = 0;i < 8;i++)
	{	
		GPIO_ClearBit(TM1624_CLK);  	 //CLK  ��
		if(dat & 0X01) 
			GPIO_SetBit(TM1624_DIN);     //DIN  ��
		else         
			GPIO_ClearBit(TM1624_DIN);   //DIN  ��
		nop_delay_xus(1);
		GPIO_SetBit(TM1624_CLK);		 //CLK  �� 1bit������ʱ�ӵ������ز���
		dat >>= 1;	 
		nop_delay_xus(1);
	}
	GPIO_ClearBit(TM1624_CLK);
	GPIO_ClearBit(TM1624_DIN);
}
/********************************************************
**	������	uint8_t TM1624_ReadByte(void)
**	����	TM1624��ȡһ���ֽ�
**	����	������
**	����	����
*********************************************************/
static uint8_t TM1624_ReadByte(void)
{
	uint8_t i;
	uint8_t dat;
	for(i = 0;i < 8;i++)
	{	
		dat >>= 1;	 
		GPIO_ClearBit(TM1624_CLK);  	 //CLK  ��
		nop_delay_xus(1);
		GPIO_SetBit(TM1624_CLK);		 //CLK  �� 1bit������ʱ�ӵ������ز���
		nop_delay_xus(1);
		if(GPIO_ReadBit(TM1624_DIN)) 
			dat |= 0X80;
		nop_delay_xus(1);
	}
	GPIO_ClearBit(TM1624_CLK);
	return dat;
}
/********************************************************
**	������	void TM1624_SendCommand(uint8_t cmd)
**	����	TM1624����һ������
**	����	������
**	����	����
*********************************************************/
static void TM1624_SendCommand(uint8_t cmd)  //���������ֽ�
{ 
   GPIO_SetBit(TM1624_STB);					  //STB�ø� PC5 
   nop_delay_xus(1);
   GPIO_ClearBit(TM1624_STB);					  //STB�õ�  
   TM1624_SendByte(cmd);			      //����8bit����
}
/********************************************************
**	������	void TM1624_Display_Update(uint8_t *display,uint8_t num)
**	����	TM1624ˢ��
**	����	��display��ʾ������ָ�� num��ʾ������ Config����ָ��������ȵ����Լ�������ʾ
**	����	����
*********************************************************/
void TM1624_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
{
	uint8_t i;
	TM1624_SendCommand(TM1624_DISP_COM);//��ʾģʽ����(00)  
    TM1624_SendCommand(0x40);	        //������������(01)������ģʽ����Ϊ��ͨģʽ(0)--��ַģʽ����Ϊ��ַ�Լ�(0)--��дģʽ(00)     (����01 -- 0 0 00)
    TM1624_SendCommand(0xC0);	        //��ַ�趨(11)��������ʾģʽ,��00H��ʼ(0000)   (����11 -- 0000) 
	for(i = 0;i < num;i++,display++)
	{
		TM1624_SendByte(*display);
	}
	TM1624_SendCommand(0x80 | Config);	  //��ʾ����ָ������
}
/********************************************************
**	������	uint8_t TM1624_Read_Key(uint8_t *data)
**	����	TM1624��ȡ����
**	����	��data ����ָ��
**	����	����
*********************************************************/
uint8_t TM1624_Read_Key(uint8_t *data)
{
	uint8_t *temp = data;
	TM1624_SendCommand(0x42);	        //��ַ�趨(11)��������ʾģʽ,��00H��ʼ(0000)   (����11 -- 0000) 
	nop_delay_xus(1);
	GPIO_ClearBit(TM1624_CLK);  	 //CLK  ��
	GPIO_Init(TM1624_DIN,INPUT|PULL_UP); 
	nop_delay_xus(1);
	for(uint8_t i = 0;i < 5;i++,temp++)
	{
		*temp = TM1624_ReadByte();
	}
	GPIO_Init(TM1624_DIN,OUTPUT|PUSHPULL); 
	GPIO_ClearBit(TM1624_DIN);
	return 1;
}
#endif

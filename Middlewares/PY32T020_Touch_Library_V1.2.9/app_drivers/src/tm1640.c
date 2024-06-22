#include "tm1640.h"
#if APP_TM1640_ENABLE
/********************************************************
**	函数名	void TM1640_Init(void)
**	描述	TM1640初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TM1640_Init(void)
{
	GPIO_Init(TM1640_CLK,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1640_CLK);
	GPIO_Init(TM1640_DIN,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1640_DIN);
}
/********************************************************
**	函数名	void TM1640_Start(void)
**	描述	TM1624发送开始信号
**	传入	：无
**	返回	：无
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
**	函数名	void TM1640_Stop(void)
**	描述	TM1624发送停止信号
**	传入	：无
**	返回	：无
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
**	函数名	void TM1640_SendByte(uint8_t data)
**	描述	TM1624发送1个字节
**	传入	：data 待发送的数据
**	返回	：无
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
**	函数名	void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
**	描述	TM1640刷新
**	传入	：display显示的数据指针 num显示的数量 Config命令指令，用于亮度调节以及开关显示
**	返回	：无
*********************************************************/
void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
{
	uint8_t i;
	TM1640_Start();
	TM1640_SendByte(0X40);				//设置地址自加模式
	TM1640_Stop();
	
	TM1640_Start();
	TM1640_SendByte(0XC0);				//发送首地址
	for(i = 0;i < num;i++,display++)
	{
		TM1640_SendByte(*display);
	}
	TM1640_Stop();
	
	TM1640_Start();
	TM1640_SendByte(0X80 | Config);		//设置亮度
	TM1640_Stop();
}

#endif

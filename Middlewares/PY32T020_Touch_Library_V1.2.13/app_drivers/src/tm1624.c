#include "tm1624.h"
#if APP_TM1624_ENABLE
/********************************************************
**	函数名	void TM1624_Init(void)
**	描述	TM1624初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TM1624_Init(void)
{
	GPIO_Init(TM1624_CLK,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1624_CLK);
	GPIO_Init(TM1624_DIN,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1624_DIN);
	GPIO_Init(TM1624_STB,OUTPUT|PUSHPULL); GPIO_ClearBit(TM1624_STB);
}
/********************************************************
**	函数名	void TM1624_SendByte(uint8_t dat)
**	描述	TM1624发送一个字节
**	传入	：数据
**	返回	：无
*********************************************************/
static void TM1624_SendByte(uint8_t dat)
{
	uint8_t i;
	for(i = 0;i < 8;i++)
	{	
		GPIO_ClearBit(TM1624_CLK);  	 //CLK  低
		if(dat & 0X01) 
			GPIO_SetBit(TM1624_DIN);     //DIN  高
		else         
			GPIO_ClearBit(TM1624_DIN);   //DIN  低
		nop_delay_xus(1);
		GPIO_SetBit(TM1624_CLK);		 //CLK  高 1bit数据在时钟的上升沿操作
		dat >>= 1;	 
		nop_delay_xus(1);
	}
	GPIO_ClearBit(TM1624_CLK);
	GPIO_ClearBit(TM1624_DIN);
}
/********************************************************
**	函数名	uint8_t TM1624_ReadByte(void)
**	描述	TM1624读取一个字节
**	传入	：数据
**	返回	：无
*********************************************************/
static uint8_t TM1624_ReadByte(void)
{
	uint8_t i;
	uint8_t dat;
	for(i = 0;i < 8;i++)
	{	
		dat >>= 1;	 
		GPIO_ClearBit(TM1624_CLK);  	 //CLK  低
		nop_delay_xus(1);
		GPIO_SetBit(TM1624_CLK);		 //CLK  高 1bit数据在时钟的上升沿操作
		nop_delay_xus(1);
		if(GPIO_ReadBit(TM1624_DIN)) 
			dat |= 0X80;
		nop_delay_xus(1);
	}
	GPIO_ClearBit(TM1624_CLK);
	return dat;
}
/********************************************************
**	函数名	void TM1624_SendCommand(uint8_t cmd)
**	描述	TM1624发送一个命令
**	传入	：命令
**	返回	：无
*********************************************************/
static void TM1624_SendCommand(uint8_t cmd)  //发送命令字节
{ 
   GPIO_SetBit(TM1624_STB);					  //STB置高 PC5 
   nop_delay_xus(1);
   GPIO_ClearBit(TM1624_STB);					  //STB置低  
   TM1624_SendByte(cmd);			      //发送8bit数据
}
/********************************************************
**	函数名	void TM1624_Display_Update(uint8_t *display,uint8_t num)
**	描述	TM1624刷新
**	传入	：display显示的数据指针 num显示的数量 Config命令指令，用于亮度调节以及开关显示
**	返回	：无
*********************************************************/
void TM1624_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
{
	uint8_t i;
	TM1624_SendCommand(TM1624_DISP_COM);//显示模式设置(00)  
    TM1624_SendCommand(0x40);	        //数据命令设置(01)：测试模式设置为普通模式(0)--地址模式设置为地址自加(0)--读写模式(00)     (即：01 -- 0 0 00)
    TM1624_SendCommand(0xC0);	        //地址设定(11)：设置显示模式,从00H开始(0000)   (即：11 -- 0000) 
	for(i = 0;i < num;i++,display++)
	{
		TM1624_SendByte(*display);
	}
	TM1624_SendCommand(0x80 | Config);	  //显示控制指令设置
}
/********************************************************
**	函数名	uint8_t TM1624_Read_Key(uint8_t *data)
**	描述	TM1624读取按键
**	传入	：data 按键指针
**	返回	：无
*********************************************************/
uint8_t TM1624_Read_Key(uint8_t *data)
{
	uint8_t *temp = data;
	TM1624_SendCommand(0x42);	        //地址设定(11)：设置显示模式,从00H开始(0000)   (即：11 -- 0000) 
	nop_delay_xus(1);
	GPIO_ClearBit(TM1624_CLK);  	 //CLK  低
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

#ifndef _APP_IR_RECEIVED_H_
#define _APP_IR_RECEIVED_H_
#include "app_config.h"


#if APP_IR_RECEIVED_ENABLE

typedef struct
{
    uint8_t ir_address;  				
    uint8_t ir_command; 	
	uint8_t ir_count;
} Ir_TypeDef;



typedef enum 
{
	/*	引导码高电平脉宽	*/
	SYNC_MIN_TIME	= 4000/D_IR_sample,	
	SYNC_MAX_TIME	= 5000/D_IR_sample,	
	/*	数据码1高电平脉宽	*/
	DATA1_MIN_TIME	= 1495/D_IR_sample,	
	DATA1_MAX_TIME	= 1895/D_IR_sample,	
	/*	数据码0高电平脉宽	*/
	DATA0_MIN_TIME	= 365/D_IR_sample,	
	DATA0_MAX_TIME	= 765/D_IR_sample,
	/*	长按码高电平脉宽	*/
	LONG_MIN_TIME	= 2000/D_IR_sample,	
	LONG_MAX_TIME	= 3000/D_IR_sample,		

	RELEASE_TIME    = 200000/D_IR_sample,
}TIMEEnum;
/********************************************************
**	函数名	void IR_Received_Init(void)
**	描述	红外扫描解码初始化
**	传入	：无
**	返回	：无
*********************************************************/
void IR_Received_Init(void);
/********************************************************
**	函数名	void IR_Received_Scan(void)
**	描述	红外解码电平扫描函数，放在定时器中断内
**	传入	：无
**	返回	：无
*********************************************************/
void IR_Received_Scan(void);
/********************************************************
**	函数名	uint8_t IR_Press(Ir_TypeDef *remote)
**	描述	红外解码函数
**	传入	：remote 数据接收结构体
**	返回	：	0：无信号输入
				1：收到正确的红外信号
*********************************************************/
uint8_t IR_Press(Ir_TypeDef *remote);

#endif

#endif

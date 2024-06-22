#ifndef _TM1624_H_
#define _TM1624_H_

#include "app_config.h"
#if APP_TM1624_ENABLE
#define TM1624_ON		0X08
#define TM1624_OFF		0X00
/********************************************************
**	函数名	void TM1624_Init(void)
**	描述	TM1624初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TM1624_Init(void);
/********************************************************
**	函数名	void TM1624_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
**	描述	TM1624刷新
**	传入	：display显示的数据指针 num显示的数量 Config命令指令，用于亮度调节以及开关显示
**	返回	：无
*********************************************************/
void TM1624_Display_Update(uint8_t *display,uint8_t num,uint8_t Config);
/********************************************************
**	函数名	uint8_t TM1624_Read_Key(uint8_t *data)
**	描述	TM1624读取按键
**	传入	：data 按键指针
**	返回	：无
*********************************************************/
uint8_t TM1624_Read_Key(uint8_t *data);
#endif

#endif

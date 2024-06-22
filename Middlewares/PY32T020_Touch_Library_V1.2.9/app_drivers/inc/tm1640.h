#ifndef _TM1640_H_
#define _TM1640_H_

#include "app_config.h"
#if APP_TM1640_ENABLE
#define TM1640_ON		0X08
#define TM1640_OFF		0X00
/********************************************************
**	函数名	void TM1640_Init(void)
**	描述	TM1640初始化
**	传入	：无
**	返回	：无
*********************************************************/
void TM1640_Init(void);
/********************************************************
**	函数名	void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
**	描述	TM1640刷新
**	传入	：display显示的数据指针 num显示的数量 Config命令指令，用于亮度调节以及开关显示
**	返回	：无
*********************************************************/
void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config);
#endif

#endif


#ifndef _TM1640_H_
#define _TM1640_H_

#include "app_config.h"
#if APP_TM1640_ENABLE
#define TM1640_ON		0X08
#define TM1640_OFF		0X00
/********************************************************
**	������	void TM1640_Init(void)
**	����	TM1640��ʼ��
**	����	����
**	����	����
*********************************************************/
void TM1640_Init(void);
/********************************************************
**	������	void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
**	����	TM1640ˢ��
**	����	��display��ʾ������ָ�� num��ʾ������ Config����ָ��������ȵ����Լ�������ʾ
**	����	����
*********************************************************/
void TM1640_Display_Update(uint8_t *display,uint8_t num,uint8_t Config);
#endif

#endif


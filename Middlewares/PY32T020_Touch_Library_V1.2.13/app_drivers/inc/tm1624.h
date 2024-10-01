#ifndef _TM1624_H_
#define _TM1624_H_

#include "app_config.h"
#if APP_TM1624_ENABLE
#define TM1624_ON		0X08
#define TM1624_OFF		0X00
/********************************************************
**	������	void TM1624_Init(void)
**	����	TM1624��ʼ��
**	����	����
**	����	����
*********************************************************/
void TM1624_Init(void);
/********************************************************
**	������	void TM1624_Display_Update(uint8_t *display,uint8_t num,uint8_t Config)
**	����	TM1624ˢ��
**	����	��display��ʾ������ָ�� num��ʾ������ Config����ָ��������ȵ����Լ�������ʾ
**	����	����
*********************************************************/
void TM1624_Display_Update(uint8_t *display,uint8_t num,uint8_t Config);
/********************************************************
**	������	uint8_t TM1624_Read_Key(uint8_t *data)
**	����	TM1624��ȡ����
**	����	��data ����ָ��
**	����	����
*********************************************************/
uint8_t TM1624_Read_Key(uint8_t *data);
#endif

#endif

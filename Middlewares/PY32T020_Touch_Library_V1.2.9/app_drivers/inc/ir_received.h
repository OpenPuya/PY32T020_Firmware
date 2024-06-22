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
	/*	������ߵ�ƽ����	*/
	SYNC_MIN_TIME	= 4000/D_IR_sample,	
	SYNC_MAX_TIME	= 5000/D_IR_sample,	
	/*	������1�ߵ�ƽ����	*/
	DATA1_MIN_TIME	= 1495/D_IR_sample,	
	DATA1_MAX_TIME	= 1895/D_IR_sample,	
	/*	������0�ߵ�ƽ����	*/
	DATA0_MIN_TIME	= 365/D_IR_sample,	
	DATA0_MAX_TIME	= 765/D_IR_sample,
	/*	������ߵ�ƽ����	*/
	LONG_MIN_TIME	= 2000/D_IR_sample,	
	LONG_MAX_TIME	= 3000/D_IR_sample,		

	RELEASE_TIME    = 200000/D_IR_sample,
}TIMEEnum;
/********************************************************
**	������	void IR_Received_Init(void)
**	����	����ɨ������ʼ��
**	����	����
**	����	����
*********************************************************/
void IR_Received_Init(void);
/********************************************************
**	������	void IR_Received_Scan(void)
**	����	��������ƽɨ�躯�������ڶ�ʱ���ж���
**	����	����
**	����	����
*********************************************************/
void IR_Received_Scan(void);
/********************************************************
**	������	uint8_t IR_Press(Ir_TypeDef *remote)
**	����	������뺯��
**	����	��remote ���ݽ��սṹ��
**	����	��	0�����ź�����
				1���յ���ȷ�ĺ����ź�
*********************************************************/
uint8_t IR_Press(Ir_TypeDef *remote);

#endif

#endif

#ifndef _UART_DRIVERS_H_
#define _UART_DRIVERS_H_

#include "app_config.h"

#if APP_UART_ENABLE
typedef struct SqQueue
{
    uint8_t *base;
    uint8_t front, rear;
    uint8_t sq_size;
} SqQueue;
#if UART1_ENABLE
/********************************************************
**	������	void UART1_Init(uint32_t BaudRate)
**	����	��UART1��ʼ������
**	����	��BaudRate ������
**	����	��	��
*********************************************************/												
void UART1_Init(uint32_t BaudRate);
#if (UART1_RX_PIN != NO_PIN)
/********************************************************
**	������	uint8_t UART1_QueueRead(uint8_t *data)
**	����	��UART1��ȡ����
**	����	��data����������ָ��
**	����	��	0 ����Ϊ��
				1 ������Ч
*********************************************************/	
uint8_t UART1_QueueRead(uint8_t *data);
#endif
#if (UART1_TX_PIN != NO_PIN)
/********************************************************
**	������	uint8_t UART1_QueueSend(uint8_t data)
**	����	��UART1��������
**	����	��data�����͵�����
**	����	��	0 ����ʧ��
				1 ���ͳɹ�
*********************************************************/	
uint8_t UART1_QueueSend(uint8_t data);
#endif
#endif
#if UART2_ENABLE
/********************************************************
**	������	void UART2_Init(uint32_t BaudRate)
**	����	��UART2��ʼ������
**	����	��BaudRate ������
**	����	��	��
*********************************************************/	
void UART2_Init(uint32_t BaudRate);
#if (UART2_RX_PIN != NO_PIN)
/********************************************************
**	������	uint8_t UART2_QueueRead(uint8_t *data)
**	����	��UART2��ȡ����
**	����	��data����������ָ��
**	����	��	0 ����Ϊ��
				1 ������Ч
*********************************************************/	
uint8_t UART2_QueueRead(uint8_t *data);
#endif
#if (UART2_TX_PIN != NO_PIN)
/********************************************************
**	������	uint8_t UART2_QueueSend(uint8_t data)
**	����	��UART2��������
**	����	��data�����͵�����
**	����	��	0 ����ʧ��
				1 ���ͳɹ�
*********************************************************/	
uint8_t UART2_QueueSend(uint8_t data);
#endif
#endif
#if UART3_ENABLE
/********************************************************
**	������	void UART3_Init(uint32_t BaudRate)
**	����	��UART3��ʼ������
**	����	��BaudRate ������
**	����	��	��
*********************************************************/	
void UART3_Init(uint32_t BaudRate);
#if (UART3_RX_PIN != NO_PIN)
/********************************************************
**	������	uint8_t UART3_QueueRead(uint8_t *data)
**	����	��UART3��ȡ����
**	����	��data����������ָ��
**	����	��	0 ����Ϊ��
				1 ������Ч
*********************************************************/	
uint8_t UART3_QueueRead(uint8_t *data);
#endif
#if (UART3_TX_PIN != NO_PIN)
/********************************************************
**	������	uint8_t UART3_QueueSend(uint8_t data)
**	����	��UART3��������
**	����	��data�����͵�����
**	����	��	0 ����ʧ��
				1 ���ͳɹ�
*********************************************************/	
uint8_t UART3_QueueSend(uint8_t data);
#endif
#endif
#endif
#endif

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
**	函数名	void UART1_Init(uint32_t BaudRate)
**	描述	：UART1初始化函数
**	传入	：BaudRate 波特率
**	返回	：	无
*********************************************************/												
void UART1_Init(uint32_t BaudRate);
#if (UART1_RX_PIN != NO_PIN)
/********************************************************
**	函数名	uint8_t UART1_QueueRead(uint8_t *data)
**	描述	：UART1读取数据
**	传入	：data：接收数据指针
**	返回	：	0 数据为空
				1 数据有效
*********************************************************/	
uint8_t UART1_QueueRead(uint8_t *data);
#endif
#if (UART1_TX_PIN != NO_PIN)
/********************************************************
**	函数名	uint8_t UART1_QueueSend(uint8_t data)
**	描述	：UART1发送数据
**	传入	：data：发送的数据
**	返回	：	0 发送失败
				1 发送成功
*********************************************************/	
uint8_t UART1_QueueSend(uint8_t data);
#endif
#endif
#if UART2_ENABLE
/********************************************************
**	函数名	void UART2_Init(uint32_t BaudRate)
**	描述	：UART2初始化函数
**	传入	：BaudRate 波特率
**	返回	：	无
*********************************************************/	
void UART2_Init(uint32_t BaudRate);
#if (UART2_RX_PIN != NO_PIN)
/********************************************************
**	函数名	uint8_t UART2_QueueRead(uint8_t *data)
**	描述	：UART2读取数据
**	传入	：data：接收数据指针
**	返回	：	0 数据为空
				1 数据有效
*********************************************************/	
uint8_t UART2_QueueRead(uint8_t *data);
#endif
#if (UART2_TX_PIN != NO_PIN)
/********************************************************
**	函数名	uint8_t UART2_QueueSend(uint8_t data)
**	描述	：UART2发送数据
**	传入	：data：发送的数据
**	返回	：	0 发送失败
				1 发送成功
*********************************************************/	
uint8_t UART2_QueueSend(uint8_t data);
#endif
#endif
#if UART3_ENABLE
/********************************************************
**	函数名	void UART3_Init(uint32_t BaudRate)
**	描述	：UART3初始化函数
**	传入	：BaudRate 波特率
**	返回	：	无
*********************************************************/	
void UART3_Init(uint32_t BaudRate);
#if (UART3_RX_PIN != NO_PIN)
/********************************************************
**	函数名	uint8_t UART3_QueueRead(uint8_t *data)
**	描述	：UART3读取数据
**	传入	：data：接收数据指针
**	返回	：	0 数据为空
				1 数据有效
*********************************************************/	
uint8_t UART3_QueueRead(uint8_t *data);
#endif
#if (UART3_TX_PIN != NO_PIN)
/********************************************************
**	函数名	uint8_t UART3_QueueSend(uint8_t data)
**	描述	：UART3发送数据
**	传入	：data：发送的数据
**	返回	：	0 发送失败
				1 发送成功
*********************************************************/	
uint8_t UART3_QueueSend(uint8_t data);
#endif
#endif
#endif
#endif

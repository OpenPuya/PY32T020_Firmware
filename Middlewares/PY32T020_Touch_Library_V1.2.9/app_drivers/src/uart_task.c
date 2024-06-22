#include "drivers.h"
#if APP_UART_ENABLE
#if ((UART1_RX_PIN != NO_PIN) && UART1_ENABLE)
uint8_t rx1_timeout;
#endif
#if ((UART2_RX_PIN != NO_PIN) && UART2_ENABLE)
uint8_t rx2_timeout;
#endif
#if ((UART3_RX_PIN != NO_PIN) && UART3_ENABLE)
uint8_t rx3_timeout;
#endif
/********************************************************
**	函数名	void UART_TimeOut(void)
**	描述	UART接收超时定时器，调用频率1ms
**	传入	：无
**	返回	：无
*********************************************************/
void UART_TimeOut(void)
{
	#if ((UART1_RX_PIN != NO_PIN) && UART1_ENABLE)
	if(rx1_timeout)
		rx1_timeout--;
	#endif
	#if ((UART2_RX_PIN != NO_PIN) && UART2_ENABLE)
	if(rx2_timeout)
		rx2_timeout--;
	#endif
	#if ((UART3_RX_PIN != NO_PIN) && UART3_ENABLE)
	if(rx3_timeout)
		rx3_timeout--;
	#endif
}
/********************************************************
**	函数名	void UART_Loop(void)
**	描述	UART任务处理，轮询UART接收队列，当接收空闲超过5ms后表示接收结束，回发收到的数据
**	传入	：无
**	返回	：无
*********************************************************/
void UART_Loop(void)
{
	#if ((UART1_RX_PIN != NO_PIN) && UART1_ENABLE)
	static uint8_t rx1_data[64];
	static uint8_t rx1_count = 0;
	#endif
	#if ((UART2_RX_PIN != NO_PIN) && UART2_ENABLE)
	static uint8_t rx2_data[64];
	static uint8_t rx2_count = 0;
	#endif
	#if ((UART3_RX_PIN != NO_PIN) && UART3_ENABLE)
	static uint8_t rx3_data[64];
	static uint8_t rx3_count = 0;
	#endif
	#if ((UART1_RX_PIN != NO_PIN) && UART1_ENABLE)
	while(UART1_QueueRead(&rx1_data[rx1_count]))
	{
		rx1_count++;
		rx1_timeout = 5;
		if (rx1_count == 64)
			break;
	}
	if ((rx1_count && rx1_timeout == 0) || rx1_count == 64)
	{
		#if (UART1_TX_PIN != NO_PIN)
		/*	串口接收完成	*/
		for(uint8_t n = 0;n < rx1_count;n++)
		{
			UART1_QueueSend(rx1_data[n]);
		}
		#endif
		rx1_count = 0;
	}
	#endif
	#if ((UART2_RX_PIN != NO_PIN) && UART2_ENABLE)
	while(UART2_QueueRead(&rx2_data[rx2_count]))
	{
		rx2_count++;
		rx2_timeout = 5;
		if (rx2_count == 64)
			break;
	}
	if ((rx2_count && rx2_timeout == 0) || rx2_count == 64)
	{
		#if (UART2_TX_PIN != NO_PIN)
		/*	串口接收完成	*/
		for(uint8_t n = 0;n < rx2_count;n++)
		{
			UART2_QueueSend(rx2_data[n]);
		}
		#endif
		rx2_count = 0;
	}
	#endif
	#if ((UART3_RX_PIN != NO_PIN) && UART3_ENABLE)
	while(UART3_QueueRead(&rx3_data[rx3_count]))
	{
		rx3_count++;
		rx3_timeout = 5;
		if (rx3_count == 64)
			break;
	}
	if ((rx3_count && rx3_timeout == 0) || rx3_count == 64)
	{
		#if (UART3_TX_PIN != NO_PIN)
		/*	串口接收完成	*/
		for(uint8_t n = 0;n < rx3_count;n++)
		{
			UART3_QueueSend(rx3_data[n]);
		}
		#endif
		rx3_count = 0;
	}
	#endif
}
#endif

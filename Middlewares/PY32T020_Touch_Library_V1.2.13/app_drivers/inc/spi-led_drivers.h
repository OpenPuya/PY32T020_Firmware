#ifndef _SPI_LED_DRIVERS_H_
#define _SPI_LED_DRIVERS_H_
#include "app_config.h"
#if APP_SPI_LED_ENABLE
/********************************************************
**	函数名	void SPI_LED_Init(void)
**	描述	SPI初始化，用于驱动类W2812灯珠
**	传入	：	无
**	返回	：	无
*********************************************************/
void SPI_LED_Init(void);
/********************************************************
**	函数名	void SPI_LED_RgbLoad(uint8_t offset, uint8_t red, uint8_t green, uint8_t blue)
**	描述	将RGB数据写入缓存中
**	传入	：	offset：灯珠偏移量	red:红灯PWM值	green:绿灯PWM值	blue:蓝灯PWM值
**	返回	：	无
*********************************************************/
void SPI_LED_RgbLoad(uint8_t offset, uint8_t red, uint8_t green, uint8_t blue);
/********************************************************
**	函数名	void SPI_LED_Transmit(void)
**	描述	将缓存数据通过SPI发送
**	传入	：	无
**	返回	：	无
*********************************************************/
void SPI_LED_Transmit(void);


#endif
#endif

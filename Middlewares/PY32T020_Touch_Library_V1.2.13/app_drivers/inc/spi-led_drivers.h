#ifndef _SPI_LED_DRIVERS_H_
#define _SPI_LED_DRIVERS_H_
#include "app_config.h"
#if APP_SPI_LED_ENABLE
/********************************************************
**	������	void SPI_LED_Init(void)
**	����	SPI��ʼ��������������W2812����
**	����	��	��
**	����	��	��
*********************************************************/
void SPI_LED_Init(void);
/********************************************************
**	������	void SPI_LED_RgbLoad(uint8_t offset, uint8_t red, uint8_t green, uint8_t blue)
**	����	��RGB����д�뻺����
**	����	��	offset������ƫ����	red:���PWMֵ	green:�̵�PWMֵ	blue:����PWMֵ
**	����	��	��
*********************************************************/
void SPI_LED_RgbLoad(uint8_t offset, uint8_t red, uint8_t green, uint8_t blue);
/********************************************************
**	������	void SPI_LED_Transmit(void)
**	����	����������ͨ��SPI����
**	����	��	��
**	����	��	��
*********************************************************/
void SPI_LED_Transmit(void);


#endif
#endif

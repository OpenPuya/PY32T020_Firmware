#include "spi-led_drivers.h"
#if APP_SPI_LED_ENABLE
#define WS2812_1_CODE 		0X0C
#define WS2812_0_CODE 		0X08
static uint16_t rgb_data[SPI_LED_CNT * 6];
union {
    uint16_t halfword;
    struct
    {
        uint8_t bit0 : 4;
        uint8_t bit1 : 4;
        uint8_t bit2 : 4;
        uint8_t bit3 : 4;
    } halfbyte;
} argb_spi;
/********************************************************
**	函数名	void SPI_LED_Init(void)
**	描述	SPI初始化，用于驱动类W2812灯珠
**	传入	：	无
**	返回	：	无
*********************************************************/
void SPI_LED_Init(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	/* Enable SPI1 clock */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
	/* Set SPI1 features */
	SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	if (SystemCoreClock == 48000000l)
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
	else
		SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	LL_SPI_Init(SPI1, &SPI_InitStruct);
	LL_SPI_Enable(SPI1);
	 /* MOSI*/
    GPIO_Init(SPI_LED_PIN, ALTERNATE | PUSHPULL | GPIO_SPI1);
	for (uint16_t i = 0; i < SPI_LED_CNT; i++)
    {
        SPI_LED_RgbLoad(i, 0, 0, 0);
    }
    SPI_LED_Transmit();
}
/********************************************************
**	函数名	void SPI_LED_RgbLoad(uint8_t offset, uint8_t red, uint8_t green, uint8_t blue)
**	描述	将RGB数据写入缓存中
**	传入	：	offset：灯珠偏移量	red:红灯PWM值	green:绿灯PWM值	blue:蓝灯PWM值
**	返回	：	无
*********************************************************/
void SPI_LED_RgbLoad(uint8_t offset, uint8_t red, uint8_t green, uint8_t blue)
{
    uint16_t n = offset * 6;
    if (offset >= SPI_LED_CNT)
        return;
    /*	绿色PWM填充	*/
    argb_spi.halfbyte.bit3 = (green & 0x80) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit2 = (green & 0x40) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit1 = (green & 0x20) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit0 = (green & 0x10) ? WS2812_1_CODE : WS2812_0_CODE;
    rgb_data[n++] = argb_spi.halfword;
    argb_spi.halfbyte.bit3 = (green & 0x08) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit2 = (green & 0x04) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit1 = (green & 0x02) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit0 = (green & 0x01) ? WS2812_1_CODE : WS2812_0_CODE;
    rgb_data[n++] = argb_spi.halfword;
    /*	红色PWM填充	*/
    argb_spi.halfbyte.bit3 = (red & 0x80) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit2 = (red & 0x40) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit1 = (red & 0x20) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit0 = (red & 0x10) ? WS2812_1_CODE : WS2812_0_CODE;
    rgb_data[n++] = argb_spi.halfword;
    argb_spi.halfbyte.bit3 = (red & 0x08) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit2 = (red & 0x04) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit1 = (red & 0x02) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit0 = (red & 0x01) ? WS2812_1_CODE : WS2812_0_CODE;
    rgb_data[n++] = argb_spi.halfword;
    /*	蓝色PWM填充	*/
    argb_spi.halfbyte.bit3 = (blue & 0x80) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit2 = (blue & 0x40) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit1 = (blue & 0x20) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit0 = (blue & 0x10) ? WS2812_1_CODE : WS2812_0_CODE;
    rgb_data[n++] = argb_spi.halfword;
    argb_spi.halfbyte.bit3 = (blue & 0x08) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit2 = (blue & 0x04) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit1 = (blue & 0x02) ? WS2812_1_CODE : WS2812_0_CODE;
    argb_spi.halfbyte.bit0 = (blue & 0x01) ? WS2812_1_CODE : WS2812_0_CODE;
    rgb_data[n++] = argb_spi.halfword;
}
/********************************************************
**	函数名	void SPI_LED_Transmit(void)
**	描述	将缓存数据通过SPI发送
**	传入	：	无
**	返回	：	无
*********************************************************/
void SPI_LED_Transmit(void)
{
	for(uint16_t i = 0;i < SPI_LED_CNT * 6;i++)
	{
		while(LL_SPI_IsActiveFlag_TXE(SPI1) == 0)
		{
			;
		}
		LL_SPI_TransmitData16(SPI1, rgb_data[i]);
	}
}
#endif

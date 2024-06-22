#include "drivers.h"
#if APP_SMG_ENABLE
extern uint8_t smg_data[COM_COUNT];
/**
 * @brief  段码表
 */
const uint8_t num_tbl[] = {
    0X3F, 0X06, 0X5B, 0X4F, 0X66, 0X6D, 0X7D, 0X07, 0X7F, 0X6F,
};
/**
 * @brief  TK按键对应LED
 */
const uint8_t TK_LED[] = {
    (1 << 3), (1 << 2), (1 << 1), (1 << 6), (1 << 4), (1 << 5), (1 << 0), (1 << 7)
};
/********************************************************
**	函数名	void SMG_Default(void)
**	描述	：设置上电默认显示内容
**	传入	：无
**	返回	：无
*********************************************************/
void SMG_Default(void)
{
    /*	默认显示		*/
	smg_data[0] = 0X40;
	smg_data[1] = 0X40;
	smg_data[2] = 0X00;
}
/********************************************************
**	函数名	void SMG_Show(uint8_t num)
**	描述	：使用数码管显示10进制数字
**	传入	：num	需要显示的数
**	返回	：无
*********************************************************/
void SMG_Show(uint16_t num)
{
    smg_data[0] = num_tbl[num / 10];
    smg_data[1] = num_tbl[num % 10];
}
/********************************************************
**	函数名	void LED_Show(uint8_t led_bit)
**	描述	：按键LED指示
**	传入	：led_bit	需要显示的bit位置，
                        bit0表示TK0的LED,
                        bit1表示TK1的LED,
                        ......
**	返回	：无
*********************************************************/
void LED_Show(uint16_t led_bit)
{
    uint8_t i;
    uint8_t bit = 0;
    for (i = 0; i < SEG_COUNT; i++)
    {
        if (led_bit & 0X01)
            bit |= TK_LED[i];
        led_bit >>= 1;
    }
    smg_data[2] = bit;
}

#endif

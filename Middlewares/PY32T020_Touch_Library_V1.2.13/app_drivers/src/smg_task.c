#include "drivers.h"
#if APP_SMG_ENABLE
extern uint8_t smg_data[COM_COUNT];
/**
 * @brief  �����
 */
const uint8_t num_tbl[] = {
    0X3F, 0X06, 0X5B, 0X4F, 0X66, 0X6D, 0X7D, 0X07, 0X7F, 0X6F,
};
/**
 * @brief  TK������ӦLED
 */
const uint8_t TK_LED[] = {
    (1 << 3), (1 << 2), (1 << 1), (1 << 6), (1 << 4), (1 << 5), (1 << 0), (1 << 7)
};
/********************************************************
**	������	void SMG_Default(void)
**	����	�������ϵ�Ĭ����ʾ����
**	����	����
**	����	����
*********************************************************/
void SMG_Default(void)
{
    /*	Ĭ����ʾ		*/
	smg_data[0] = 0X40;
	smg_data[1] = 0X40;
	smg_data[2] = 0X00;
}
/********************************************************
**	������	void SMG_Show(uint8_t num)
**	����	��ʹ���������ʾ10��������
**	����	��num	��Ҫ��ʾ����
**	����	����
*********************************************************/
void SMG_Show(uint16_t num)
{
    smg_data[0] = num_tbl[num / 10];
    smg_data[1] = num_tbl[num % 10];
}
/********************************************************
**	������	void LED_Show(uint8_t led_bit)
**	����	������LEDָʾ
**	����	��led_bit	��Ҫ��ʾ��bitλ�ã�
                        bit0��ʾTK0��LED,
                        bit1��ʾTK1��LED,
                        ......
**	����	����
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

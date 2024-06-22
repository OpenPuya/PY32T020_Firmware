#include "user-otp_drivers.h"
#if APP_USER_OTP_ENABLE
#define FLASH_USER_START_ADDR     0X1FFF0280
static uint32_t USER_DATA[32];
static uint8_t flash_flag = 0;
/********************************************************
**	������	void User_Flash_Write(void)
**	����	������������д��FLASH��
**	����	��	��
**	����	��	0������ʧ��
				1������ɹ�
*********************************************************/
uint8_t User_Flash_Write(void)
{
	/*	����д��FLASH	������д������Ҫ5ms	*/
	if(flash_flag & 0X02)
	{
		/* Unlock Flash */
		FLASH_Unlock();
		Erase_UserData(FLASH_USER_START_ADDR);
		Program_UserData(FLASH_USER_START_ADDR,USER_DATA);
		FLASH_Lock();
		uint32_t addr = FLASH_USER_START_ADDR;
		for(uint8_t i = 0;i < 32;i++,addr += 4)		//У��
		{
			if(USER_DATA[i] != HW32_REG(addr))
				return 0;
		}
	}
	flash_flag &= ~0X02;
	return 1;
}
/********************************************************
**	������	uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len)
**	����	�ӻ����ж�ȡ���ݣ��ܹ�124���ֽڿ�ʹ��
**	����	��	offset��ƫ����	data:��ȡ������ָ�룬len:��ȡ�����ݳ���
**	����	��	0��������Ч�򳤶ȳ���
				1�����ݶ�ȡ�ɹ�
*********************************************************/
uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len)
{
	/*	��FLASH���ݶ��뻺��*/
	if((flash_flag & 0X01) == 0X00)
	{
		uint32_t addr = FLASH_USER_START_ADDR;
		for(uint8_t i = 0;i < 32;i++,addr += 4)
		{
			USER_DATA[i] = HW32_REG(addr);
		}
		flash_flag |= 0X01;
	}
	/*	δ��ȡ��У������		*/
	if((USER_DATA[0] & 0XFFFF0000) != 0XA5A50000)
	{
		return 0;
	}
	if((offset + len) > 124)
		return 0;
	uint8_t *copy_data = (uint8_t *)&USER_DATA[1];
	copy_data += offset;
	for(uint8_t i = 0;i < len;i++)
	{
		*data = *copy_data;
		data++;
		copy_data++;
	}
	return 1;
}
/********************************************************
**	������	uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len)
**	����	д�����ݵ������У��ܹ�124���ֽڿ�ʹ��
**	����	��	offset��ƫ����	data:д�������ָ�룬len:��ȡ�����ݳ���
**	����	��	0�����ȳ���
				1������д��ɹ�
*********************************************************/
uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len)
{
	if((offset + len) > 124)
		return 0;
	uint8_t *copy_data = (uint8_t *)&USER_DATA[1];
	copy_data += offset;
	/*	���ݸ���	*/
	for(uint8_t i = 0;i < len;i++)
	{
		if(*copy_data != *data)
			flash_flag |= 0X02;
		*copy_data = *data;
		data++;
		copy_data++;
	}
	/*	д��У������		*/
	USER_DATA[0] &= ~0XFFFF0000;
	USER_DATA[0] |= 0XA5A50000;
	return 1;
}
#endif

#include "user-otp_drivers.h"
#if APP_USER_OTP_ENABLE
#define FLASH_USER_START_ADDR     0X1FFF0280
static uint32_t USER_DATA[32];
static uint8_t flash_flag = 0;
/********************************************************
**	函数名	void User_Flash_Write(void)
**	描述	将缓存内数据写入FLASH中
**	传入	：	无
**	返回	：	0：保存失败
				1：保存成功
*********************************************************/
uint8_t User_Flash_Write(void)
{
	/*	数据写入FLASH	擦除加写入大概需要5ms	*/
	if(flash_flag & 0X02)
	{
		/* Unlock Flash */
		FLASH_Unlock();
		Erase_UserData(FLASH_USER_START_ADDR);
		Program_UserData(FLASH_USER_START_ADDR,USER_DATA);
		FLASH_Lock();
		uint32_t addr = FLASH_USER_START_ADDR;
		for(uint8_t i = 0;i < 32;i++,addr += 4)		//校验
		{
			if(USER_DATA[i] != HW32_REG(addr))
				return 0;
		}
	}
	flash_flag &= ~0X02;
	return 1;
}
/********************************************************
**	函数名	uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len)
**	描述	从缓存中读取数据，总共124个字节可使用
**	传入	：	offset，偏移量	data:读取的数据指针，len:读取的数据长度
**	返回	：	0：数据无效或长度超出
				1：数据读取成功
*********************************************************/
uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len)
{
	/*	将FLASH数据读入缓存*/
	if((flash_flag & 0X01) == 0X00)
	{
		uint32_t addr = FLASH_USER_START_ADDR;
		for(uint8_t i = 0;i < 32;i++,addr += 4)
		{
			USER_DATA[i] = HW32_REG(addr);
		}
		flash_flag |= 0X01;
	}
	/*	未读取到校验数据		*/
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
**	函数名	uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len)
**	描述	写入数据到缓存中，总共124个字节可使用
**	传入	：	offset，偏移量	data:写入的数据指针，len:读取的数据长度
**	返回	：	0：长度超出
				1：数据写入成功
*********************************************************/
uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len)
{
	if((offset + len) > 124)
		return 0;
	uint8_t *copy_data = (uint8_t *)&USER_DATA[1];
	copy_data += offset;
	/*	数据复制	*/
	for(uint8_t i = 0;i < len;i++)
	{
		if(*copy_data != *data)
			flash_flag |= 0X02;
		*copy_data = *data;
		data++;
		copy_data++;
	}
	/*	写入校验数据		*/
	USER_DATA[0] &= ~0XFFFF0000;
	USER_DATA[0] |= 0XA5A50000;
	return 1;
}
#endif

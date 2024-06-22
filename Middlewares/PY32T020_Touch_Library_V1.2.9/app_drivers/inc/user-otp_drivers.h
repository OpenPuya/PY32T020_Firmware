#ifndef _USER_OTP_DRIVERS_H_
#define _USER_OTP_DRIVERS_H_
#include "app_config.h"
#if APP_USER_OTP_ENABLE
/********************************************************
**	函数名	void User_Flash_Write(void)
**	描述	将缓存内数据写入FLASH中
**	传入	：	无
**	返回	：	0：保存失败
				1：保存成功
*********************************************************/
uint8_t User_Flash_Write(void);
/********************************************************
**	函数名	uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len)
**	描述	从缓存中读取数据，总共124个字节可使用
**	传入	：	offset，偏移量	data:读取的数据指针，len:读取的数据长度
**	返回	：	0：数据无效或长度超出
				1：数据读取成功
*********************************************************/
uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len);
/********************************************************
**	函数名	uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len)
**	描述	写入数据到缓存中，总共124个字节可使用
**	传入	：	offset，偏移量	data:写入的数据指针，len:读取的数据长度
**	返回	：	0：长度超出
				1：数据写入成功
*********************************************************/
uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len);
#endif
#endif

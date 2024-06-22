#ifndef _USER_OTP_DRIVERS_H_
#define _USER_OTP_DRIVERS_H_
#include "app_config.h"
#if APP_USER_OTP_ENABLE
/********************************************************
**	������	void User_Flash_Write(void)
**	����	������������д��FLASH��
**	����	��	��
**	����	��	0������ʧ��
				1������ɹ�
*********************************************************/
uint8_t User_Flash_Write(void);
/********************************************************
**	������	uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len)
**	����	�ӻ����ж�ȡ���ݣ��ܹ�124���ֽڿ�ʹ��
**	����	��	offset��ƫ����	data:��ȡ������ָ�룬len:��ȡ�����ݳ���
**	����	��	0��������Ч�򳤶ȳ���
				1�����ݶ�ȡ�ɹ�
*********************************************************/
uint8_t User_Cache_Read(uint8_t offset,uint8_t *data,uint8_t len);
/********************************************************
**	������	uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len)
**	����	д�����ݵ������У��ܹ�124���ֽڿ�ʹ��
**	����	��	offset��ƫ����	data:д�������ָ�룬len:��ȡ�����ݳ���
**	����	��	0�����ȳ���
				1������д��ɹ�
*********************************************************/
uint8_t User_Cache_Write(uint8_t offset,uint8_t *data,uint8_t len);
#endif
#endif

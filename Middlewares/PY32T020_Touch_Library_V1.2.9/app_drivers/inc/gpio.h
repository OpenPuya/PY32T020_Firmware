#ifndef _GPIO_H
#define _GPIO_H

#include "py32t020_ll_gpio.h"
#include "py32t020_ll_exti.h"
enum
{
	PA0 = 0,
	PA1,
	PA2,
	PA3,
	PA4,
	PA5,
	PA6,
	PA7,
	PA8,
	PA9,
	PA10,
	PA11,
	PA12,
	PA13,
	PA14,
	PA15,
	PB0,//16
	PB1,
	PB2,
	PB3,
	PF0,//20
	PF1,
	PF2,
	PF3,
	PF4,
	PF5,
	PIN_NONE
};
enum
{		
	INPUT		            = 0X01,         	//����
	OUTPUT		            = 0X02,   			//���
	ALTERNATE	            = 0X03,				//���ù���	
	ANALOG		            = 0X04,				//ģ�⹦��	
	
	PULL_NO					= 0X10,				//û��������
	PULL_UP					= 0X20,				//����
	PULL_DOWN 				= 0X30,				//����
	PULL_UP_DOWN			= 0X40,				//������
	
	OPENDRAIN 				= 0X100,			//��©
	PUSHPULL				= 0X200,			//����	
	
	GPIO_SPI1				= 0X0000,			//IO�ڸ��ù��ܣ�������ģʽ��Ч
	GPIO_I2C_AF3			= 0X1000,
	GPIO_I2C_AF4			= 0X2000,
	GPIO_UART1				= 0X3000,
	GPIO_UART2				= 0X4000,
	GPIO_UART3				= 0X5000,
	GPIO_TIM1_AF2			= 0X6000,
	GPIO_TIM1_AF5			= 0X7000,
	GPIO_TIM14_AF5			= 0X8000,
	GPIO_TIM14_AF6			= 0X9000,
	GPIO_COMP1				= 0XA000,
	GPIO_COMP2				= 0XB000,
	GPIO_MCO				= 0XC000,
	
	EXTI_TRIGGER_RISING		= 0X10000,			//IO���жϣ������أ�������ģʽ����
	EXTI_TRIGGER_FALLING	= 0X20000,			//IO���жϣ��½��أ�������ģʽ����
	EXTI_TRIGGER_RISING_FALLING	= 0X30000,		//IO���жϣ�˫���أ�������ģʽ����
};

#define NO_PIN				 26
#define GPIO_PORT(__PIN__)   ((__PIN__ <= 15 ) ? GPIOA : ((__PIN__) <= (19))? GPIOB : GPIOF)
#define GPIO_PIN(__PIN__)    (1 << ((__PIN__ <= 15 ) ? (__PIN__) : ((__PIN__) <= (19))? (__PIN__ - 16) : (__PIN__ - 20)))	

/********************************************************
**	������	void GPIO_Init(uint8_t gpio,uint32_t Init)
**	����	GPIO��ʼ��
**	����	��	gpio��IPA0~PA15,PB0~PB3,PF0~PF5
				Init����ʼ���Ĳ���
	��GPIO����Ϊģ��ģʽ					->	GPIO_Init(PA0,ANALOG)
	��GPIO����Ϊ��������ģʽ				->	GPIO_Init(PA0,INPUT|PULL_UP)
	��GPIO����Ϊ���������½����ж�ģʽ	->	GPIO_Init(PA0,INPUT|PULL_UP|EXTI_TRIGGER_FALLING)
	��GPIO����Ϊ�������ģʽ				->	GPIO_Init(PA0,OUTPUT|PUSHPULL)
	��GPIO����Ϊ��©���ģʽ				->	GPIO_Init(PA0,OUTPUT|OPENDRAIN)
	��GPIO����ΪTIM1_CH3���				->	GPIO_Init(PA0,ALTERNATE | GPIO_TIM1_AF2) 
												����ѡ��GPIO_TIM1_AF2����GPIO_TIM1_AF5��Ҫ�鿴�����ĸ��ù���ӳ��
	��GPIO����ΪUART����					->	GPIO_Init(PA0,ALTERNATE | GPIO_UART2) 
**	����	����
*********************************************************/
void GPIO_Init(uint8_t gpio,uint32_t Init);
/********************************************************
**	������	void GPIO_SetBit(uint8_t gpio)
**	����	GPIO����ߵ�ƽ
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	����
*********************************************************/
void GPIO_SetBit(uint8_t gpio);
/********************************************************
**	������	void GPIO_ClearBit(uint8_t gpio)
**	����	GPIO����͵�ƽ
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	����
*********************************************************/
void GPIO_ClearBit(uint8_t gpio);
/********************************************************
**	������	void GPIO_ToggleBit(uint8_t gpio)
**	����	GPIO�����ת
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	����
*********************************************************/
void GPIO_ToggleBit(uint8_t gpio);
/********************************************************
**	������	uint8_t GPIO_ReadBit(uint8_t gpio)
**	����	��ȡGPIO״̬
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	��gpio״̬
*********************************************************/
uint8_t GPIO_ReadBit(uint8_t gpio);
/********************************************************
**	������	void EXTI0_15_IRQHandlerCallback(uint32_t PR)
**	����	�ⲿ�жϻص�����
**	����	��PR:�жϱ�־
**	����	����
*********************************************************/
void EXTI0_15_IRQHandlerCallback(uint32_t PR);

#endif

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
	INPUT		            = 0X01,         	//输入
	OUTPUT		            = 0X02,   			//输出
	ALTERNATE	            = 0X03,				//复用功能	
	ANALOG		            = 0X04,				//模拟功能	
	
	PULL_NO					= 0X10,				//没有上下拉
	PULL_UP					= 0X20,				//上拉
	PULL_DOWN 				= 0X30,				//下拉
	PULL_UP_DOWN			= 0X40,				//上下拉
	
	OPENDRAIN 				= 0X100,			//开漏
	PUSHPULL				= 0X200,			//推挽	
	
	GPIO_SPI1				= 0X0000,			//IO口复用功能，仅复用模式有效
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
	
	EXTI_TRIGGER_RISING		= 0X10000,			//IO口中断，上升沿，仅输入模式有限
	EXTI_TRIGGER_FALLING	= 0X20000,			//IO口中断，下降沿，仅输入模式有限
	EXTI_TRIGGER_RISING_FALLING	= 0X30000,		//IO口中断，双边沿，仅输入模式有限
};

#define NO_PIN				 26
#define GPIO_PORT(__PIN__)   ((__PIN__ <= 15 ) ? GPIOA : ((__PIN__) <= (19))? GPIOB : GPIOF)
#define GPIO_PIN(__PIN__)    (1 << ((__PIN__ <= 15 ) ? (__PIN__) : ((__PIN__) <= (19))? (__PIN__ - 16) : (__PIN__ - 20)))	

/********************************************************
**	函数名	void GPIO_Init(uint8_t gpio,uint32_t Init)
**	描述	GPIO初始化
**	传入	：	gpio：IPA0~PA15,PB0~PB3,PF0~PF5
				Init：初始化的参数
	将GPIO设置为模拟模式					->	GPIO_Init(PA0,ANALOG)
	将GPIO设置为输入上拉模式				->	GPIO_Init(PA0,INPUT|PULL_UP)
	将GPIO设置为输入上拉下降沿中断模式	->	GPIO_Init(PA0,INPUT|PULL_UP|EXTI_TRIGGER_FALLING)
	将GPIO设置为推挽输出模式				->	GPIO_Init(PA0,OUTPUT|PUSHPULL)
	将GPIO设置为开漏输出模式				->	GPIO_Init(PA0,OUTPUT|OPENDRAIN)
	将GPIO设置为TIM1_CH3输出				->	GPIO_Init(PA0,ALTERNATE | GPIO_TIM1_AF2) 
												这里选择GPIO_TIM1_AF2或者GPIO_TIM1_AF5需要查看规格书的复用功能映射
	将GPIO设置为UART功能					->	GPIO_Init(PA0,ALTERNATE | GPIO_UART2) 
**	返回	：无
*********************************************************/
void GPIO_Init(uint8_t gpio,uint32_t Init);
/********************************************************
**	函数名	void GPIO_SetBit(uint8_t gpio)
**	描述	GPIO输出高电平
**	传入	：	gpio：PA0~PA15,PB0~PB3,PF0~PF5
**	返回	：无
*********************************************************/
void GPIO_SetBit(uint8_t gpio);
/********************************************************
**	函数名	void GPIO_ClearBit(uint8_t gpio)
**	描述	GPIO输出低电平
**	传入	：	gpio：PA0~PA15,PB0~PB3,PF0~PF5
**	返回	：无
*********************************************************/
void GPIO_ClearBit(uint8_t gpio);
/********************************************************
**	函数名	void GPIO_ToggleBit(uint8_t gpio)
**	描述	GPIO输出翻转
**	传入	：	gpio：PA0~PA15,PB0~PB3,PF0~PF5
**	返回	：无
*********************************************************/
void GPIO_ToggleBit(uint8_t gpio);
/********************************************************
**	函数名	uint8_t GPIO_ReadBit(uint8_t gpio)
**	描述	读取GPIO状态
**	传入	：	gpio：PA0~PA15,PB0~PB3,PF0~PF5
**	返回	：gpio状态
*********************************************************/
uint8_t GPIO_ReadBit(uint8_t gpio);
/********************************************************
**	函数名	void EXTI0_15_IRQHandlerCallback(uint32_t PR)
**	描述	外部中断回调函数
**	传入	：PR:中断标志
**	返回	：无
*********************************************************/
void EXTI0_15_IRQHandlerCallback(uint32_t PR);

#endif

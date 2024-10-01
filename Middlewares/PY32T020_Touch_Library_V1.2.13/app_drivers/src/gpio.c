#include "gpio.h"
typedef struct
{
    GPIO_TypeDef *PORT;
    uint16_t PIN;
} IO_Type;

static const IO_Type T020_IO[26] = 
{
    {GPIOA, LL_GPIO_PIN_0}, 
	{GPIOA, LL_GPIO_PIN_1}, 
	{GPIOA, LL_GPIO_PIN_2}, 
	{GPIOA, LL_GPIO_PIN_3}, 
	{GPIOA, LL_GPIO_PIN_4}, 
	{GPIOA, LL_GPIO_PIN_5}, 
	{GPIOA, LL_GPIO_PIN_6}, 
	{GPIOA, LL_GPIO_PIN_7}, 
	{GPIOA, LL_GPIO_PIN_8}, 
	{GPIOA, LL_GPIO_PIN_9}, 
	{GPIOA, LL_GPIO_PIN_10}, 
	{GPIOA, LL_GPIO_PIN_11}, 
	{GPIOA, LL_GPIO_PIN_12}, 
	{GPIOA, LL_GPIO_PIN_13}, 
	{GPIOA, LL_GPIO_PIN_14}, 
	{GPIOA, LL_GPIO_PIN_15},

	{GPIOB, LL_GPIO_PIN_0}, 
	{GPIOB, LL_GPIO_PIN_1}, 
	{GPIOB, LL_GPIO_PIN_2}, 
	{GPIOB, LL_GPIO_PIN_3}, 
	
	{GPIOF, LL_GPIO_PIN_0}, 
	{GPIOF, LL_GPIO_PIN_1},
	{GPIOF, LL_GPIO_PIN_2},
	{GPIOF, LL_GPIO_PIN_3},
	{GPIOF, LL_GPIO_PIN_4},
	{GPIOF, LL_GPIO_PIN_5},
};

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
static uint32_t exti_bit = 0;
void GPIO_Init(uint8_t gpio,uint32_t Init)
{
	uint32_t Mode      = (Init & 0X000000F);		//����
	uint32_t Pull      = (Init & 0X00000F0);		//����������
	uint32_t OutputType= (Init & 0X0000F00);		//���ģʽ
	uint32_t Alternate = (Init & 0X000F000);		//��������ģʽ
	uint32_t Exti      = (Init & 0X00F0000);		//��������ģʽ
	if(gpio >= NO_PIN)
		return;
	GPIO_TypeDef *GPIOx = GPIO_PORT(gpio);
	uint32_t PINx = GPIO_PIN(gpio);
	uint32_t EXTI_CONFIG_LINE = LL_EXTI_LINE_NONE;
	/* Speed mode configuration */
	LL_GPIO_SetPinSpeed(GPIOx, PINx, LL_GPIO_SPEED_FREQ_HIGH);
	/*	���֮ǰ���ⲿ�ж�ʹ��	*/
	if(exti_bit & (1 << gpio))
	{
		exti_bit &= ~(1 << gpio);
		LL_EXTI_DisableFallingTrig(PINx);
		LL_EXTI_DisableRisingTrig(PINx);
		LL_EXTI_DisableIT(PINx);
	}
	switch(Pull)
	{
		default:
		case PULL_NO:
			LL_GPIO_SetPinPull(GPIOx, PINx, LL_GPIO_PULL_NO); 
		break;
		case PULL_UP:
			LL_GPIO_SetPinPull(GPIOx, PINx, LL_GPIO_PULL_UP); 
		break;
		case PULL_DOWN:
			LL_GPIO_SetPinPull(GPIOx, PINx, LL_GPIO_PULL_DOWN); 
		break;
		case PULL_UP_DOWN:
			LL_GPIO_SetPinPull(GPIOx, PINx, LL_GPIO_PULL_UP_DOWN); 
		break;
	}
	switch(OutputType)
	{
		case OPENDRAIN:
			LL_GPIO_SetPinOutputType(GPIOx, PINx, LL_GPIO_OUTPUT_OPENDRAIN);
		break;
		case PUSHPULL:
			LL_GPIO_SetPinOutputType(GPIOx, PINx, LL_GPIO_OUTPUT_PUSHPULL);
		break;
	}
	switch(Mode)
	{
		case INPUT:
			LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_INPUT);
			switch(Exti)
			{
				case EXTI_TRIGGER_RISING:
					/* First Disable Falling Trigger on provided Lines */
					LL_EXTI_DisableFallingTrig(PINx);
					/* Then Enable Rising Trigger on provided Lines */
					LL_EXTI_EnableRisingTrig(PINx);
				break;
				case EXTI_TRIGGER_FALLING:
					/* First Disable Rising Trigger on provided Lines */
					LL_EXTI_DisableRisingTrig(PINx);
					/* Then Enable Falling Trigger on provided Lines */
					LL_EXTI_EnableFallingTrig(PINx);
				break;
				case EXTI_TRIGGER_RISING_FALLING:
					LL_EXTI_EnableRisingTrig(PINx);
					LL_EXTI_EnableFallingTrig(PINx);
				break;
			}
			switch(Exti)
			{
				case EXTI_TRIGGER_RISING:
				case EXTI_TRIGGER_FALLING:
				case EXTI_TRIGGER_RISING_FALLING:
					switch(PINx)
					{
						case 0X01:
							EXTI_CONFIG_LINE = LL_EXTI_CONFIG_LINE0;
						break;
						case 0X02:
							EXTI_CONFIG_LINE = LL_EXTI_CONFIG_LINE1;
						break;
						case 0X04:
							EXTI_CONFIG_LINE = LL_EXTI_CONFIG_LINE2;
						break;
						case 0X08:
							EXTI_CONFIG_LINE = LL_EXTI_CONFIG_LINE3;
						break;
						case 0X10:
							EXTI_CONFIG_LINE = LL_EXTI_CONFIG_LINE4;
						break;
						case 0X20:
							EXTI_CONFIG_LINE = LL_EXTI_CONFIG_LINE5;
						break;
					}
					if(EXTI_CONFIG_LINE != LL_EXTI_LINE_NONE)
					{
						if(GPIOx == GPIOA)
						{
							LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA,EXTI_CONFIG_LINE);
						}
						else if(GPIOx == GPIOB)
						{
							LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTB,EXTI_CONFIG_LINE);
						}
						else
						{
							LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTF,EXTI_CONFIG_LINE);
						}
					}
					exti_bit |= 1 << gpio;
					 /* Configure Button pin as input with External interrupt */
					LL_EXTI_EnableIT(PINx);
					/* Enable and set Button EXTI Interrupt to the lowest priority */
					if(PINx == 0X01 || PINx == 0X02)
					{
						NVIC_SetPriority(EXTI0_1_IRQn, 3);
						NVIC_EnableIRQ(EXTI0_1_IRQn);
					}
					else if(PINx == 0X04 || PINx == 0X08)
					{
						NVIC_SetPriority(EXTI2_3_IRQn, 3);
						NVIC_EnableIRQ(EXTI2_3_IRQn);
					}
					else
					{
						NVIC_SetPriority(EXTI4_15_IRQn, 3);
						NVIC_EnableIRQ(EXTI4_15_IRQn);
					}
				break;
			}
		break;
		case OUTPUT:
			LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_OUTPUT);
		break;
		case ANALOG:
			LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_ANALOG);
		break;
		case ALTERNATE:
			LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_ALTERNATE);
			switch(Alternate)
			{
				case GPIO_SPI1:
					Alternate = LL_GPIO_AF0_SPI1;
				break;
				case GPIO_I2C_AF3:
					Alternate = LL_GPIO_AF3_I2C;
				break;
				case GPIO_I2C_AF4:
					Alternate = LL_GPIO_AF4_I2C;
				break;
				case GPIO_UART1:
					Alternate = LL_GPIO_AF1_UART1;
				break;
				case GPIO_UART2:
					Alternate = LL_GPIO_AF3_UART2;
				break;
				case GPIO_UART3:
					Alternate = LL_GPIO_AF1_UART3;
				break;
				case GPIO_TIM1_AF2:
					Alternate = LL_GPIO_AF2_TIM1;
				break;
				case GPIO_TIM1_AF5:
					Alternate = LL_GPIO_AF5_TIM1;
				break;
				case GPIO_TIM14_AF5:
					Alternate = LL_GPIO_AF5_TIM14;
				break;
				case GPIO_TIM14_AF6:
					Alternate = LL_GPIO_AF6_TIM14;
				break;
				case GPIO_COMP1:
					Alternate = LL_GPIO_AF6_COMP1;
				break;
				case GPIO_COMP2:
					Alternate = LL_GPIO_AF6_COMP2;
				break;
				case GPIO_MCO:
					Alternate = LL_GPIO_AF6_MCO;
				break;
			}
			if (PINx < LL_GPIO_PIN_8)
			{
				LL_GPIO_SetAFPin_0_7(GPIOx, PINx, Alternate);
			}
			else
			{
				LL_GPIO_SetAFPin_8_15(GPIOx, PINx, Alternate);
			}
		break;
	}
}
/********************************************************
**	������	void GPIO_SetBit(uint8_t gpio)
**	����	GPIO����ߵ�ƽ
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	����
*********************************************************/
void GPIO_SetBit(uint8_t gpio)
{
	if(gpio >= NO_PIN)
		return;
	GPIO_PORT(gpio)->BSRR = GPIO_PIN(gpio);
}
/********************************************************
**	������	void GPIO_ClearBit(uint8_t gpio)
**	����	GPIO����͵�ƽ
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	����
*********************************************************/
void GPIO_ClearBit(uint8_t gpio)
{
	if(gpio >= NO_PIN)
		return;
	GPIO_PORT(gpio)->BRR = GPIO_PIN(gpio);
}
/********************************************************
**	������	void GPIO_ToggleBit(uint8_t gpio)
**	����	GPIO�����ת
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	����
*********************************************************/
void GPIO_ToggleBit(uint8_t gpio)
{
	if(gpio >= NO_PIN)
		return;
	WRITE_REG(GPIO_PORT(gpio)->ODR, READ_REG(GPIO_PORT(gpio)->ODR) ^ GPIO_PIN(gpio));
}
/********************************************************
**	������	uint8_t GPIO_ReadBit(uint8_t gpio)
**	����	��ȡGPIO״̬
**	����	��	gpio��PA0~PA15,PB0~PB3,PF0~PF5
**	����	��gpio״̬
*********************************************************/
uint8_t GPIO_ReadBit(uint8_t gpio)
{
	if(gpio >= NO_PIN)
		return 0;
	if(READ_BIT(GPIO_PORT(gpio)->IDR, GPIO_PIN(gpio)) == (GPIO_PIN(gpio)))
		return 1;
	return 0;
}
/********************************************************
**	������	void EXTI0_1_IRQHandler(void)
**	����	�ⲿ�ж�0��1���
**	����	����
**	����	����
*********************************************************/
void EXTI0_1_IRQHandler(void)
{
	uint32_t PR = EXTI->PR;
	EXTI->PR = PR;
	__disable_irq();
	EXTI0_15_IRQHandlerCallback(PR);
	__enable_irq();
}
/********************************************************
**	������	void EXTI2_3_IRQHandler(void)
**	����	�ⲿ�ж�2��3���
**	����	����
**	����	����
*********************************************************/
void EXTI2_3_IRQHandler(void)
{
	uint32_t PR = EXTI->PR;
	EXTI->PR = PR;
	__disable_irq();
	EXTI0_15_IRQHandlerCallback(PR);
	__enable_irq();
}
/********************************************************
**	������	void EXTI4_15_IRQHandler(void)
**	����	�ⲿ�ж�4~15���
**	����	����
**	����	����
*********************************************************/
void EXTI4_15_IRQHandler(void)
{
	uint32_t PR = EXTI->PR;
	EXTI->PR = PR;
	__disable_irq();
	EXTI0_15_IRQHandlerCallback(PR);
	__enable_irq();
}
/********************************************************
**	������	void EXTI0_15_IRQHandlerCallback(uint32_t PR)
**	����	�ⲿ�жϻص�����
**	����	��PR:�жϱ�־
**	����	����
*********************************************************/
__weak void EXTI0_15_IRQHandlerCallback(uint32_t PR)
{


}

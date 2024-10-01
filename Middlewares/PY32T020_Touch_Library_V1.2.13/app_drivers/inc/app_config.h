#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include <stdio.h>
#include "py32t020_ll_rcc.h"
#include "py32t020_ll_bus.h"
#include "py32t020_ll_system.h"
#include "py32t020_ll_cortex.h"
#include "py32t020_ll_utils.h"
#include "py32t020_ll_pwr.h"
#include "py32t020_ll_gpio.h"
#include "py32t020_ll_uart.h"
#include "py32t020_ll_adc.h"
#include "py32t020_ll_tim.h"
#include "py32t020_ll_spi.h"
#include "py32t020_ll_exti.h"
#include "py32t020_ll_iwdg.h"
#include "py32t020_ll_rtc.h"
#if defined(USE_FULL_ASSERT)
#include "py32_assert.h"
#endif /* USE_FULL_ASSERT */


// <<< Use Configuration Wizard in Context Menu >>>
//  <o>	HSI
//  <i> ����ϵͳʱ�ӣ�Ĭ��24M��
//  <24000000UL=> 24MHz
//  <48000000UL=> 48MHz
#define HSI_FREQUENCE 		24000000UL

#define fac_us  			(HSI_FREQUENCE / 1000000)
//  <o>	TK_MODULE
//  <i> ������رմ�����
//  <0=> DISABLE
//  <1=> ENABLE
#define	APP_TK_ENABLE			1	

//  <e>	SYSTICK
//  <i>ʹ��SYSTICK���ж�ʱ����ʱ���жϷ�����ΪSysTick_Handler,Ϊ�������ṩʱ��
#define APP_SYSTICK_ENABLE			1	

#if APP_SYSTICK_ENABLE
//  <o>��ʱʱ��(��λus)
//  <i>����ϵͳ��ʱ��ʱ�䣬����1ms
//  <0-1000>
#define SYSTICK_TIMING_TIME			1000
//  <o> SYSTICK�ж����ȼ�
//  <i>Default: LOWEST
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define SYSTICK_IRQ_PRIORITY			0
// <e>	�жϷ������IO�ڷ�ת����
#define SYSTICK_DEBUG					0
// <o.0..15>IOѡ��
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SYSTICK_DEBUG_GPIO 			20
// </e>
#endif
// </e>			

// <e>	PRINTF
//  <i>	��ӡ�û�������Ϣ,����log_printf���д�ӡ��ʹ�÷���ͬprintf
#define DEBUG_ENABLE			0
#if DEBUG_ENABLE
//  <e>	UART
//  <i> ��ѡΪʹ�ô��ڴ�ӡ��Ĭ��Ϊ����������λ����ӡ(�迪����������)
#define DEBUG_MODE				0	
//  <o>	UARTѡ��
//  <i> ��Ҫͬʱ������Ӧ��UARTģ��,��������Ӧ�Ĳ���
//  <1=> UART1
//  <2=> UART2
//  <3=> UART3
#define PRINTF_UART				1	
// </e>
// </e>
#endif

// <e>	UART
//  <i> UART���ã�Ĭ��Ϊ8λ����λ��1��ֹͣλ��ֻ�����ùܽ��Լ������ʼ���
#define APP_UART_ENABLE			0				//ʹ�ܴ�������	
#if APP_UART_ENABLE
// <e>	UART1
//  <i> UART1���ã�ʹ���ж���ʹ�ö��з����Լ����գ����г���Ϊ64�ֽ�
#define UART1_ENABLE			1
/*
	UART1_TX		PF3/PA6/PA11
	UART1_RX		PF4/PA7/PA12
*/
#if UART1_ENABLE
// <o.0..15>UART1_TX_PIN
//  <i> UART1���͹ܽ����ã���ʼ����ɺ����UART1_QueueSend���ɽ��з��ͣ�ѡ��NO_PIN��رշ��͹���
//  <6=>PA6
//  <11=>PA11
//  <23=>PF3
//  <26=>NO_PIN
#define UART1_TX_PIN				6
// <o.0..15>UART1_RX_PIN
//  <i> UART1���չܽ����ã���ʼ����ɺ����UART1_QueueRead��ѯ���ݣ�����1ʱ��ʾ������Ч��ѡ��NO_PIN��رս��չ���
//  <7=>PA7
//  <12=>PA12
//  <24=>PF4
//  <26=>NO_PIN
#define UART1_RX_PIN				7
// <o>UART1������
// <i>UART����������
// <1200=> 1200 baud 
// <2400=> 2400 baud 
// <4800=> 4800 baud 
// <9600=> 9600 baud 
// <19200=> 19200 baud 
// <38400=> 38400 baud 
// <57600=> 57600 baud 
// <115200=> 115200 baud 
#define UART1_BAUDRATE				115200
//  <o> UART1�ж�ʹ��
//  <i>UART�жϿ��أ�Ĭ�ϴ򿪣��ر������ֻ��ʹ�ò�ѯ�ķ�ʽ���з��ͽ���
//  <0=> DISABLE
//  <1=> ENABLE
#define UART1_IRQ_ENABLE			1
//  <o> UART1�ж����ȼ�
//  <i>UART�ж����ȼ��趨
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define UART1_IRQ_PRIORITY			3
#endif
// </e>
/*
	UART2_TX		PA0/PA4/PB0/PA9/PA13
	UART2_RX		PA1/PA5/PB1/PA10/PB3
*/
// <e>	UART2
//  <i> UART2���ã�ʹ���ж���ʹ�ö��з����Լ����գ����г���Ϊ64�ֽ�
#define UART2_ENABLE				1
#if UART2_ENABLE
// <o.0..15>UART2_TX_PIN
//  <i> UART2���͹ܽ����ã���ʼ����ɺ����UART2_QueueSend���ɽ��з��ͣ�ѡ��NO_PIN��رշ��͹���
//  <0=>PA0
//  <4=>PA4
//  <9=>PA9
//  <13=>PA13
//  <16=>PB0
//  <26=>NO_PIN
#define UART2_TX_PIN				0
// <o.0..15>UART2_RX_PIN
//  <i> UART2���չܽ����ã���ʼ����ɺ����UART2_QueueRead��ѯ���ݣ�����1ʱ��ʾ������Ч��ѡ��NO_PIN��رս��չ���
//  <1=>PA1
//  <5=>PA5
//  <10=>PA10
//  <17=>PB1
//  <19=>PB3
//  <26=>NO_PIN
#define UART2_RX_PIN				1
// <o>UART2������
// <i>UART����������
// <1200=> 1200 baud 
// <2400=> 2400 baud 
// <4800=> 4800 baud 
// <9600=> 9600 baud 
// <19200=> 19200 baud 
// <38400=> 38400 baud 
// <57600=> 57600 baud 
// <115200=> 115200 baud 
#define UART2_BAUDRATE				115200
//  <o> UART2�ж�ʹ��
//  <i>UART�жϿ��أ�Ĭ�ϴ򿪣��ر������ֻ��ʹ�ò�ѯ�ķ�ʽ���з��ͽ���
//  <0=> DISABLE
//  <1=> ENABLE
#define UART2_IRQ_ENABLE			1
//  <o> UART2�ж����ȼ�
//  <i>UART�ж����ȼ��趨
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define UART2_IRQ_PRIORITY			3
#endif
// </e>
/*
	UART3_TX		PA2/PA6/PA14
	UART3_RX		PA3/PA7/PA15
*/
// <e>	UART3
//  <i> UART3���ã�ʹ���ж���ʹ�ö��з����Լ����գ����г���Ϊ64�ֽ�
#define UART3_ENABLE				1
#if UART3_ENABLE
// <o.0..15>UART3_TX_PIN
//  <i> UART3���͹ܽ����ã���ʼ����ɺ����UART3_QueueSend���ɽ��з��ͣ�ѡ��NO_PIN��رշ��͹���
//  <2=>PA2
//  <8=>PA8
//  <14=>PA14
//  <26=>NO_PIN
#define UART3_TX_PIN				2
// <o.0..15>UART3_RX_PIN
//  <i> UART3���չܽ����ã���ʼ����ɺ����UART3_QueueRead��ѯ���ݣ�����1ʱ��ʾ������Ч��ѡ��NO_PIN��رս��չ���
//  <3=>PA3
//  <15=>PA15
//  <18=>PB2
//  <26=>NO_PIN
#define UART3_RX_PIN				3
// <o>UART3������
// <i>UART����������
// <1200=> 1200 baud 
// <2400=> 2400 baud 
// <4800=> 4800 baud 
// <9600=> 9600 baud 
// <19200=> 19200 baud 
// <38400=> 38400 baud 
// <57600=> 57600 baud 
// <115200=> 115200 baud 
#define UART3_BAUDRATE				115200
//  <o> UART3�ж�ʹ��
//  <i>UART�жϿ��أ�Ĭ�ϴ򿪣��ر������ֻ��ʹ�ò�ѯ�ķ�ʽ���з��ͽ���
//  <0=> DISABLE
//  <1=> ENABLE
#define UART3_IRQ_ENABLE				1
//  <o> UART3�ж����ȼ�
//  <i>UART�ж����ȼ��趨
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define UART3_IRQ_PRIORITY			3
// </e>
#endif
#endif
// </e>

//  <e>	Flash Option byte
//  <i> �������õ͵�ѹ��λ��Ӳ�����Ź�����λ�Ÿ���ΪGPIO
#define OB_USER_OPTION				0		
#if OB_USER_OPTION
//  <o> ����PF2(NRST)ģʽ
//  <i> PF2Ĭ��Ϊ��λ����,����ҪPF2��ΪGPIO��������ʱ������ΪGPIOģʽ
//  <0=> RST
//  <1=> GPIO
#define OPTION_NRST_MODE			0
#if OPTION_NRST_MODE
#define OB_GPIO_PIN_MODE		FLASH_OPTR_NRST_MODE                                /*!< PF2: GPIO */
#else
#define OB_GPIO_PIN_MODE		0x00000000U                                         /*!< PF2: NRST */
#endif
//  <o> �͵�ѹ��λʹ��
//  <i> ʹ�ܺ󣬵���ѹ������ֵʱ��Ƭ��������λ
//  <0=> DISABLE
//  <1=> ENABLE
#define OPTION_BOR_EN				0
#if OPTION_BOR_EN
#define OB_BOR_EN					FLASH_OPTR_BOR_EN	
#else
#define OB_BOR_EN					0x00000000U	
#endif
//  <o> �͵�ѹ��ֵ
//  <i> �������õ͵�ѹ��ֵ
//  <1=> 1.7V
//  <2=> 2.0V
//  <3=> 2.2V
//  <4=> 2.4V
//  <5=> 2.8V
//  <6=> 3.7V
//  <7=> 4.2V
#define OPTION_BOR_LEV					4
#if (OPTION_BOR_LEV == 1)
#define OB_BOR_LEVEL					((uint32_t)(FLASH_OPTR_BOR_LEV_0))	
#elif (OPTION_BOR_LEV == 2)
#define OB_BOR_LEVEL					((uint32_t)(FLASH_OPTR_BOR_LEV_1))	
#elif (OPTION_BOR_LEV == 3)
#define OB_BOR_LEVEL					((uint32_t)(FLASH_OPTR_BOR_LEV_0 | FLASH_OPTR_BOR_LEV_1))	
#elif (OPTION_BOR_LEV == 4)
#define OB_BOR_LEVEL					((uint32_t)(FLASH_OPTR_BOR_LEV_2))	
#elif (OPTION_BOR_LEV == 5)
#define OB_BOR_LEVEL					((uint32_t)(FLASH_OPTR_BOR_LEV_0 | FLASH_OPTR_BOR_LEV_2))
#elif (OPTION_BOR_LEV == 6)
#define OB_BOR_LEVEL					((uint32_t)(FLASH_OPTR_BOR_LEV_1 | FLASH_OPTR_BOR_LEV_2))
#elif (OPTION_BOR_LEV == 7)
#define OB_BOR_LEVEL					((uint32_t)(FLASH_OPTR_BOR_LEV_0 | FLASH_OPTR_BOR_LEV_1 | FLASH_OPTR_BOR_LEV_2))
#endif
//  <o> Ӳ�����Ź�
//  <i> Ӳ�����Ź�(�ϵ��Զ�����)��������Ź�(��������ر�)
//  <0=> Ӳ�����Ź�
//  <1=> ������Ź�
#define OPTION_IWDG_SW_EN				1
#if OPTION_IWDG_SW_EN
#define OB_IWDG_MODE					FLASH_OPTR_IWDG_SW	
#else
#define OB_IWDG_MODE					0x00000000U	
#endif
//  <o> ���Ź���STOPģʽ������״̬
//  <i> �������ÿ��Ź���STOPģʽ�µ�״̬
//  <0=> ֹͣ����
//  <1=> ��������
#define OPTION_IWDG_STOP				0
#if OPTION_IWDG_STOP
#define OB_IWDG_STOP					((uint32_t)FLASH_OPTR_IWDG_STOP) /*!< IWDG counter active in STOP mode */
#else
#define OB_IWDG_STOP					0x00000000U	
#endif
// </e>

#endif

// <e>	Flash User OTP	
// <i>  ʹ��USER_OTP���򱣴��û����ݣ���124�ֽ�,����User_Cache_Read��ȡ���ݣ�����1ʱ������Ч������User_Cache_Write������д�뻺�棬����User_Flash_Write����������д��FLASH
#define APP_USER_OTP_ENABLE			0
#if APP_USER_OTP_ENABLE
// <e>	���籣������
// <i>  ��Ҫ����ADCģ���Լ�ʹ��VREFINTͨ��
#define LVD_WRITE_USER_DATA			1			
//  <o> �������ѹ
//  <i>��λ:MV �����ѹ�������õù��ͣ���ֹ��ѹ�½�������������޷���������
//  <3000-4200>
#define low_voltage					3100
// </e>
#endif
// </e>

// <e>	IWDG
//  <i> ��������������Ź������ó�ʱʱ��
#define APP_IWDG_ENABLE			0				//ʹ�ܴ�������	
//  <o> ���Ź���ʱʱ��
//  <i>��λ:ms ���Ź���ʱʱ��
//  <0-4095>
#define IDWG_RELOAD_VALUE		1500
// </e>

//  <e>	BEEP
//  <i> ����������Դ������
#define APP_BEEP_ENABLE					0	

#if APP_BEEP_ENABLE
// <o.0..15>���������ƹܽ�
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
#define BEEP_GPIO		 			15
//  <o> ������������ƽ
//  <0=> �͵�ƽ��Ч
//  <1=> �ߵ�ƽ��Ч
#define BEEP_GPIO_TYPE				1
#endif
// </e>

//  <e>	TIM14
//  <i>ʹ��TIM14���ж�ʱ����ʱ���жϷ�����ΪTIM14_PeriodElapsedCallback
#define APP_TIM14_ENABLE			0	

#if APP_TIM14_ENABLE
//  <o>��ʱʱ��(��λus)
//  <i>Default: 125 (Unit:us)
//  <0-1000000>
#define TIM14_TIMING_TIME			125
//  <o> TIM14�ж�ʹ��
//  <i>Default: DISABLE
//  <0=> DISABLE
//  <1=> ENABLE
#define TIM14_IRQ_ENABLE			1
//  <o> TIM14�ж����ȼ�
//  <i>Default: LOWEST
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define TIM14_IRQ_PRIORITY			0
// <e>	�жϷ������IO�ڷ�ת����
#define TIM14_DEBUG					0
// <o.0..15>IOѡ��
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define TIM14_DEBUG_GPIO 			19
// </e>
#endif
// </e>

// <e>	TIM14-PWM
//  <i>ʹ��TIM14����PWM���,��TIM14��ʱ����Դ���⣬����void TIM14_PWM_Pulse(uint32_t CHx,uint8_t percent)���ðٷֱ�
#define APP_TIM14_PWM_ENABLE		0				
#if APP_TIM14_PWM_ENABLE
//  <o>PWMƵ������
//  <i>����PWMƵ�ʣ���λHZ
//  <0-1000000>
#define TIM14_PWM_FREQUENCE			25000
// <o>TIM14_RESOLUTION
//  <i>�ֱ�������
//  <0-65535>
#define TIM14_RESOLUTION			100
// <o>TIM14_DUTY_CYCLE
//  <i>Ĭ��ռ�ձ�����
//  <0-65535>
#define TIM14_DUTY_CYCLE			10
// <o>TIM14_CH1_PIN
//  <i>TIM14_CH1����ܽ�����
//  <2=>PA2
//  <9=>PA9
//  <10=>PA10
//  <12=>PA12
//  <18=>PB2
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
#define TIM14_CH1_PIN 				9
#endif
// </e>

// <e>	TIM1
//  <i>ʹ��TIM1���ж�ʱ����ʱ���жϷ�����ΪTIM1_PeriodElapsedCallback
#define APP_TIM1_ENABLE				0	

#if APP_TIM1_ENABLE
//  <o>��ʱʱ��(��λus)
//  <i>Default: 125 (Unit:us)
//  <0-1000000>
#define TIM1_TIMING_TIME			125
//  <o> TIM1�ж�ʹ��
//  <i>Default: DISABLE
//  <0=> DISABLE
//  <1=> ENABLE
#define TIM1_IRQ_ENABLE				1
//  <o> TIM1�ж����ȼ�
//  <i>Default: LOWEST
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define TIM1_IRQ_PRIORITY			0
// <e>	�жϷ������IO�ڷ�ת����
#define TIM1_DEBUG					0
// <o.0..15>IOѡ��
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define TIM1_DEBUG_GPIO 			20
// </e>
#endif
// </e>

/*
	TIM1_CH1		PA3/PA6/PA8/PA7
	TIM1_CH2		PA5/PB3/PB0
	TIM1_CH3		PA0/PA4/PA10
	TIM1_CH4		PA1/PA11/PA15
*/
// <e>	TIM1-PWM
//  <i>ʹ��TIM1����PWM���,��TIM1��ʱ����Դ���⣬����void TIM1_PWM_Pulse(uint32_t CHx,uint8_t percent)���ðٷֱ�
#define APP_TIM1_PWM_ENABLE		0				
#if APP_TIM1_PWM_ENABLE
//  <o>PWMƵ������
//  <i>����PWMƵ�ʣ���λHZ
//  <0-1000000>
#define TIM1_PWM_FREQUENCE		4000
// <o>TIM1_RESOLUTION
//  <i>�ֱ�������
//  <0-65535>
#define TIM1_RESOLUTION			100
// <e>	PWM-CH1
#define TIM1_CH1_ENABLE			1
// <o>DUTY1_CYCLE
//  <i>Ĭ��ռ�ձ�����
//  <0-100>
#define TIM1_DUTY1_CYCLE		10
// <o>TIM1_CH1_PIN
//  <i>TIM1_CH1����ܽ�����
//  <3=>PA3
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
#define TIM1_CH1_PIN 			3
// </e>
// <e>	PWM-CH2
#define TIM1_CH2_ENABLE			1
// <o>DUTY2_CYCLE
//  <i>Ĭ��ռ�ձ�����
//  <0-100>
#define TIM1_DUTY2_CYCLE		20
// <o>TIM1_CH2_PIN
//  <i>TIM1_CH2����ܽ�����
//  <5=>PA5
//  <16=>PB0
//  <19=>PB3
#define TIM1_CH2_PIN 			5
// </e>
// <e>	PWM-CH3
#define TIM1_CH3_ENABLE			1
// <o>DUTY3_CYCLE
//  <i>Ĭ��ռ�ձ�����
//  <0-100>
#define TIM1_DUTY3_CYCLE		30
// <o>TIM1_CH3_PIN
//  <i>TIM1_CH1����ܽ�����
//  <0=>PA0
//  <4=>PA4
//  <10=>PA10
#define TIM1_CH3_PIN 			0
// </e>
// <e>	PWM-CH4
#define TIM1_CH4_ENABLE			1
// <o>DUTY4_CYCLE
//  <i>Ĭ��ռ�ձ�����
//  <0-100>
#define TIM1_DUTY4_CYCLE		50
// <o>TIM1-CH4���IO
//  <i>TIM1_CH4����ܽ�����
//  <1=>PA1
//  <11=>PA11
//  <15=>PA15
#define TIM1_CH4_PIN 			11
// </e>
#endif
// </e>

// <e>	ADC
//  <i> ADCģ�����ã�Ĭ��ʹ��12λ���ο���ѹѡ��ΪVCC������Ҫ�����ο���ѹ����VCC��ʱ����Ҫ�������ã�
#define APP_ADC_ENABLE			0
//  <i> 1����ȫadc_task.c�е�ADC_GPIO_Init��������IO������Ϊģ�⹦�ܣ���GPIO_Init(PA0, ANALOG);
//  <i> 2����ȫADC_Loop��������ADC���е�ʱ��adc_state == 2������APP_ADCConvert(ADC_CHANNEL_0, ADC_VREFBUF_1P5V)�������ݵĲɼ�
#if APP_ADC_ENABLE
// <o>ADC�������ʱ��
//  <i> ˳��ɨ����ٶȣ���λms
//  <0-100>
#define ADC_SPEED				10			//ADC�����ٶȣ���λms
// <o>	ADC-PA0-IN0
//  <i> PA0
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN0_ENABLE			0	
// <o>	ADC-PA1-IN1
//  <i> PA1
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN1_ENABLE			0	
// <o>	ADC-PA2-IN2
//  <i> PA2
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN2_ENABLE			0	
// <o>	ADC-PA3-IN3
//  <i> PA3
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN3_ENABLE			0	
// <o>	ADC-PA4-IN4
//  <i> PA4
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN4_ENABLE			0	
// <o>	ADC-PA5-IN5
//  <i> PA5
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN5_ENABLE			0
// <o>	ADC-PA6-IN6
//  <i> PA6
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN6_ENABLE			0	
// <o>	ADC-PA7-IN7
//  <i> PA7
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN7_ENABLE			0	
// <o>	ADC-PB0-IN8
//  <i> PB0
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN8_ENABLE			0	
// <o>	ADC-PB1-IN9
//  <i> PB1
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN9_ENABLE			0
// <o>	ADC-TEMPSENSOR
//  <i> �ڲ��¶ȴ�����
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN10_ENABLE			1
// <o>	ADC-VREFINT
//  <i> �ڲ�1.2V�ο���ѹ
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN11_ENABLE			1	
// <o>	ADC-1_3VCCA
//  <i> VCCA/3
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN12_ENABLE			0	
#endif
// </e>

// <e>	SPI-LED
// <i>  ����������W2812����
#define APP_SPI_LED_ENABLE			0
#if APP_SPI_LED_ENABLE
// <o>SPI_LED_PIN
//  <i>�������ÿ��ƹܽţ��õ�SPI-MOSI�ܽ�
//  <2=>PA2
//  <7=>PA7
//  <8=>PA8
//  <26=>NO_PIN
#define SPI_LED_PIN					2	
//  <o> LED����
//  <i>������Ҫˢ�µ�LED��������
//  <0-100>
#define SPI_LED_CNT					32
#endif
// </e>

// <e>	SMG	
// <i>  ���������������,��COM0��SEG0��ʼ�����û��ѡ��NO_PIN��ͨ���ı�smg_data��������ʾ����
#define APP_SMG_ENABLE			0		
#if APP_SMG_ENABLE
// <o>COM0-GPIO
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <26=>NO_PIN
#define COM0_GPIO_PIN 			12
// <o>COM1-GPIO
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <26=>NO_PIN
#define COM1_GPIO_PIN 			13
// <o>COM2-GPIO
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <26=>NO_PIN
#define COM2_GPIO_PIN 			14
// <o>COM3-GPIO
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <26=>NO_PIN
#define COM3_GPIO_PIN 			26
// <o>COM4-GPIO
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <26=>NO_PIN
#define COM4_GPIO_PIN 			26
// <o>SEG0-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG0_GPIO_PIN 			5
// <o>SEG1-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG1_GPIO_PIN 			8
// <o>SEG2-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG2_GPIO_PIN 			18
// <o>SEG3-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG3_GPIO_PIN 			1
// <o>SEG4-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG4_GPIO_PIN 			19
// <o>SEG5-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG5_GPIO_PIN 			0
// <o>SEG6-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG6_GPIO_PIN 			7
// <o>SEG7-GPIO
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
//  <26=>NO_PIN
#define SEG7_GPIO_PIN 			6
#endif
// </e>

// <e>	TM1624	
// <i>  TM1624��������,����TM1624_Display_Update����ˢ����ʾ
#define APP_TM1624_ENABLE	0
// <o>	CLK
// <i>  TM1624-CLK�ܽŶ���
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
#define TM1624_CLK			2
// <o>	DIN
// <i>  TM1624-DIN�ܽŶ���
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
#define TM1624_DIN			3
// <o>	STB
// <i>  TM1624-STB�ܽŶ���
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
#define TM1624_STB			1
// <o>	��ʾģʽ
// <i>  �趨��ʾģʽ
//  <0=>4λ 14��
//  <1=>5λ 13��
//  <2=>6λ 12��
//  <3=>7λ 11��
#define TM1624_DISP_COM		3
// <o>	��ʾ����
// <i>  �趨��ʾ����
//  <0=>1/16
//  <1=>2/16
//  <2=>4/16
//  <3=>10/16
//  <4=>11/16
//  <5=>12/16
//  <6=>13/16
//  <7=>14/16
#define TM1624_DISP_DIM		7
// <o>	����
// <i>  ��ʾ��ת����
//  <0=>DIASBLE
//  <1=>ENABLE
#define TM1624_DISP_TEST		1
// </e>

// <e>	TM1640	
// <i>  TM1640��������,����TM1640_Display_Update����ˢ����ʾ
#define APP_TM1640_ENABLE	0
// <o>	CLK
// <i>  TM1640-CLK�ܽŶ���
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
#define TM1640_CLK			2
// <o>	DIN
// <i>  TM1640-DIN�ܽŶ���
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
#define TM1640_DIN			3
// <o>	��ʾ����
// <i>  �趨��ʾ����
//  <0=>1/16
//  <1=>2/16
//  <2=>4/16
//  <3=>10/16
//  <4=>11/16
//  <5=>12/16
//  <6=>13/16
//  <7=>14/16
#define TM1640_DISP_DIM		7
// <o>	����
// <i>  ��ʾ��ת����
//  <0=>DIASBLE
//  <1=>ENABLE
#define TM1640_DISP_TEST		1
// </e>

// <e>	IR_RECEIVED	
// <i>  ���ں�����룬������NEC�����ʽ������IR_Press��ȡ��������
#define APP_IR_RECEIVED_ENABLE		0
// <o>	IR_IN
// <i>  ������չܽŶ���
//  <0=>PA0
//  <1=>PA1
//  <2=>PA2
//  <3=>PA3
//  <4=>PA4
//  <5=>PA5
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
//  <9=>PA9
//  <10=>PA10
//  <11=>PA11
//  <12=>PA12
//  <13=>PA13
//  <14=>PA14
//  <15=>PA15
//  <16=>PB0
//  <17=>PB1
//  <18=>PB2
//  <19=>PB3
//  <20=>PF0
//  <21=>PF1
//  <22=>PF2
//  <23=>PF3
//  <24=>PF4
//  <25=>PF5
#define IR_GPIO				20
// <o>IR_SAMPLE
// <i>  ���ⶨʱ��ѡ����Ҫ������Ӧ�Ķ�ʱ��ģ���Լ��趨��ʱʱ��
//  <1=>TIM1
//  <14=>TIM14
#define D_IR_TIM			1
// </e>
// <<< end of configuration section >>>


#ifndef NULL
#define NULL 0
#endif

typedef struct 
{
    uint8_t bit0 : 1;
    uint8_t bit1 : 1;
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} Bits8_TypeDef;

#if DEBUG_ENABLE
	#if (DEBUG_MODE == 0)
		#if (APP_TK_ENABLE == 0)
			#error "error! ʹ����λ����ӡ����Ҫʹ�ܴ���ģ��"
		#else
		extern char log_string[64];
		#define log_printf(...) 		do{snprintf(log_string,64,__VA_ARGS__);log_putstr(log_string);}while(0)
		#endif
	#elif (DEBUG_MODE == 1)
		#if (APP_UART_ENABLE == 0)
			#error "error! ʹ��UART��ӡ����Ҫʹ��UARTģ��"
		#else
			#if (PRINTF_UART == 1)	
				#if (UART1_ENABLE == 0)
				#error "error! ʹ��UART1��ӡ����Ҫ��Ӧ��UART1,�����ö�Ӧ��PIN"
				#else
				#define printf_UARTx			UART1					//����printfʹ�õĴ��ں�
				#endif
			#elif (PRINTF_UART == 2)
				#if (UART2_ENABLE == 0)
				#error "error! ʹ��UART2��ӡ����Ҫ��Ӧ��UART2,�����ö�Ӧ��PIN"
				#else
				#define printf_UARTx			UART2					//����printfʹ�õĴ��ں�
				#endif
			#else
				#if (UART3_ENABLE == 0)
				#error "error! ʹ��UART3��ӡ����Ҫ��Ӧ��UART3,�����ö�Ӧ��PIN"
				#else
				#define printf_UARTx			UART3					//����printfʹ�õĴ��ں�
				#endif
			#endif
		#endif
		#define log_printf(...) 		printf(__VA_ARGS__)		
	#endif
#else
	#define log_printf(...) 
#endif

#include "gpio.h"
#include "py32t020_flash.h"

#if	APP_TK_ENABLE
#include "tk_cfg.h"
#endif

#if APP_BEEP_ENABLE
#include "beep_drivers.h"
#endif

#if (APP_TIM14_ENABLE || APP_TIM14_PWM_ENABLE)
#include "tim14_drivers.h"
#endif

#if (APP_TIM14_ENABLE && APP_TIM14_PWM_ENABLE)
	#error "error! ����ͬʱ����TIM14�Լ�TIM14_PWM"
#endif

#if (APP_TIM1_ENABLE || APP_TIM1_PWM_ENABLE)
#include "tim1_drivers.h"
#endif

#if (APP_TIM1_ENABLE && APP_TIM1_PWM_ENABLE)
	#error "error! ����ͬʱ����TIM1�Լ�TIM1_PWM"
#endif

#if APP_ADC_ENABLE
#include "adc_drivers.h"
#define ADC_ENABLE_CHS			((ADC_IN0_ENABLE << 0) | (ADC_IN1_ENABLE << 1) | (ADC_IN2_ENABLE << 2) | 	\
								(ADC_IN3_ENABLE << 3) | (ADC_IN4_ENABLE << 4) | (ADC_IN5_ENABLE << 5) | 	\
								(ADC_IN6_ENABLE << 6) | (ADC_IN7_ENABLE << 7) | (ADC_IN8_ENABLE << 8) | 	\
								(ADC_IN9_ENABLE << 9) | (ADC_IN10_ENABLE << 10) | (ADC_IN11_ENABLE << 11) | \
								(ADC_IN12_ENABLE << 12))	
#endif

#if APP_UART_ENABLE
#include "uart_drivers.h"
#endif

#if APP_SPI_LED_ENABLE
#include "spi-led_drivers.h"
#include "argb_task.h"
#endif

#if APP_USER_OTP_ENABLE
#include "user-otp_drivers.h"
	#if LVD_WRITE_USER_DATA
		#if (APP_ADC_ENABLE == 0)
		#error "error! ���������鹦�ܣ���Ҫʹ��ADCģ�飬��ʹ��ADC-VREFINT"
		#else
		extern uint8_t power_count;
		#endif
	#endif
#endif

#if APP_SMG_ENABLE
#include "smg_drivers.h"
#if (SEG0_GPIO_PIN == NO_PIN)
	#error "error! ����ʹ��1��SEG"
	#define SEG_COUNT		0
#elif (SEG1_GPIO_PIN == NO_PIN)	
	#define SEG_COUNT		1
#elif (SEG2_GPIO_PIN == NO_PIN)	
	#define SEG_COUNT		2
#elif (SEG3_GPIO_PIN == NO_PIN)	
	#define SEG_COUNT		3
#elif (SEG4_GPIO_PIN == NO_PIN)	
	#define SEG_COUNT		4
#elif (SEG5_GPIO_PIN == NO_PIN)	
	#define SEG_COUNT		5
#elif (SEG6_GPIO_PIN == NO_PIN)	
	#define SEG_COUNT		6
#elif (SEG7_GPIO_PIN == NO_PIN)	
	#define SEG_COUNT		7
#else
	#define SEG_COUNT		8
#endif
#if (COM0_GPIO_PIN == NO_PIN)
	#error "error! ����ʹ��1��COM"
	#define COM_COUNT		0
#elif (COM1_GPIO_PIN == NO_PIN)	
	#define COM_COUNT		1
#elif (COM2_GPIO_PIN == NO_PIN)	
	#define COM_COUNT		2
#elif (COM3_GPIO_PIN == NO_PIN)	
	#define COM_COUNT		3
#elif (COM4_GPIO_PIN == NO_PIN)	
	#define COM_COUNT		4
#else
	#define COM_COUNT		5
#endif
#endif

#if APP_TM1624_ENABLE
#include "tm1624.h"
#endif


#if APP_TM1640_ENABLE
#include "tm1640.h"
#endif

#if APP_IR_RECEIVED_ENABLE
#if (D_IR_TIM == 1)
#if (APP_TIM1_ENABLE == 0)
	#error "error! ��ʱ��1û��ʹ��"
#endif
#define D_IR_sample			TIM1_TIMING_TIME
#elif (D_IR_TIM == 14)
#if (APP_TIM14_ENABLE == 0)
	#error "error! ��ʱ��14û��ʹ��"
#endif
#define D_IR_sample			TIM14_TIMING_TIME
#else
	#error "error! ������ն�ʱ���������"
#endif
#if (D_IR_sample < 60 ||D_IR_sample > 250)
	#error "error! ��ʱ��ʱ�䳬����Ч��Χ"
#endif
#include "ir_received.h"
#endif

void nop_delay_xus(uint16_t nus);

#endif

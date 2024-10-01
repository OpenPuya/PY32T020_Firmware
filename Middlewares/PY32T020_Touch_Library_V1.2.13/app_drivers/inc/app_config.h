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
//  <i> 设置系统时钟（默认24M）
//  <24000000UL=> 24MHz
//  <48000000UL=> 48MHz
#define HSI_FREQUENCE 		24000000UL

#define fac_us  			(HSI_FREQUENCE / 1000000)
//  <o>	TK_MODULE
//  <i> 开启或关闭触摸库
//  <0=> DISABLE
//  <1=> ENABLE
#define	APP_TK_ENABLE			1	

//  <e>	SYSTICK
//  <i>使用SYSTICK进行定时，定时器中断服务函数为SysTick_Handler,为触摸库提供时基
#define APP_SYSTICK_ENABLE			1	

#if APP_SYSTICK_ENABLE
//  <o>定时时间(单位us)
//  <i>设置系统定时器时间，最慢1ms
//  <0-1000>
#define SYSTICK_TIMING_TIME			1000
//  <o> SYSTICK中断优先级
//  <i>Default: LOWEST
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define SYSTICK_IRQ_PRIORITY			0
// <e>	中断服务程序IO口翻转测试
#define SYSTICK_DEBUG					0
// <o.0..15>IO选择
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
//  <i>	打印用户调试信息,调用log_printf进行打印，使用方法同printf
#define DEBUG_ENABLE			0
#if DEBUG_ENABLE
//  <e>	UART
//  <i> 勾选为使用串口打印，默认为触摸调试上位机打印(需开启触摸功能)
#define DEBUG_MODE				0	
//  <o>	UART选择
//  <i> 需要同时开启对应的UART模块,并配置相应的参数
//  <1=> UART1
//  <2=> UART2
//  <3=> UART3
#define PRINTF_UART				1	
// </e>
// </e>
#endif

// <e>	UART
//  <i> UART配置，默认为8位数据位，1个停止位，只需配置管脚以及波特率即可
#define APP_UART_ENABLE			0				//使能串口驱动	
#if APP_UART_ENABLE
// <e>	UART1
//  <i> UART1配置，使能中断则使用队列发送以及接收，队列长度为64字节
#define UART1_ENABLE			1
/*
	UART1_TX		PF3/PA6/PA11
	UART1_RX		PF4/PA7/PA12
*/
#if UART1_ENABLE
// <o.0..15>UART1_TX_PIN
//  <i> UART1发送管脚配置，初始化完成后调用UART1_QueueSend即可进行发送，选择NO_PIN则关闭发送功能
//  <6=>PA6
//  <11=>PA11
//  <23=>PF3
//  <26=>NO_PIN
#define UART1_TX_PIN				6
// <o.0..15>UART1_RX_PIN
//  <i> UART1接收管脚配置，初始化完成后调用UART1_QueueRead查询数据，返回1时表示数据有效，选择NO_PIN则关闭接收功能
//  <7=>PA7
//  <12=>PA12
//  <24=>PF4
//  <26=>NO_PIN
#define UART1_RX_PIN				7
// <o>UART1波特率
// <i>UART波特率设置
// <1200=> 1200 baud 
// <2400=> 2400 baud 
// <4800=> 4800 baud 
// <9600=> 9600 baud 
// <19200=> 19200 baud 
// <38400=> 38400 baud 
// <57600=> 57600 baud 
// <115200=> 115200 baud 
#define UART1_BAUDRATE				115200
//  <o> UART1中断使能
//  <i>UART中断开关，默认打开，关闭情况下只能使用查询的方式进行发送接收
//  <0=> DISABLE
//  <1=> ENABLE
#define UART1_IRQ_ENABLE			1
//  <o> UART1中断优先级
//  <i>UART中断优先级设定
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
//  <i> UART2配置，使能中断则使用队列发送以及接收，队列长度为64字节
#define UART2_ENABLE				1
#if UART2_ENABLE
// <o.0..15>UART2_TX_PIN
//  <i> UART2发送管脚配置，初始化完成后调用UART2_QueueSend即可进行发送，选择NO_PIN则关闭发送功能
//  <0=>PA0
//  <4=>PA4
//  <9=>PA9
//  <13=>PA13
//  <16=>PB0
//  <26=>NO_PIN
#define UART2_TX_PIN				0
// <o.0..15>UART2_RX_PIN
//  <i> UART2接收管脚配置，初始化完成后调用UART2_QueueRead查询数据，返回1时表示数据有效，选择NO_PIN则关闭接收功能
//  <1=>PA1
//  <5=>PA5
//  <10=>PA10
//  <17=>PB1
//  <19=>PB3
//  <26=>NO_PIN
#define UART2_RX_PIN				1
// <o>UART2波特率
// <i>UART波特率设置
// <1200=> 1200 baud 
// <2400=> 2400 baud 
// <4800=> 4800 baud 
// <9600=> 9600 baud 
// <19200=> 19200 baud 
// <38400=> 38400 baud 
// <57600=> 57600 baud 
// <115200=> 115200 baud 
#define UART2_BAUDRATE				115200
//  <o> UART2中断使能
//  <i>UART中断开关，默认打开，关闭情况下只能使用查询的方式进行发送接收
//  <0=> DISABLE
//  <1=> ENABLE
#define UART2_IRQ_ENABLE			1
//  <o> UART2中断优先级
//  <i>UART中断优先级设定
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
//  <i> UART3配置，使能中断则使用队列发送以及接收，队列长度为64字节
#define UART3_ENABLE				1
#if UART3_ENABLE
// <o.0..15>UART3_TX_PIN
//  <i> UART3发送管脚配置，初始化完成后调用UART3_QueueSend即可进行发送，选择NO_PIN则关闭发送功能
//  <2=>PA2
//  <8=>PA8
//  <14=>PA14
//  <26=>NO_PIN
#define UART3_TX_PIN				2
// <o.0..15>UART3_RX_PIN
//  <i> UART3接收管脚配置，初始化完成后调用UART3_QueueRead查询数据，返回1时表示数据有效，选择NO_PIN则关闭接收功能
//  <3=>PA3
//  <15=>PA15
//  <18=>PB2
//  <26=>NO_PIN
#define UART3_RX_PIN				3
// <o>UART3波特率
// <i>UART波特率设置
// <1200=> 1200 baud 
// <2400=> 2400 baud 
// <4800=> 4800 baud 
// <9600=> 9600 baud 
// <19200=> 19200 baud 
// <38400=> 38400 baud 
// <57600=> 57600 baud 
// <115200=> 115200 baud 
#define UART3_BAUDRATE				115200
//  <o> UART3中断使能
//  <i>UART中断开关，默认打开，关闭情况下只能使用查询的方式进行发送接收
//  <0=> DISABLE
//  <1=> ENABLE
#define UART3_IRQ_ENABLE				1
//  <o> UART3中断优先级
//  <i>UART中断优先级设定
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
//  <i> 用于配置低电压复位、硬件看门狗、复位脚复用为GPIO
#define OB_USER_OPTION				0		
#if OB_USER_OPTION
//  <o> 设置PF2(NRST)模式
//  <i> PF2默认为复位功能,当需要PF2作为GPIO或触摸功能时需配置为GPIO模式
//  <0=> RST
//  <1=> GPIO
#define OPTION_NRST_MODE			0
#if OPTION_NRST_MODE
#define OB_GPIO_PIN_MODE		FLASH_OPTR_NRST_MODE                                /*!< PF2: GPIO */
#else
#define OB_GPIO_PIN_MODE		0x00000000U                                         /*!< PF2: NRST */
#endif
//  <o> 低电压复位使能
//  <i> 使能后，当电压低于阈值时单片机产生复位
//  <0=> DISABLE
//  <1=> ENABLE
#define OPTION_BOR_EN				0
#if OPTION_BOR_EN
#define OB_BOR_EN					FLASH_OPTR_BOR_EN	
#else
#define OB_BOR_EN					0x00000000U	
#endif
//  <o> 低电压阈值
//  <i> 用于设置低电压阈值
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
//  <o> 硬件看门狗
//  <i> 硬件看门狗(上电自动运行)，软件看门狗(软件开启关闭)
//  <0=> 硬件看门狗
//  <1=> 软件看门狗
#define OPTION_IWDG_SW_EN				1
#if OPTION_IWDG_SW_EN
#define OB_IWDG_MODE					FLASH_OPTR_IWDG_SW	
#else
#define OB_IWDG_MODE					0x00000000U	
#endif
//  <o> 看门狗在STOP模式下运行状态
//  <i> 用于设置看门狗在STOP模式下的状态
//  <0=> 停止运行
//  <1=> 正常运行
#define OPTION_IWDG_STOP				0
#if OPTION_IWDG_STOP
#define OB_IWDG_STOP					((uint32_t)FLASH_OPTR_IWDG_STOP) /*!< IWDG counter active in STOP mode */
#else
#define OB_IWDG_STOP					0x00000000U	
#endif
// </e>

#endif

// <e>	Flash User OTP	
// <i>  使用USER_OTP区域保存用户数据，共124字节,调用User_Cache_Read读取数据，返回1时数据有效，调用User_Cache_Write将数据写入缓存，调用User_Flash_Write将缓存数据写入FLASH
#define APP_USER_OTP_ENABLE			0
#if APP_USER_OTP_ENABLE
// <e>	掉电保存数据
// <i>  需要开启ADC模块以及使能VREFINT通道
#define LVD_WRITE_USER_DATA			1			
//  <o> 掉电检测电压
//  <i>单位:MV 掉电电压不可设置得过低，防止电压下降过快造成数据无法正常保存
//  <3000-4200>
#define low_voltage					3100
// </e>
#endif
// </e>

// <e>	IWDG
//  <i> 用于配置软件看门狗，设置超时时间
#define APP_IWDG_ENABLE			0				//使能串口驱动	
//  <o> 看门狗超时时间
//  <i>单位:ms 看门狗超时时间
//  <0-4095>
#define IDWG_RELOAD_VALUE		1500
// </e>

//  <e>	BEEP
//  <i> 仅适用于有源蜂鸣器
#define APP_BEEP_ENABLE					0	

#if APP_BEEP_ENABLE
// <o.0..15>蜂鸣器控制管脚
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
//  <o> 蜂鸣器开启电平
//  <0=> 低电平有效
//  <1=> 高电平有效
#define BEEP_GPIO_TYPE				1
#endif
// </e>

//  <e>	TIM14
//  <i>使用TIM14进行定时，定时器中断服务函数为TIM14_PeriodElapsedCallback
#define APP_TIM14_ENABLE			0	

#if APP_TIM14_ENABLE
//  <o>定时时间(单位us)
//  <i>Default: 125 (Unit:us)
//  <0-1000000>
#define TIM14_TIMING_TIME			125
//  <o> TIM14中断使能
//  <i>Default: DISABLE
//  <0=> DISABLE
//  <1=> ENABLE
#define TIM14_IRQ_ENABLE			1
//  <o> TIM14中断优先级
//  <i>Default: LOWEST
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define TIM14_IRQ_PRIORITY			0
// <e>	中断服务程序IO口翻转测试
#define TIM14_DEBUG					0
// <o.0..15>IO选择
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
//  <i>使用TIM14进行PWM输出,与TIM14定时器资源互斥，调用void TIM14_PWM_Pulse(uint32_t CHx,uint8_t percent)设置百分比
#define APP_TIM14_PWM_ENABLE		0				
#if APP_TIM14_PWM_ENABLE
//  <o>PWM频率设置
//  <i>设置PWM频率，单位HZ
//  <0-1000000>
#define TIM14_PWM_FREQUENCE			25000
// <o>TIM14_RESOLUTION
//  <i>分辨率设置
//  <0-65535>
#define TIM14_RESOLUTION			100
// <o>TIM14_DUTY_CYCLE
//  <i>默认占空比设置
//  <0-65535>
#define TIM14_DUTY_CYCLE			10
// <o>TIM14_CH1_PIN
//  <i>TIM14_CH1输出管脚设置
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
//  <i>使用TIM1进行定时，定时器中断服务函数为TIM1_PeriodElapsedCallback
#define APP_TIM1_ENABLE				0	

#if APP_TIM1_ENABLE
//  <o>定时时间(单位us)
//  <i>Default: 125 (Unit:us)
//  <0-1000000>
#define TIM1_TIMING_TIME			125
//  <o> TIM1中断使能
//  <i>Default: DISABLE
//  <0=> DISABLE
//  <1=> ENABLE
#define TIM1_IRQ_ENABLE				1
//  <o> TIM1中断优先级
//  <i>Default: LOWEST
//  <0=> HIGHEST
//  <1=> HIGH
//  <2=> LOW 
//  <3=> LOWEST
#define TIM1_IRQ_PRIORITY			0
// <e>	中断服务程序IO口翻转测试
#define TIM1_DEBUG					0
// <o.0..15>IO选择
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
//  <i>使用TIM1进行PWM输出,与TIM1定时器资源互斥，调用void TIM1_PWM_Pulse(uint32_t CHx,uint8_t percent)设置百分比
#define APP_TIM1_PWM_ENABLE		0				
#if APP_TIM1_PWM_ENABLE
//  <o>PWM频率设置
//  <i>设置PWM频率，单位HZ
//  <0-1000000>
#define TIM1_PWM_FREQUENCE		4000
// <o>TIM1_RESOLUTION
//  <i>分辨率设置
//  <0-65535>
#define TIM1_RESOLUTION			100
// <e>	PWM-CH1
#define TIM1_CH1_ENABLE			1
// <o>DUTY1_CYCLE
//  <i>默认占空比设置
//  <0-100>
#define TIM1_DUTY1_CYCLE		10
// <o>TIM1_CH1_PIN
//  <i>TIM1_CH1输出管脚设置
//  <3=>PA3
//  <6=>PA6
//  <7=>PA7
//  <8=>PA8
#define TIM1_CH1_PIN 			3
// </e>
// <e>	PWM-CH2
#define TIM1_CH2_ENABLE			1
// <o>DUTY2_CYCLE
//  <i>默认占空比设置
//  <0-100>
#define TIM1_DUTY2_CYCLE		20
// <o>TIM1_CH2_PIN
//  <i>TIM1_CH2输出管脚设置
//  <5=>PA5
//  <16=>PB0
//  <19=>PB3
#define TIM1_CH2_PIN 			5
// </e>
// <e>	PWM-CH3
#define TIM1_CH3_ENABLE			1
// <o>DUTY3_CYCLE
//  <i>默认占空比设置
//  <0-100>
#define TIM1_DUTY3_CYCLE		30
// <o>TIM1_CH3_PIN
//  <i>TIM1_CH1输出管脚设置
//  <0=>PA0
//  <4=>PA4
//  <10=>PA10
#define TIM1_CH3_PIN 			0
// </e>
// <e>	PWM-CH4
#define TIM1_CH4_ENABLE			1
// <o>DUTY4_CYCLE
//  <i>默认占空比设置
//  <0-100>
#define TIM1_DUTY4_CYCLE		50
// <o>TIM1-CH4输出IO
//  <i>TIM1_CH4输出管脚设置
//  <1=>PA1
//  <11=>PA11
//  <15=>PA15
#define TIM1_CH4_PIN 			11
// </e>
#endif
// </e>

// <e>	ADC
//  <i> ADC模块配置，默认使用12位，参考电压选择为VCC，当需要测量参考电压不是VCC的时候需要单独配置，
#define APP_ADC_ENABLE			0
//  <i> 1、补全adc_task.c中的ADC_GPIO_Init函数，将IO口设置为模拟功能，如GPIO_Init(PA0, ANALOG);
//  <i> 2、补全ADC_Loop函数，在ADC空闲的时候（adc_state == 2）调用APP_ADCConvert(ADC_CHANNEL_0, ADC_VREFBUF_1P5V)进行数据的采集
#if APP_ADC_ENABLE
// <o>ADC采样间隔时间
//  <i> 顺序扫描的速度，单位ms
//  <0-100>
#define ADC_SPEED				10			//ADC采样速度，单位ms
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
//  <i> 内部温度传感器
//  <0=> DISABLE
//  <1=> ENABLE
#define ADC_IN10_ENABLE			1
// <o>	ADC-VREFINT
//  <i> 内部1.2V参考电压
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
// <i>  用于驱动类W2812灯珠
#define APP_SPI_LED_ENABLE			0
#if APP_SPI_LED_ENABLE
// <o>SPI_LED_PIN
//  <i>用于设置控制管脚，用到SPI-MOSI管脚
//  <2=>PA2
//  <7=>PA7
//  <8=>PA8
//  <26=>NO_PIN
#define SPI_LED_PIN					2	
//  <o> LED数量
//  <i>设置需要刷新的LED灯珠数量
//  <0-100>
#define SPI_LED_CNT					32
#endif
// </e>

// <e>	SMG	
// <i>  驱动共阴极数码管,从COM0跟SEG0开始填，后面没有选择NO_PIN，通过改变smg_data来设置显示内容
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
// <i>  TM1624驱动定义,调用TM1624_Display_Update进行刷新显示
#define APP_TM1624_ENABLE	0
// <o>	CLK
// <i>  TM1624-CLK管脚定义
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
// <i>  TM1624-DIN管脚定义
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
// <i>  TM1624-STB管脚定义
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
// <o>	显示模式
// <i>  设定显示模式
//  <0=>4位 14段
//  <1=>5位 13段
//  <2=>6位 12段
//  <3=>7位 11段
#define TM1624_DISP_COM		3
// <o>	显示亮度
// <i>  设定显示亮度
//  <0=>1/16
//  <1=>2/16
//  <2=>4/16
//  <3=>10/16
//  <4=>11/16
//  <5=>12/16
//  <6=>13/16
//  <7=>14/16
#define TM1624_DISP_DIM		7
// <o>	测试
// <i>  显示翻转测试
//  <0=>DIASBLE
//  <1=>ENABLE
#define TM1624_DISP_TEST		1
// </e>

// <e>	TM1640	
// <i>  TM1640驱动定义,调用TM1640_Display_Update进行刷新显示
#define APP_TM1640_ENABLE	0
// <o>	CLK
// <i>  TM1640-CLK管脚定义
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
// <i>  TM1640-DIN管脚定义
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
// <o>	显示亮度
// <i>  设定显示亮度
//  <0=>1/16
//  <1=>2/16
//  <2=>4/16
//  <3=>10/16
//  <4=>11/16
//  <5=>12/16
//  <6=>13/16
//  <7=>14/16
#define TM1640_DISP_DIM		7
// <o>	测试
// <i>  显示翻转测试
//  <0=>DIASBLE
//  <1=>ENABLE
#define TM1640_DISP_TEST		1
// </e>

// <e>	IR_RECEIVED	
// <i>  用于红外解码，适用于NEC传输格式，调用IR_Press获取红外数据
#define APP_IR_RECEIVED_ENABLE		0
// <o>	IR_IN
// <i>  红外接收管脚定义
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
// <i>  红外定时器选择，需要开启对应的定时器模块以及设定定时时间
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
			#error "error! 使用上位机打印，需要使能触摸模块"
		#else
		extern char log_string[64];
		#define log_printf(...) 		do{snprintf(log_string,64,__VA_ARGS__);log_putstr(log_string);}while(0)
		#endif
	#elif (DEBUG_MODE == 1)
		#if (APP_UART_ENABLE == 0)
			#error "error! 使用UART打印，需要使能UART模块"
		#else
			#if (PRINTF_UART == 1)	
				#if (UART1_ENABLE == 0)
				#error "error! 使用UART1打印，需要对应的UART1,并配置对应的PIN"
				#else
				#define printf_UARTx			UART1					//定义printf使用的串口号
				#endif
			#elif (PRINTF_UART == 2)
				#if (UART2_ENABLE == 0)
				#error "error! 使用UART2打印，需要对应的UART2,并配置对应的PIN"
				#else
				#define printf_UARTx			UART2					//定义printf使用的串口号
				#endif
			#else
				#if (UART3_ENABLE == 0)
				#error "error! 使用UART3打印，需要对应的UART3,并配置对应的PIN"
				#else
				#define printf_UARTx			UART3					//定义printf使用的串口号
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
	#error "error! 不能同时开启TIM14以及TIM14_PWM"
#endif

#if (APP_TIM1_ENABLE || APP_TIM1_PWM_ENABLE)
#include "tim1_drivers.h"
#endif

#if (APP_TIM1_ENABLE && APP_TIM1_PWM_ENABLE)
	#error "error! 不能同时开启TIM1以及TIM1_PWM"
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
		#error "error! 开启掉电检查功能，需要使能ADC模块，并使能ADC-VREFINT"
		#else
		extern uint8_t power_count;
		#endif
	#endif
#endif

#if APP_SMG_ENABLE
#include "smg_drivers.h"
#if (SEG0_GPIO_PIN == NO_PIN)
	#error "error! 最少使能1个SEG"
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
	#error "error! 最少使能1个COM"
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
	#error "error! 定时器1没有使能"
#endif
#define D_IR_sample			TIM1_TIMING_TIME
#elif (D_IR_TIM == 14)
#if (APP_TIM14_ENABLE == 0)
	#error "error! 定时器14没有使能"
#endif
#define D_IR_sample			TIM14_TIMING_TIME
#else
	#error "error! 红外接收定时器定义错误"
#endif
#if (D_IR_sample < 60 ||D_IR_sample > 250)
	#error "error! 定时器时间超过有效范围"
#endif
#include "ir_received.h"
#endif

void nop_delay_xus(uint16_t nus);

#endif

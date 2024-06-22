#ifndef _TK_CFG_H
#define _TK_CFG_H
#include "tk_lib.h"
#include "tk_cfg_user.h"

// <<< Use Configuration Wizard in Context Menu >>>
//  <h> 工作配置
//  <i> 设置电容模式，参考电压，扫描窗口宽度等
//  <o>	CMOD电容
//  <i> 设置CMOD外接或内置
//  <0=> 内部
//  <1=> 外部
#define EXTCMOD						0	
//  <o>	参考电压
//  <i> 设置触摸比较器参考电压
//  <0=> 0.6V
//  <1=> 1.0V
//  <2=> 1.5V
//  <3=> 2.0V
#define CONFIG_VREF					3			
//  <o> 扫描窗口
//  <i> 采样窗口时间 = win * (1/Ftk_clk)，如Ftk_clk = 24M, win=12000, 即采样窗口时间为500us。
//  <0-65535>
#define CONFIG_WIN					12000		
//  <e>	自适应电流
//  <i> 自动设置电流
#define CONFIG_IDAC0				1	
//  <o> 灵敏度均衡
//  <i> 使各通道灵敏度趋向一致
//  <0=> DISABLE
//  <1=> ENABLE
#define CONFIG_IDAC1				1	
// </e>
//  </h>	

//  <h> 扫描模式
//  <i> 设置SW模式以及频率
//  <o>分频系数
//  <i>PWM:SWCLK = APB_CLK / SW_DIV，定义为6，即SWCLK = 4M
//  <i>PRS:SWCLK = APB_CLK / (SW_DIV * 2)，定义为8，即SWCLK = 1.5M
//  <1-511>
#define SW_DIV					6	
//  <e>	跳频模式
//  <i> 用于开启跳频
#define SW_MODE					1	
//  <o>跳频模式多项式宽度
//  <i>范围：1 ~ 15
//  <1-15>
#define PWMM_PRS_LFSRW		8					
//  <o>随机数种子
//  <i>（最低位必须为1）
//  <1-255>
#define PWMM_PRS_SEED		0XA7		
//  </e>	
//  </h>	

//  <e>	输出补偿
//  <i> 未扫描输出控制 
#define COPA_ENABLE				1	
//  <o>	补偿输出电压
//  <i> 设置补偿电压
//  <0=> 1.6V
//  <1=> 2.0V
//  <2=> 2.4V
//  <3=> 2.8V
//  <4=> 3.2V
//  <5=> 3.6V
//  <6=> 4.0V
//  <7=> 4.4V
#define CONFIG_VCC1				1																				
//  <o>	补偿输出类型
//  <i> 设置输出状态
//  <0=> 低电平
//  <1=> 高电平
//  <2=> 反相
//  <3=> 同相
#define COPADATA_OUTPUT			0	
//  </e>	

//  <e>	触摸省电
//  <i> 用于设置触摸省电模式，无操作一段时间后进入休眠模式
#define SLEEP_EN		   	0	
//  <o>无操作进入休眠时间
//  <i>进入触摸省电模式倒计时时间，单位为5ms，定义为200即为1秒。
//  <0-65535>
#define ENTER_SLEEPTIME		200  	
//  <o>扫描触摸的间隔时间
//  <i>单位为1ms，最大值为1000，定义为100即为100ms。注：此时间越长，触摸待机功耗越低，触摸按键响应速度越慢。
//  <0-1000>
#define LP_WAUP_TIME		100		
//  <o>触摸省电模式下门限值
//  <i>当触摸数据变化量大于此数值将退出省电模式，设置过大会造成触摸较难唤醒，设置过小会造成触摸误唤醒（休眠功耗也会对应增加）
//  <0-65535>
#define LP_NORMRMALDELTA	15		
//  <o>休眠模式窗口值
//  <i>设置低功耗触摸扫描窗口时间，时间越短，功耗越低
//  <0-65535>
#define CONFIG_LP_WIN		6000				
//  <o>	参考电压
//  <i> 设置休眠状态下触摸比较器的参考电压，电压越低，功耗越低
//  <0=> 0.6V
//  <1=> 1.0V
//  <2=> 1.5V
//  <3=> 2.0V
#define CONFIG_LPVREF		0																												
//  </e>	

//  <h>	参数设置
//  <i> 设置触摸参数
//  <o>NOISE门限值
//  <i>噪音数值大小
//  <0-100>
#define NOISE_THD_DEFAULT	15								//
//  <o>触摸数据滤波次数
//  <i>范围：1~30，此数值越大，数据越平滑，但是按键响应速度越慢
//  <1-30>
#define FILTERCOUNT			5								//	
//  <o>限制按键最长输出时间
//  <i>单位为ms，最长为2分钟，若定义为0，则关闭此功能	
//  <0-120000>
#define KEY_OUT_MAX_TIME					30000

#define KEY_OUT_MAX_TICK_CNT				(KEY_OUT_MAX_TIME / 5)	
//  <o>触摸按键按下消抖时间
//  <i>单位为5ms
//  <0-100>
#define FINGER_CONFIRM_TICK_CNT					(5) 		
//  <o>触摸按键释放消抖时间
//  <i>单位为5ms
//  <0-100>
#define FINGER_RELEASE_CONFIRM_TICK_CNT			(5)  		
//  <o>WATER AREA更新基线时间
//  <i>单位为5ms
//  <0-100>
#define WATER_AREA_CONFIRM_TICK_CNT				(150)		//单位为5ms
//  <o>NOISE AREA更新基线时间
//  <i>单位为5ms
//  <0-100>
#define NOISE_AREA_CONFIRM_TICK_CNT				(100)		//单位为5ms
//  <o>BOTTON AREA更新基线时间
//  <i>单位为5ms
//  <0-100>
#define BOTTON_AREA_CONFIRM_TICK_CNT			(60)		//单位为5ms
//  <o>	单键触发
//  <i> 设置单键触发
//  <0=> DISABLE
//  <1=> ENABLE
#define SIGLE_KEY_TRIGGER			0		
//  <o>	设置多按键的数量
//  <i> 当多按键同时触发时强制更新BASELINE,设置为0即功能关闭
//  <0-255>
#define MAX_TRIGGER_KEY_CNT 		0				
// </h>

//  <e>	内部参考通道
//  <i> 设置内部参考通道
#define REF_CH_EN				1			
//  <o> 门限值
//  <i> 设置内部参考通道门限值
//  <0-2000>
#define REF_CH_THD				40			
// </e>


// <<< end of configuration section >>>

//******************************************************************************************************************************************************
#if (CH_KEY0 == TK_CH_NONE)
	#define KEY_CH_TOTAL		0
#elif (CH_KEY1 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		1
#elif (CH_KEY2 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		2
#elif (CH_KEY3 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		3
#elif (CH_KEY4 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		4
#elif (CH_KEY5 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		5
#elif (CH_KEY6 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		6
#elif (CH_KEY7 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		7
#elif (CH_KEY8 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		8
#elif (CH_KEY9 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		9
#elif (CH_KEY10 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		10
#elif (CH_KEY11 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		11
#elif (CH_KEY12 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		12
#elif (CH_KEY13 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		13
#elif (CH_KEY14 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		14
#elif (CH_KEY15 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		15
#elif (CH_KEY16 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		16
#elif (CH_KEY17 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		17
#elif (CH_KEY18 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		18
#elif (CH_KEY19 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		19
#elif (CH_KEY20 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		20
#elif (CH_KEY21 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		21
#elif (CH_KEY22 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		22
#elif (CH_KEY23 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		23
#elif (CH_KEY24 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		24
#elif (CH_KEY25 == TK_CH_NONE) 
	#define KEY_CH_TOTAL		25
#else
	#define KEY_CH_TOTAL		26
#endif

#if WATER_PROOF_EN	
	#define SHIELD_CH_TOTAL  1
#elif HIGH_SENSITVITY_EN
#if (SENSITVITY_SHIELD_CH == TK_CH_NONE)
	#define SHIELD_CH_TOTAL  0
#else
	#define SHIELD_CH_TOTAL  1
#endif
#else
	#define SHIELD_CH_TOTAL  0
#endif

#if (WATER_PROOF_EN && HIGH_SENSITVITY_EN)
	#error "error!"
#endif


#if (SLIDER_OR_WHEEL0_TYPE != TK_APP_NONE)
	#if (SLIDER_OR_WHEEL0_CH0 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL0_CH1 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL0_CH2 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL0_CH_CNT	2
	#elif (SLIDER_OR_WHEEL0_CH3 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL0_CH_CNT	3
	#elif (SLIDER_OR_WHEEL0_CH4 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL0_CH_CNT	4
	#elif (SLIDER_OR_WHEEL0_CH5 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL0_CH_CNT	5
	#elif (SLIDER_OR_WHEEL0_CH6 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL0_CH_CNT	6
	#elif (SLIDER_OR_WHEEL0_CH7 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL0_CH_CNT	7
	#else
		#define SLIDER_OR_WHEEL0_CH_CNT	8
	#endif
#else 
	#define SLIDER_OR_WHEEL0_CH_CNT	0
#endif
#if (SLIDER_OR_WHEEL1_TYPE != TK_APP_NONE)
	#if (SLIDER_OR_WHEEL1_CH0 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL1_CH1 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL1_CH2 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL1_CH_CNT	2
	#elif (SLIDER_OR_WHEEL1_CH3 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL1_CH_CNT	3
	#elif (SLIDER_OR_WHEEL1_CH4 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL1_CH_CNT	4
	#elif (SLIDER_OR_WHEEL1_CH5 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL1_CH_CNT	5
	#elif (SLIDER_OR_WHEEL1_CH6 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL1_CH_CNT	6
	#elif (SLIDER_OR_WHEEL1_CH7 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL1_CH_CNT	7
	#else
		#define SLIDER_OR_WHEEL1_CH_CNT	8
	#endif
#else 
	#define SLIDER_OR_WHEEL1_CH_CNT	0
#endif
#if (SLIDER_OR_WHEEL2_TYPE != TK_APP_NONE)
	#if (SLIDER_OR_WHEEL2_CH0 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL2_CH1 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL2_CH2 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL2_CH_CNT	2
	#elif (SLIDER_OR_WHEEL2_CH3 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL2_CH_CNT	3
	#elif (SLIDER_OR_WHEEL2_CH4 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL2_CH_CNT	4
	#elif (SLIDER_OR_WHEEL2_CH5 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL2_CH_CNT	5
	#elif (SLIDER_OR_WHEEL2_CH6 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL2_CH_CNT	6
	#elif (SLIDER_OR_WHEEL2_CH7 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL2_CH_CNT	7
	#else
		#define SLIDER_OR_WHEEL2_CH_CNT	8
	#endif
#else 
	#define SLIDER_OR_WHEEL2_CH_CNT	0
#endif
#if (SLIDER_OR_WHEEL3_TYPE != TK_APP_NONE)
	#if (SLIDER_OR_WHEEL3_CH0 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL3_CH1 == TK_CH_NONE)
		#error "error!"
	#elif (SLIDER_OR_WHEEL3_CH2 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL3_CH_CNT	2
	#elif (SLIDER_OR_WHEEL3_CH3 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL3_CH_CNT	3
	#elif (SLIDER_OR_WHEEL3_CH4 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL3_CH_CNT	4
	#elif (SLIDER_OR_WHEEL3_CH5 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL3_CH_CNT	5
	#elif (SLIDER_OR_WHEEL3_CH6 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL3_CH_CNT	6
	#elif (SLIDER_OR_WHEEL3_CH7 == TK_CH_NONE)
		#define SLIDER_OR_WHEEL3_CH_CNT	7
	#else
		#define SLIDER_OR_WHEEL3_CH_CNT	8
	#endif
#else 
	#define SLIDER_OR_WHEEL3_CH_CNT		0
#endif

#if (REF_CH_EN)
#define EXTERNAL_CHS		INSIDE_CHS		/*	外部参考通道 */
#define EXTERNAL_CNT		1
#else
#define EXTERNAL_CNT		0
#endif

#define SLIDER_OR_WHEEL_CH_TOTAL	(SLIDER_OR_WHEEL0_CH_CNT + SLIDER_OR_WHEEL1_CH_CNT + SLIDER_OR_WHEEL2_CH_CNT + SLIDER_OR_WHEEL3_CH_CNT)

#define TK_CHMAX  					(KEY_CH_TOTAL + EXTERNAL_CNT + SHIELD_CH_TOTAL + SLIDER_OR_WHEEL_CH_TOTAL)

#endif

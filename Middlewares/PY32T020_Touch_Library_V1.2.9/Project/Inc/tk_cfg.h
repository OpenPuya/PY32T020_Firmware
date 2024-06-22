#ifndef _TK_CFG_H
#define _TK_CFG_H
#include "tk_lib.h"
#include "tk_cfg_user.h"

// <<< Use Configuration Wizard in Context Menu >>>
//  <h> ��������
//  <i> ���õ���ģʽ���ο���ѹ��ɨ�贰�ڿ�ȵ�
//  <o>	CMOD����
//  <i> ����CMOD��ӻ�����
//  <0=> �ڲ�
//  <1=> �ⲿ
#define EXTCMOD						0	
//  <o>	�ο���ѹ
//  <i> ���ô����Ƚ����ο���ѹ
//  <0=> 0.6V
//  <1=> 1.0V
//  <2=> 1.5V
//  <3=> 2.0V
#define CONFIG_VREF					3			
//  <o> ɨ�贰��
//  <i> ��������ʱ�� = win * (1/Ftk_clk)����Ftk_clk = 24M, win=12000, ����������ʱ��Ϊ500us��
//  <0-65535>
#define CONFIG_WIN					12000		
//  <e>	����Ӧ����
//  <i> �Զ����õ���
#define CONFIG_IDAC0				1	
//  <o> �����Ⱦ���
//  <i> ʹ��ͨ������������һ��
//  <0=> DISABLE
//  <1=> ENABLE
#define CONFIG_IDAC1				1	
// </e>
//  </h>	

//  <h> ɨ��ģʽ
//  <i> ����SWģʽ�Լ�Ƶ��
//  <o>��Ƶϵ��
//  <i>PWM:SWCLK = APB_CLK / SW_DIV������Ϊ6����SWCLK = 4M
//  <i>PRS:SWCLK = APB_CLK / (SW_DIV * 2)������Ϊ8����SWCLK = 1.5M
//  <1-511>
#define SW_DIV					6	
//  <e>	��Ƶģʽ
//  <i> ���ڿ�����Ƶ
#define SW_MODE					1	
//  <o>��Ƶģʽ����ʽ���
//  <i>��Χ��1 ~ 15
//  <1-15>
#define PWMM_PRS_LFSRW		8					
//  <o>���������
//  <i>�����λ����Ϊ1��
//  <1-255>
#define PWMM_PRS_SEED		0XA7		
//  </e>	
//  </h>	

//  <e>	�������
//  <i> δɨ��������� 
#define COPA_ENABLE				1	
//  <o>	���������ѹ
//  <i> ���ò�����ѹ
//  <0=> 1.6V
//  <1=> 2.0V
//  <2=> 2.4V
//  <3=> 2.8V
//  <4=> 3.2V
//  <5=> 3.6V
//  <6=> 4.0V
//  <7=> 4.4V
#define CONFIG_VCC1				1																				
//  <o>	�����������
//  <i> �������״̬
//  <0=> �͵�ƽ
//  <1=> �ߵ�ƽ
//  <2=> ����
//  <3=> ͬ��
#define COPADATA_OUTPUT			0	
//  </e>	

//  <e>	����ʡ��
//  <i> �������ô���ʡ��ģʽ���޲���һ��ʱ����������ģʽ
#define SLEEP_EN		   	0	
//  <o>�޲�����������ʱ��
//  <i>���봥��ʡ��ģʽ����ʱʱ�䣬��λΪ5ms������Ϊ200��Ϊ1�롣
//  <0-65535>
#define ENTER_SLEEPTIME		200  	
//  <o>ɨ�败���ļ��ʱ��
//  <i>��λΪ1ms�����ֵΪ1000������Ϊ100��Ϊ100ms��ע����ʱ��Խ����������������Խ�ͣ�����������Ӧ�ٶ�Խ����
//  <0-1000>
#define LP_WAUP_TIME		100		
//  <o>����ʡ��ģʽ������ֵ
//  <i>���������ݱ仯�����ڴ���ֵ���˳�ʡ��ģʽ�����ù������ɴ������ѻ��ѣ����ù�С����ɴ������ѣ����߹���Ҳ���Ӧ���ӣ�
//  <0-65535>
#define LP_NORMRMALDELTA	15		
//  <o>����ģʽ����ֵ
//  <i>���õ͹��Ĵ���ɨ�贰��ʱ�䣬ʱ��Խ�̣�����Խ��
//  <0-65535>
#define CONFIG_LP_WIN		6000				
//  <o>	�ο���ѹ
//  <i> ��������״̬�´����Ƚ����Ĳο���ѹ����ѹԽ�ͣ�����Խ��
//  <0=> 0.6V
//  <1=> 1.0V
//  <2=> 1.5V
//  <3=> 2.0V
#define CONFIG_LPVREF		0																												
//  </e>	

//  <h>	��������
//  <i> ���ô�������
//  <o>NOISE����ֵ
//  <i>������ֵ��С
//  <0-100>
#define NOISE_THD_DEFAULT	15								//
//  <o>���������˲�����
//  <i>��Χ��1~30������ֵԽ������Խƽ�������ǰ�����Ӧ�ٶ�Խ��
//  <1-30>
#define FILTERCOUNT			5								//	
//  <o>���ư�������ʱ��
//  <i>��λΪms���Ϊ2���ӣ�������Ϊ0����رմ˹���	
//  <0-120000>
#define KEY_OUT_MAX_TIME					30000

#define KEY_OUT_MAX_TICK_CNT				(KEY_OUT_MAX_TIME / 5)	
//  <o>����������������ʱ��
//  <i>��λΪ5ms
//  <0-100>
#define FINGER_CONFIRM_TICK_CNT					(5) 		
//  <o>���������ͷ�����ʱ��
//  <i>��λΪ5ms
//  <0-100>
#define FINGER_RELEASE_CONFIRM_TICK_CNT			(5)  		
//  <o>WATER AREA���»���ʱ��
//  <i>��λΪ5ms
//  <0-100>
#define WATER_AREA_CONFIRM_TICK_CNT				(150)		//��λΪ5ms
//  <o>NOISE AREA���»���ʱ��
//  <i>��λΪ5ms
//  <0-100>
#define NOISE_AREA_CONFIRM_TICK_CNT				(100)		//��λΪ5ms
//  <o>BOTTON AREA���»���ʱ��
//  <i>��λΪ5ms
//  <0-100>
#define BOTTON_AREA_CONFIRM_TICK_CNT			(60)		//��λΪ5ms
//  <o>	��������
//  <i> ���õ�������
//  <0=> DISABLE
//  <1=> ENABLE
#define SIGLE_KEY_TRIGGER			0		
//  <o>	���öఴ��������
//  <i> ���ఴ��ͬʱ����ʱǿ�Ƹ���BASELINE,����Ϊ0�����ܹر�
//  <0-255>
#define MAX_TRIGGER_KEY_CNT 		0				
// </h>

//  <e>	�ڲ��ο�ͨ��
//  <i> �����ڲ��ο�ͨ��
#define REF_CH_EN				1			
//  <o> ����ֵ
//  <i> �����ڲ��ο�ͨ������ֵ
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
#define EXTERNAL_CHS		INSIDE_CHS		/*	�ⲿ�ο�ͨ�� */
#define EXTERNAL_CNT		1
#else
#define EXTERNAL_CNT		0
#endif

#define SLIDER_OR_WHEEL_CH_TOTAL	(SLIDER_OR_WHEEL0_CH_CNT + SLIDER_OR_WHEEL1_CH_CNT + SLIDER_OR_WHEEL2_CH_CNT + SLIDER_OR_WHEEL3_CH_CNT)

#define TK_CHMAX  					(KEY_CH_TOTAL + EXTERNAL_CNT + SHIELD_CH_TOTAL + SLIDER_OR_WHEEL_CH_TOTAL)

#endif

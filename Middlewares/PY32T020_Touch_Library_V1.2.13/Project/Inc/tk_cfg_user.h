#ifndef _TK_CFG_USER_H
#define _TK_CFG_USER_H

//按键触摸通道定义，按KEY的顺序填写，未用到的按键必须定义为TK_CH_NONE
#define CH_KEY0			TK_CH17
#define CH_KEY1			TK_CH15
#define CH_KEY2			TK_CH11
#define CH_KEY3			TK_CH5
#define CH_KEY4			TK_CH16
#define CH_KEY5			TK_CH10
#define CH_KEY6			TK_CH7
#define CH_KEY7			TK_CH6
#define CH_KEY8			TK_CH_NONE
#define CH_KEY9			TK_CH_NONE
#define CH_KEY10		TK_CH_NONE
#define CH_KEY11		TK_CH_NONE
#define CH_KEY12		TK_CH_NONE
#define CH_KEY13		TK_CH_NONE
#define CH_KEY14		TK_CH_NONE
#define CH_KEY15		TK_CH_NONE
#define CH_KEY16		TK_CH_NONE
#define CH_KEY17		TK_CH_NONE
#define CH_KEY18		TK_CH_NONE
#define CH_KEY19		TK_CH_NONE
#define CH_KEY20		TK_CH_NONE
#define CH_KEY21		TK_CH_NONE
#define CH_KEY22		TK_CH_NONE
#define CH_KEY23		TK_CH_NONE
#define CH_KEY24		TK_CH_NONE
#define CH_KEY25		TK_CH_NONE
//--------------------------------------------------------------


//--------------------------------------------------------------
//按键触发门限值定义
#define KEY0_THD        100
#define KEY1_THD        100
#define KEY2_THD        100
#define KEY3_THD        100
#define KEY4_THD        100
#define KEY5_THD        100
#define KEY6_THD        100
#define KEY7_THD        100
#define KEY8_THD		40
#define KEY9_THD		40
#define KEY10_THD		40
#define KEY11_THD		40
#define KEY12_THD		40
#define KEY13_THD		40
#define KEY14_THD		40
#define KEY15_THD		40
#define KEY16_THD		40
#define KEY17_THD		40
#define KEY18_THD		40
#define KEY19_THD		40
#define KEY20_THD		40
#define KEY21_THD		40
#define KEY22_THD		40
#define KEY23_THD		40
#define KEY24_THD		40
#define KEY25_THD		40
//==============================================================
#define WATER_PROOF_EN			0			/*	1: 防水功能使能	 0：防水功能关闭   */
#define WATER_SHIELD_CH			TK_CH_NONE	/*	Shield通道号  */
#define WATER_PROOF_MODE		0			/*	0：有水状态下按键正常触发	1：有水状态下屏蔽按键 */
#define WATER_RATIO_0			15			/*	有水按键判断门限值	*/
#define WATER_RATIO_1			45			/*	水流判断门限值		*/
#define DUSTERCLOTH_EN			0			/*	1：防湿抹布功能开启		0：防湿抹布功能关闭*/
#define DUSTERCLOTH_THD			250			/*	湿抹布判断门限值	*/

#define HIGH_SENSITVITY_EN		0			/*	1：高灵敏度模式开启	0：高灵敏度模式关闭 */
#define SENSITVITY_SHIELD_CH	TK_CH_NONE		/* Shield通道号	*/

//==============================================================
#define SLIDER_OR_WHEEL0_TYPE			TK_APP_NONE			//滑条类型
#define SLIDER_OR_WHEEL0_RESOLUTION		100					//滑条分辨率
#define SLIDER_OR_WHEEL0_THD			80					//滑条门限值
#define SLIDER_OR_WHEEL0_CH0			TK_CH4				//滑条通道，按顺序填写
#define SLIDER_OR_WHEEL0_CH1			TK_CH8				
#define SLIDER_OR_WHEEL0_CH2			TK_CH5
#define SLIDER_OR_WHEEL0_CH3			TK_CH6
#define SLIDER_OR_WHEEL0_CH4			TK_CH7
#define SLIDER_OR_WHEEL0_CH5			TK_CH_NONE
#define SLIDER_OR_WHEEL0_CH6			TK_CH_NONE
#define SLIDER_OR_WHEEL0_CH7			TK_CH_NONE

#define SLIDER_OR_WHEEL1_TYPE			TK_APP_NONE
#define SLIDER_OR_WHEEL1_RESOLUTION		255
#define SLIDER_OR_WHEEL1_THD			450
#define SLIDER_OR_WHEEL1_CH0			TK_CH19
#define SLIDER_OR_WHEEL1_CH1			TK_CH23
#define SLIDER_OR_WHEEL1_CH2			TK_CH24
#define SLIDER_OR_WHEEL1_CH3			TK_CH25
#define SLIDER_OR_WHEEL1_CH4			TK_CH_NONE
#define SLIDER_OR_WHEEL1_CH5			TK_CH_NONE
#define SLIDER_OR_WHEEL1_CH6			TK_CH_NONE
#define SLIDER_OR_WHEEL1_CH7			TK_CH_NONE

#define SLIDER_OR_WHEEL2_TYPE			TK_APP_NONE
#define SLIDER_OR_WHEEL2_RESOLUTION		255
#define SLIDER_OR_WHEEL2_THD			80
#define SLIDER_OR_WHEEL2_CH0			TK_CH_NONE
#define SLIDER_OR_WHEEL2_CH1			TK_CH_NONE
#define SLIDER_OR_WHEEL2_CH2			TK_CH_NONE
#define SLIDER_OR_WHEEL2_CH3			TK_CH_NONE
#define SLIDER_OR_WHEEL2_CH4			TK_CH_NONE
#define SLIDER_OR_WHEEL2_CH5			TK_CH_NONE
#define SLIDER_OR_WHEEL2_CH6			TK_CH_NONE
#define SLIDER_OR_WHEEL2_CH7			TK_CH_NONE

#define SLIDER_OR_WHEEL3_TYPE			TK_APP_NONE
#define SLIDER_OR_WHEEL3_RESOLUTION		255
#define SLIDER_OR_WHEEL3_THD			80
#define SLIDER_OR_WHEEL3_CH0			TK_CH_NONE
#define SLIDER_OR_WHEEL3_CH1			TK_CH_NONE
#define SLIDER_OR_WHEEL3_CH2			TK_CH_NONE
#define SLIDER_OR_WHEEL3_CH3			TK_CH_NONE
#define SLIDER_OR_WHEEL3_CH4			TK_CH_NONE
#define SLIDER_OR_WHEEL3_CH5			TK_CH_NONE
#define SLIDER_OR_WHEEL3_CH6			TK_CH_NONE
#define SLIDER_OR_WHEEL3_CH7			TK_CH_NONE
//==============================================================
#endif
//--------------------------------------------------------------

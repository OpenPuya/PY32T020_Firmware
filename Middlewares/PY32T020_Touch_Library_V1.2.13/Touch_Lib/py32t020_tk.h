#ifndef PY32T020_TK_H
#define PY32T020_TK_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "py32t0xx.h"
#include <stdio.h>

#define TK_STDIV2 	(0x00U)
#define TK_STDIV4 	(0x01U)
#define TK_STDIV8 	(0x02U)
#define TK_STDIV12 	(0x03U)
#define TK_STDIV16 	(0x04U)
#define TK_STDIV24 	(0x05U)

#define TK_DIV0 	(0x00U)
#define TK_DIV2 	(0x02U)
#define TK_DIV3 	(0x03U)
#define TK_DIV4 	(0x04U)
#define TK_DIV5 	(0x05U)
#define TK_DIV6 	(0x06U)
#define TK_DIV7 	(0x07U)

#define TK_CFT0 	(0x00U)
#define TK_CFT1 	(0x01U)
#define TK_CFT2 	(0x02U)
#define TK_CFT3 	(0x03U)
#define TK_CFT4 	(0x04U)
#define TK_CFT5 	(0x05U)
#define TK_CFT6 	(0x06U)
#define TK_CFT7 	(0x07U)

#define TK_MIDACSTEP_0_5UA 	(0x00U)
#define TK_MIDACSTEP_2UA 	(0x01U)

#define TK_VCC1CR_1_6V (0x00U)
#define TK_VCC1CR_2_0V (0x01U)
#define TK_VCC1CR_2_4V (0x02U)
#define TK_VCC1CR_2_8V (0x03U)
#define TK_VCC1CR_3_2V (0x04U)
#define TK_VCC1CR_3_6V (0x05U)
#define TK_VCC1CR_4_0V (0x06U)
#define TK_VCC1CR_4_4V (0x07U)

#define TK_COPADATA_OUTPUT_LOW 			(0x00U)
#define TK_COPADATA_OUTPUT_HIGH 		(0x01U)
#define TK_COPADATA_OUTPUT_S1_INVERSE 	(0x02U)
#define TK_COPADATA_OUTPUT_S1 			(0x03U)

#define TK_KVREF_0_6V (0x00U)
#define TK_KVREF_1_0V (0x01U)
#define TK_KVREF_1_5V (0x02U)
#define TK_KVREF_2_0V (0x03U)

#define TK_SHORTSW_NUM_0 (0x00U)
#define TK_SHORTSW_NUM_1 (0x01U)
#define TK_SHORTSW_NUM_2 (0x02U)
#define TK_SHORTSW_NUM_3 (0x03U)

#define TK_LPABNORMAL_MODE0 (0x00U)
#define TK_LPABNORMAL_MODE1 (0x01U)

#define TK_CTRLSOURCE_SW (0x00U)
#define TK_CTRLSOURCE_HW (0x01U)

#define TK_PWMM_PWM (0x00U)
#define TK_PWMM_PRS (0x01U)


#define PRS_DOUBLEHOP 		(1) // 1开启双跳频	0单跳频

#if PRS_DOUBLEHOP
#define PWMM_PRS_LFSRW_2 	15
#endif

#define MAX_CHS				26
#define SLIDER_MAX 			4 // 支持的滑条数量
#define SHIELD_CHS			0XF0
#define INSIDE_CHS			0XF1

#define TK_SLEEP			1
typedef enum
{
	TK_OK = 0x00U,
	TK_ERROR = 0x01U,
	TK_BUSY = 0x02U,
	TK_TIMEOUT = 0x03U
} TK_Status;
/**
 * @brief LL TK Init Structure definition
 */
#pragma pack(4)
typedef struct
{
	uint8_t Div;          // TK_CLK分频系数
	uint8_t Stdiv;        // ST_CLK分频系数
	uint8_t PwmM;         // PWM模式
	uint8_t Lfsrw;        // 线性反馈移位寄存器位宽
	uint8_t ExtCmod;      // 外部CMODE使能
	uint8_t CmphysEnable; // 比较迟滞器使能
	uint8_t Cft;          // 比较器输出滤波时间
	uint8_t Vcc1Cr;       // VCC1电压设置
	uint8_t CopaData;     // 未扫描输出控制
	uint8_t CopaMode;     // 按键补偿使能
	uint8_t TrimIdac[MAX_CHS]; // 自适应电流值
	uint8_t TrimState;    // 自适应状态
	uint8_t ShortswNum;   // shortswitch次数
	uint8_t MidacStep;    // Midac电流步进
	uint8_t Midac;        // MiDac电流
	uint8_t Vref;         // 参考电压
	uint8_t TrimEnable;   // 自适应电流使能
	uint8_t TrimRatio[2]; // 校准窗口范围
	uint16_t Swdiv;       // SW分频系数
	uint16_t Win;         // 窗口时间
	uint16_t SetTime;     // shortswitch时间
	uint8_t prsseed;     // 
	uint32_t CopaChs;     // IO补偿使能
	uint32_t TrimStep;    // 自适应电流档位
	uint32_t Kenable;     // 通道使能
#if (LIB_TYPE > 1)
	uint8_t CopaOutLowState;
	uint8_t CopaOutLowTrimIdac[MAX_CHS];
	uint32_t CopaOutLowTrimStep;
#endif
} TK_InitTypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
	uint8_t res0;
	uint8_t Lpwait;  // 唤醒等待时间
	uint8_t LpDac;           // 休眠电流
	uint8_t WakeDelay;       // 唤醒延时，唤醒后第一次采集数据不准确，所以丢掉
	uint8_t RtcInt;          // 休眠状态使能的RTC中断类型,休眠后会更改RTC秒中断时间，RTC时间会更改
	uint8_t Vref;            // 参考电压设置
	uint16_t EnterStopTimer; // 无操作进入休眠时间
	uint16_t SleepTouchThd;  // 唤醒差值
	uint16_t LpWakeTime;
	uint16_t Baseline;       // 硬件比较器的值
	uint16_t NormalDelta;    // 唤醒差值
	uint16_t LpWin;          // 休眠窗口值
	uint16_t LpCdr;          // 唤醒值数据
	uint32_t WakeChs;        // 唤醒通道
	uint32_t LpKchs; 			// 唤醒通道
	uint32_t AsynchPrediv;   // RTC正常模式分频值
	uint16_t Alarm_tick;
} LP_InitTypeDef;
#pragma pack()

#pragma pack(4)
typedef struct __TK_HandleTypeDef
{
	TK_InitTypeDef Init;
#if TK_SLEEP
	LP_InitTypeDef Lp;
	void (*EnterStopCallback)(void); /*!< 进入深度休眠回调函数    */
	void (*ExitStopCallback)(void);  /*!< 退出深度休眠回调函数    */
#endif
	uint8_t (*TouchKeyFlagsMask)(void); /*!< 按键处理回调函数   		 */
#if (LIB_TYPE > 1)
	uint8_t (*TouchWaterFlagsMask)(uint8_t chs, int16_t Differ, int16_t DifferSigle); /*!< 防水处理回调函数 */
	uint8_t (*TouchShieldFlagsMask)(uint8_t chs,uint16_t BaseLineData,uint16_t AcqData); /*!< 按键处理回调函数 */
#endif
} TK_HandleTypeDef;
#pragma pack()

TK_Status TK_SetSenseStart(uint32_t Chs); // 开始执行触摸扫描
TK_Status TK_SetLfsrw(uint8_t Lfsrw);     // PRS模式设置移位寄存器位宽
TK_Status TK_LibInit(void);               // 触摸库初始化
TK_Status TK_LibDeInit(void);             // 触摸库
void TK_LibIRQHandler(void);
void Timer_RTC_Init(uint16_t ms,uint16_t Alarm,uint32_t RtcInt);
extern volatile TK_HandleTypeDef TK_Handle;
#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

#endif

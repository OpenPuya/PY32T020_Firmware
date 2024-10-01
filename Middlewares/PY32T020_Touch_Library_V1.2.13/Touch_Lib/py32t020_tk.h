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


#define PRS_DOUBLEHOP 		(1) // 1����˫��Ƶ	0����Ƶ

#if PRS_DOUBLEHOP
#define PWMM_PRS_LFSRW_2 	15
#endif

#define MAX_CHS				26
#define SLIDER_MAX 			4 // ֧�ֵĻ�������
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
	uint8_t Div;          // TK_CLK��Ƶϵ��
	uint8_t Stdiv;        // ST_CLK��Ƶϵ��
	uint8_t PwmM;         // PWMģʽ
	uint8_t Lfsrw;        // ���Է�����λ�Ĵ���λ��
	uint8_t ExtCmod;      // �ⲿCMODEʹ��
	uint8_t CmphysEnable; // �Ƚϳ�����ʹ��
	uint8_t Cft;          // �Ƚ�������˲�ʱ��
	uint8_t Vcc1Cr;       // VCC1��ѹ����
	uint8_t CopaData;     // δɨ���������
	uint8_t CopaMode;     // ��������ʹ��
	uint8_t TrimIdac[MAX_CHS]; // ����Ӧ����ֵ
	uint8_t TrimState;    // ����Ӧ״̬
	uint8_t ShortswNum;   // shortswitch����
	uint8_t MidacStep;    // Midac��������
	uint8_t Midac;        // MiDac����
	uint8_t Vref;         // �ο���ѹ
	uint8_t TrimEnable;   // ����Ӧ����ʹ��
	uint8_t TrimRatio[2]; // У׼���ڷ�Χ
	uint16_t Swdiv;       // SW��Ƶϵ��
	uint16_t Win;         // ����ʱ��
	uint16_t SetTime;     // shortswitchʱ��
	uint8_t prsseed;     // 
	uint32_t CopaChs;     // IO����ʹ��
	uint32_t TrimStep;    // ����Ӧ������λ
	uint32_t Kenable;     // ͨ��ʹ��
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
	uint8_t Lpwait;  // ���ѵȴ�ʱ��
	uint8_t LpDac;           // ���ߵ���
	uint8_t WakeDelay;       // ������ʱ�����Ѻ��һ�βɼ����ݲ�׼ȷ�����Զ���
	uint8_t RtcInt;          // ����״̬ʹ�ܵ�RTC�ж�����,���ߺ�����RTC���ж�ʱ�䣬RTCʱ������
	uint8_t Vref;            // �ο���ѹ����
	uint16_t EnterStopTimer; // �޲�����������ʱ��
	uint16_t SleepTouchThd;  // ���Ѳ�ֵ
	uint16_t LpWakeTime;
	uint16_t Baseline;       // Ӳ���Ƚ�����ֵ
	uint16_t NormalDelta;    // ���Ѳ�ֵ
	uint16_t LpWin;          // ���ߴ���ֵ
	uint16_t LpCdr;          // ����ֵ����
	uint32_t WakeChs;        // ����ͨ��
	uint32_t LpKchs; 			// ����ͨ��
	uint32_t AsynchPrediv;   // RTC����ģʽ��Ƶֵ
	uint16_t Alarm_tick;
} LP_InitTypeDef;
#pragma pack()

#pragma pack(4)
typedef struct __TK_HandleTypeDef
{
	TK_InitTypeDef Init;
#if TK_SLEEP
	LP_InitTypeDef Lp;
	void (*EnterStopCallback)(void); /*!< ����������߻ص�����    */
	void (*ExitStopCallback)(void);  /*!< �˳�������߻ص�����    */
#endif
	uint8_t (*TouchKeyFlagsMask)(void); /*!< ��������ص�����   		 */
#if (LIB_TYPE > 1)
	uint8_t (*TouchWaterFlagsMask)(uint8_t chs, int16_t Differ, int16_t DifferSigle); /*!< ��ˮ����ص����� */
	uint8_t (*TouchShieldFlagsMask)(uint8_t chs,uint16_t BaseLineData,uint16_t AcqData); /*!< ��������ص����� */
#endif
} TK_HandleTypeDef;
#pragma pack()

TK_Status TK_SetSenseStart(uint32_t Chs); // ��ʼִ�д���ɨ��
TK_Status TK_SetLfsrw(uint8_t Lfsrw);     // PRSģʽ������λ�Ĵ���λ��
TK_Status TK_LibInit(void);               // �������ʼ��
TK_Status TK_LibDeInit(void);             // ������
void TK_LibIRQHandler(void);
void Timer_RTC_Init(uint16_t ms,uint16_t Alarm,uint32_t RtcInt);
extern volatile TK_HandleTypeDef TK_Handle;
#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

#endif

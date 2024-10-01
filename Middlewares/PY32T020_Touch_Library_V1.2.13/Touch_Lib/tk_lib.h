#ifndef _TS_API_H
#define _TS_API_H

#include "py32t020_tk.h"

#define SIGLE_KEY			0X01U
#define BASELINE_STOP		0X02U
//#define AUTO_THD			0X04U
#define REF_CONFIG0			0X08U

#define TK_APP_NONE 		0x00U
#define TK_APP_TOUCH_KEY 	0x01U
#define TK_APP_SLIDER 		0x02U
#define TK_APP_WHEEL 		0x04U
#define TK_APP_PANAL 		0x08U

#define TK_CH0 		0x00U
#define TK_CH1 		0x01U
#define TK_CH2 		0x02U
#define TK_CH3 		0x03U
#define TK_CH4 		0x04U
#define TK_CH5 		0x05U
#define TK_CH6 		0x06U
#define TK_CH7 		0x07U
#define TK_CH8 		0x08U
#define TK_CH9 		0x09U
#define TK_CH10 	0x0AU
#define TK_CH11 	0x0BU
#define TK_CH12 	0x0CU
#define TK_CH13 	0x0DU
#define TK_CH14 	0x0EU
#define TK_CH15 	0x0FU
#define TK_CH16 	0x10U
#define TK_CH17 	0x11U
#define TK_CH18 	0x12U
#define TK_CH19 	0x13U
#define TK_CH20 	0x14U
#define TK_CH21 	0x15U
#define TK_CH22 	0x16U
#define TK_CH23 	0x17U
#define TK_CH24 	0x18U
#define TK_CH25 	0x19U
#define TK_CH_NONE 	0xFFU

typedef enum
{
    TK_INIT,
    TK_SET_BASELINE,
    TK_GO,
    TK_ENTERSLEEP,
    TK_EXITSLEEP,
} TSState_TypeDef;

typedef enum
{
    STATE_IDLE,
    STATE_SINGLE_CH_ACQING,
    STATE_SINGLE_CH_ACQ_DONE,
    STATE_MULTI_CH_ACQING,
    STATE_MULTI_CH_ACQ_DONE,
} TK_RawAcqState_TypeDef;

typedef enum
{
    TK_AREA_TOP,
    TK_AREA_FINGER,
    TK_AREA_WATER,
    TK_AREA_PNOISE,
    TK_AREA_NNOISE,
    TK_AREA_BOTTOM,
} TKArea_TypeDef;

#pragma pack(4)
typedef struct
{
    TSState_TypeDef TK_state;				// ����״̬
    uint8_t TouchKeyChCnt; 					// ��������	
    uint8_t SleepEn;						// ����ʹ��
    uint8_t KeyOutTimeLimitEn;				// ������ʱʹ��
    uint16_t SleepTime;						// ���߼���ֵ
    uint32_t KeyFlags;						// �û�������־
    uint32_t KeyFlagsPending;				// �����ⰴ����־
    uint32_t TK_KeyEnable;					// ʹ�ܴ���ͨ����
    uint16_t KeyOutTimeLimitCNT;			// ������ʱʱ��
	uint8_t TK_Config;						// ��������
	uint8_t MultiKeyCnt;					// ��λ������
	uint8_t Lib_function;					//��������
#if (LIB_TYPE > 0)
	int16_t SliderOrWheelPosition[SLIDER_MAX]; // ����λ��
#endif
} TKMainCtr_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
    uint8_t TouchAcqChCnt;  				// ɨ��ͨ����������������ˮ�Լ�����
    uint8_t *TouchKeyChSeq; 				// ͨ����
} TouchKeyChInfor_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
	#if (LIB_TYPE > 0)
	uint8_t WheelSliderState;				//����״̬0
    uint8_t WheelSliderTouch;				//����״̬1
    uint8_t SliderWheelxStartChIdx[SLIDER_MAX];      //������������
    uint8_t SliderWheelxChCnt[SLIDER_MAX];  //����ͨ������
    const uint8_t *SliderWheelType;			//��������
    const uint16_t *SliderWheelResloution;	//�����ֱ���
	#else
	const uint8_t *SliderWheelType;
	#endif
} SliderChInfor_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
    TKArea_TypeDef *TK_Area;
    TKArea_TypeDef *TK_AreaPending;
    uint16_t FingerAreaConfirmTickCNT;
    uint16_t WaterAreaConfirmTickCNT;
    uint16_t NoiseAreaConfirmTickCNT;
    uint16_t BottonAreaConfirmTickCNT;
    uint16_t FingerReleaseConfirmTickCNT;
} TkAreaInfor_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
    uint16_t *TK_AreaConfirmTickCnt;
    uint16_t *TK_KeyOutTimeLimitCnt;
#if (LIB_TYPE > 1)
    uint8_t TK_WaterStateConfirmCnt;
#endif
} TK_TickCouter_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
    uint16_t *RawData;
	uint16_t *FilteredData;
    uint16_t *BaseLineData;
    int16_t *DifferLev;
    int16_t *Differ;
    int16_t *FingerTHD;
	uint16_t *FilteredDataDBG;
    uint16_t *BaseLineDataDBG;  	
    uint8_t *FilteredCount;
    uint32_t *FilteredSum;
#if (LIB_TYPE > 1)
    uint16_t *SingleBaseLineData;
    uint16_t *SingleAcqData;
#endif
} TouchKeyDatas_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
    uint8_t *BufIdx;
	uint16_t *RawDatasBuf;
#if (LIB_TYPE > 1)
    uint16_t *SingleAcqRawDatasBuf;
#endif
} TK_DataBuf_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
    uint8_t AvgModCNT;             // �˲�����
	uint8_t ExternalStartIdx;
	int16_t NoiseTHD;
} FilterCtr_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
	uint8_t ShieldStartChIdx;    //����ͨ������
	#if (LIB_TYPE > 1)
	uint8_t Noise;				//����ͨ���ȶ�
	int16_t DusterClothDel;		//����ͨ���仯��
	int16_t trigger_ratio[2];	//����ͨ������
	
	uint8_t WaterProof_En;
	uint8_t WaterProof_Chs;			//Shieldͨ����
	int16_t WaterProof_Ratio0;		//ˮ���ж�����ֵ
	int16_t WaterProof_Ratio1;		//��ˮ�ж�����ֵ
	uint8_t WaterProof_Mode;		//1 ��ˮ״̬���ΰ���
	
	uint8_t DusterCloth_En;			//1 ��ʪĨ�����ܿ���
	uint16_t DusterCloth_THD;		//ʪĨ���ж�����ֵ
	
	uint8_t HighSensitvity_En;
	uint8_t HighSensitvity_Chs;
	#endif
} ShieldInfor_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct logQueue
{
    uint8_t *base;
    uint8_t front, rear;
    uint8_t sq_size;
} logQueue;
#pragma pack()

#pragma pack(4)
typedef struct
{
    TkAreaInfor_TypeDef *pTkAreaInfor;
    TouchKeyChInfor_TypeDef *pTouchKeyChInfor;
    TouchKeyDatas_TypeDef *pTouchKeyDatas;
    TKMainCtr_TypeDef *pTKCtr;
    TK_TickCouter_TypeDef *pTK_TickCouter;
    TK_DataBuf_TypeDef *pTK_DataBuf;
    uint16_t *pTkAcqAbandonCNT;
    SliderChInfor_TypeDef *pSliderChInfor;
    uint8_t *pDtRdy;
    uint32_t *pVersion;
	ShieldInfor_TypeDef *pShieldInfor;
	logQueue *plogQueue;
	uint32_t *pChip;
} TK_Info_T;
#pragma pack()



extern __IO TK_Info_T TK_Info;
extern TKMainCtr_TypeDef TKCtr;
extern FilterCtr_TypeDef FilterCtr; 
extern ShieldInfor_TypeDef ShieldInfor;
void TK_Init(void);
void TK_MainFsm(void);
void TK_TimerHandler(uint8_t ms);
void TK_UpdatBaseLineData(uint8_t chs);
uint32_t TK_GetVersion(void); 

void log_putstr(char *str);
#endif

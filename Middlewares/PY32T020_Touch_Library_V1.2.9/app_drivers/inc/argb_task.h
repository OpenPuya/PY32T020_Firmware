#ifndef _ARGB_TASK_H_
#define _ARGB_TASK_H_

#include "app_config.h"
#if APP_SPI_LED_ENABLE
typedef struct LedSetData
{
	unsigned char  red;
	unsigned char  green;	
	unsigned char  blue;
}LedSetData;
#pragma anon_unions
/* Exported variables prototypes ---------------------------------------------*/
typedef	struct LedDisplayData
{
	LedSetData	color;				//顶色
	union 
	{
		unsigned char flag;
		struct 
		{
			unsigned char  dir 				:2;
			unsigned char  colorCount		:5;
			unsigned char  reserved 		:1;
		}flag_b;
	};								//顶层方向数据
	uint8_t alpha;			//透明度
	uint8_t hold;
	uint8_t dim;
}ARGBDisplay;
void argb_init(void);
void argb_color_inc(void);
void argb_color_dec(void);
void argb_serive(uint8_t whlelhold,uint8_t sliderhold);
void argb_config(uint8_t offset,uint8_t alpha,uint8_t dim,uint8_t dir,uint8_t hold);
extern uint16_t argb_time;
extern ARGBDisplay argb[SPI_LED_CNT];
#endif
#endif

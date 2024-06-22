#ifndef _ADC_DRIVERS_H_
#define _ADC_DRIVERS_H_

#include "app_config.h"

#if APP_ADC_ENABLE
#define ADC_VREFBUF_VCCA      (0x00000000U)                                               /*!< VREFBUF VCCA   */
#define ADC_VREFBUF_0P6V      ((uint32_t)(ADC_CCR_VREFBUF_EN))                            /*!< VREFBUF 0.6V   */
#define ADC_VREFBUF_1P5V      ((uint32_t)(ADC_CCR_VREFBUF_EN | ADC_CCR_VREFBUF_SEL_0))    /*!< VREFBUF 1.5V   */
#define ADC_VREFBUF_2P048V    ((uint32_t)(ADC_CCR_VREFBUF_EN | ADC_CCR_VREFBUF_SEL_1))    /*!< VREFBUF 2.048V */
#define ADC_VREFBUF_2P5V      ((uint32_t)(ADC_CCR_VREFBUF_EN | ADC_CCR_VREFBUF_SEL  ))    /*!< VREFBUF 2.5V   */

#define ADC_CHANNEL_0           (0x00000000U)
#define ADC_CHANNEL_1           (0x00000001U)
#define ADC_CHANNEL_2           (0x00000002U)
#define ADC_CHANNEL_3           (0x00000003U)
#define ADC_CHANNEL_4           (0x00000004U)
#define ADC_CHANNEL_5           (0x00000005U)
#define ADC_CHANNEL_6           (0x00000006U)
#define ADC_CHANNEL_7           (0x00000007U)
#define ADC_CHANNEL_8           (0x00000008U)
#define ADC_CHANNEL_9           (0x00000009U)
#define ADC_CHANNEL_10          (0x0000000AU)
#define ADC_CHANNEL_11          (0x0000000BU)
#define ADC_CHANNEL_12          (0x0000000CU)
#define ADC_CHANNEL_TEMPSENSOR  ADC_CHANNEL_10
#define ADC_CHANNEL_VREFINT     ADC_CHANNEL_11
#define ADC_CHANNEL_1_3VCCA     ADC_CHANNEL_12
/* ADC Temperature Scale Value */
#define ADC_TSCAL1               (*(uint32_t *)(0x1fff0114))  /*!< Temperature Scale1 */
#define ADC_TSCAL2               (*(uint32_t *)(0x1fff0118))  /*!< Temperature Scale2 */

extern uint8_t adc_state;       // ADC扫描状态
extern uint16_t adc_seq;        // ADC扫描通道
extern uint16_t ADCxConvertedData[13]; // 13个通道缓存数据
/********************************************************
**	函数名	void ADC_Init(void)
**	描述	：	ADC初始化
**	传入	：	无
**	返回	：	无
*********************************************************/	
void ADC_Init(void);
/********************************************************
**	函数名	uint16_t APP_ADCConvert(uint16_t channel, uint32_t VrefBuf)
**	描述	：ADC 单次采集，主要用于参考电压不是VCC的情况
**	传入	：channel：通道号 	VrefBuf：设置的参考电压
**	返回	：采样到的12位的ADC数据
*********************************************************/	
uint16_t APP_ADCConvert(uint16_t channel, uint32_t VrefBuf);
/********************************************************
**	函数名	void ADC_GPIO_Init(void)
**	描述	初始化模拟口GPIO
**	传入	：无
**	返回	：无
*********************************************************/
void ADC_GPIO_Init(void);
#endif
#endif

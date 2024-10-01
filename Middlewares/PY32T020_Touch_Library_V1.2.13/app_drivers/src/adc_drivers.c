#include "adc_drivers.h"
#if APP_ADC_ENABLE
const uint8_t ADC_PIN[10] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1};
uint8_t adc_state;       // ADC扫描状态
uint16_t adc_seq;        // ADC扫描通道
uint16_t ADCxConvertedData[13]; // 13个通道缓存数据
#if LVD_WRITE_USER_DATA
uint8_t Lvd_Flag; // 低电压标志
#endif
/********************************************************
**	函数名	void ADC_GPIO_Init(void)
**	描述	初始化模拟口GPIO
**	传入	：无
**	返回	：无
*********************************************************/
__weak void ADC_GPIO_Init(void)
{
	
}
/********************************************************
**	函数名	void ADC_Init(void)
**	描述	：	ADC初始化
**	传入	：	无
**	返回	：	无
*********************************************************/	
void ADC_Init(void)
{
	uint16_t chs;
	uint8_t i;
	ADC_GPIO_Init();
	for (i = 0; i < 10; i++)
    {
        if (ADC_ENABLE_CHS & (1 << i))
        {
            GPIO_Init(ADC_PIN[i], ANALOG);
        }
    }
	/* Enable ADC1 clock */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
    /* Set ADC clock to pclk/8 */
    LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV8);
    /* Set ADC resolution to 12 bit */
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    /* ADC conversion data alignment: right aligned */
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    /* No ADC low power mode activated */
    LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
    /* Sampling time 239.5 ADC clock cycles */
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
    /* ADC regular group conversion trigger by software. */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    /* Set ADC conversion mode to single mode: one conversion per trigger */
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    /* ADC regular group behavior in case of overrun: data overwritten */
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    /* Disable ADC regular group sequencer discontinuous mode  */
    LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
    /* Dose not enable internal conversion channel */
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), ADC_CCR_VREFEN | ADC_CCR_TSEN);
    ADC1->CFGR1 |= ADC_CFGR1_WAIT;
    /* Configure VrefBuf VCC */
	MODIFY_REG(ADC->CCR, ADC_CCR_VREFBUF_EN | ADC_CCR_VREFBUF_SEL ,ADC_VREFBUF_VCCA);  
#if LVD_WRITE_USER_DATA
	uint16_t tmpAWDHighThresholdShifted,tmpAWDLowThresholdShifted;
	#define ADC_AWD1THRESHOLD_SHIFT_RESOLUTION(__HANDLE__, _Threshold_)            \
					((_Threshold_) << ((((__HANDLE__)->CFGR1 & ADC_CFGR1_RESSEL) >> 3U) * 2))
    /*	模拟LVD	*/
	/*    mode "all channels": ADC_CFGR_AWD1SGL=0).                           */
    ADC1->CFGR1 &= ~(ADC_CFGR1_AWDSGL |ADC_CFGR1_AWDEN  |ADC_CFGR1_AWDCH);
    ADC1->CFGR1 |= ((ADC_CFGR1_AWDSGL | ADC_CFGR1_AWDEN) | (ADC_CHANNEL_VREFINT << 26));
	tmpAWDHighThresholdShifted = ADC_AWD1THRESHOLD_SHIFT_RESOLUTION(ADC1, 1200 * 4095 / (low_voltage));
    tmpAWDLowThresholdShifted  = ADC_AWD1THRESHOLD_SHIFT_RESOLUTION(ADC1, 1200 * 4095 / (5200));
	/* Set the high and low thresholds */
    ADC1->TR &= ~(ADC_TR_HT | ADC_TR_LT);
    ADC1->TR |=  ( (tmpAWDHighThresholdShifted) << 16 | tmpAWDLowThresholdShifted);					   
    LL_ADC_ClearFlag_AWD(ADC1);
	LL_ADC_EnableIT_AWD(ADC1);
    Lvd_Flag = 0;
#endif
    /* Enable ADC calibration */
    LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
		;
    }
    /* Clear all channels */
    WRITE_REG(ADC1->CHSELR, 0);
    /* 添加需要采集的通道 */
	chs = 0;
    for (i = 0; i <= 12; i++)
    {
        if (ADC_ENABLE_CHS & (1 << i))
        {
			chs |= 1 << i;
        }
    }
	/* Set channel as conversion channel */
	LL_ADC_REG_SetSequencerChannels(ADC1, chs);
    adc_seq = ADC1->CHSELR;
	/* Enable ADC */
	LL_ADC_Enable(ADC1);
	nop_delay_xus(100);
	/* Clear eoc flag */
	LL_ADC_ClearFlag_EOC(ADC1);
	/* Clear eoc flag */
	LL_ADC_ClearFlag_EOS(ADC1);
	/* Enable EOC IT */
	LL_ADC_EnableIT_EOC(ADC1);
	/* Enable EOS IT */
	LL_ADC_EnableIT_EOS(ADC1);
	/* Enable ADC interrupt */
    NVIC_SetPriority(ADC_COMP_IRQn, 0);
    NVIC_EnableIRQ(ADC_COMP_IRQn);
    /* Start ADC conversion */
	LL_ADC_REG_StartConversion(ADC1);
	adc_state = 1;
}
/********************************************************
**	函数名	uint16_t APP_ADCConvert(uint16_t channel, uint32_t VrefBuf)
**	描述	：ADC 单次采集，主要用于参考电压不是VCC的情况
**	传入	：channel：通道号 	VrefBuf：设置的参考电压
**	返回	：采样到的12位的ADC数据
*********************************************************/	
uint16_t APP_ADCConvert(uint16_t channel, uint32_t VrefBuf)
{
    uint32_t chs = ADC1->CHSELR;
    uint16_t adcvalue;
    /*	等待序列转换完成*/
    while (LL_ADC_REG_IsConversionOngoing(ADC1) == 1)
    {
        ;
    }
	/* Disable EOC EOS IT */
	CLEAR_BIT(ADC1->IER, LL_ADC_IT_EOC|LL_ADC_IT_EOS);
	/* Clear EOC EOS flag */
	WRITE_REG(ADC1->ISR, LL_ADC_FLAG_EOC|LL_ADC_FLAG_EOS);
	/* Clear all channels */
	WRITE_REG(ADC1->CHSELR,0);
    /* Configure VrefBuf */
	MODIFY_REG(ADC->CCR, ADC_CCR_VREFBUF_EN | ADC_CCR_VREFBUF_SEL ,VrefBuf);  
    /* Set channel as conversion channel */
	LL_ADC_REG_SetSequencerChannels(ADC1, channel);
     /* Start ADC conversion */
	LL_ADC_REG_StartConversion(ADC1); 
	/* Wait ADC conversion complete */
	while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0)
	{
		;
	}
	/* Clear EOC EOS flag */
	WRITE_REG(ADC1->ISR, LL_ADC_FLAG_EOC|LL_ADC_FLAG_EOS);
    /* Get ADC Value */
    adcvalue = LL_ADC_REG_ReadConversionData12(ADC1);
    /* Disable ADC to clear channel configuration */
    LL_ADC_REG_StopConversion(ADC1);
    /* Clear all channels */
    WRITE_REG(ADC1->CHSELR, chs);
    /* Configure VrefBuf VCC */
	MODIFY_REG(ADC->CCR, ADC_CCR_VREFBUF_EN | ADC_CCR_VREFBUF_SEL ,ADC_VREFBUF_VCCA);  
    return (uint32_t)adcvalue;
}
/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
void ADC_COMP_IRQHandler(void)
{
    /* Check if the interrupt is triggered by EOC */
	if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
	{
		/* Clear ADC EOC IT flag */
		LL_ADC_ClearFlag_EOC(ADC1);
        uint16_t DR = ADC1->DR;
        /*	按照通道0-通道12依次存放，需要使能通道才会自动采集	*/
        for (uint8_t i = 0; i <= 12; i++)
        {
            if (adc_seq & (1 << i))
            {
                adc_seq &= ~(1 << i);
                ADCxConvertedData[i] = DR;
                break;
            }
        }
    }
    if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0)
	{
		/* Clear ADC EOS IT flag */
		LL_ADC_ClearFlag_EOS(ADC1);
        adc_state = 2;
    }
	#if LVD_WRITE_USER_DATA
	if(LL_ADC_IsActiveFlag_AWD(ADC1) != 0)
	{
		/* Clear ADC AWD IT flag */
		LL_ADC_ClearFlag_AWD(ADC1);
		Lvd_Flag = 0X01;
    }   
	#endif
}
#endif

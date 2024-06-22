#include "drivers.h"
#if APP_ADC_ENABLE
#define TScal1 		(float)((ADC_TSCAL1) * 3300 / Vcc_Power) /* Voltage corresponding to calibration value at 30 �� */
#define TScal2 		(float)((ADC_TSCAL2) * 3300 / Vcc_Power) /* Voltage corresponding to calibration value at 85 �� */
#define TStem1 		30                                           /* 30 �� */
#define TStem2 		85                                           /* 85 �� */
#define Temp_k 		((float)(TStem2 - TStem1) / (float)(TScal2 - TScal1)) /* Temperature calculation */	
uint16_t Vcc_Power;      // VCC��ѹ
uint16_t adc_tick;
int16_t aTEMPERATURE;
#if LVD_WRITE_USER_DATA
uint8_t power_count;
uint8_t write_otp;
#define adc_state_bit(n) ((Bits8_TypeDef *)(&(write_otp)))->bit##n
#define power_on	adc_state_bit(0)
#define power_off	adc_state_bit(1)
extern uint8_t Lvd_Flag; // �͵�ѹ��־
#endif
/********************************************************
**	������	void ADC_GPIO_Init(void)
**	����	����GPIO����Ϊģ�⹦��
**	����	����
**	����	����
*********************************************************/
void ADC_GPIO_Init(void)
{
//	GPIO_Init(PA0, ANALOG);
}
/********************************************************
**	������	void ADC_Loop(void)
**	����	ADC״̬������adc_stateΪ2ʱ��ʾ����ת����ɣ�����ֱ�Ӵ�ADCxConvertedData[x]��ȡ���ݣ�xΪͨ����
**	����	����
**	����	����
*********************************************************/
void ADC_Loop(void)
{
#if LVD_WRITE_USER_DATA
    /*	�͵�ѹ��־	*/
    if (Lvd_Flag)
    {
		Lvd_Flag = 0;
		Vcc_Power = 1200 * 4095 / ADCxConvertedData[ADC_CHANNEL_VREFINT];
        if (power_on == 1 && power_off == 0 && Vcc_Power > 2500)
        {
			uint8_t sta;
            power_off = 1;
            power_count++;
            User_Cache_Write(0, &power_count, 1);
			sta = User_Flash_Write();
			log_printf("write_otp:%d\r\n",sta);
        }
    }
#endif
    /*	ADC״̬���	*/
    switch (adc_state)
    {
		/*	����״̬		*/
		default:
		case 0:
			if (adc_tick >= ADC_SPEED)
			{
				adc_tick = 0;
				adc_seq = ADC1->CHSELR;
				if (adc_seq != 0)
				{
					/* Clear EOC EOS flag */
					WRITE_REG(ADC1->ISR, LL_ADC_FLAG_EOC|LL_ADC_FLAG_EOS);
					/* Enable EOC EOS IT */
					SET_BIT(ADC1->IER, LL_ADC_IT_EOC|LL_ADC_IT_EOS);
					/* Start ADC conversion */
					LL_ADC_REG_StartConversion(ADC1);
				}
			}
        break;
		/*	ת��״̬		*/
		case 1:
			if (adc_tick >= ADC_SPEED)
			{
				adc_tick = 0;
				/*	ת����ʱ�������������	*/
				/* Disable ADC to clear channel configuration */
				LL_ADC_REG_StopConversion(ADC1);
				adc_state = 0;
			}
        break;
		/*	ת�����״̬		*/
		case 2: 
		{
//			uint16_t adc_value;
			adc_state = 0;
			/*	�����ȼ���VCC��ѹ������Ķ�����VCCΪ�ο���ѹ�����	*/
			Vcc_Power = 1200 * 4095 / ADCxConvertedData[ADC_CHANNEL_VREFINT];
			aTEMPERATURE = (int16_t)((85 - 30) * (ADCxConvertedData[ADC_CHANNEL_TEMPSENSOR] - TScal1) / (TScal2 - TScal1) + TStem1);
			#if LVD_WRITE_USER_DATA
			if (Vcc_Power > (low_voltage + 100)) // �ָ���������
			{
				power_on = 1;
				power_off = 0;
			}
			#endif
//			log_printf("Vcc_Power = %d %d\r\n", Vcc_Power, ADCxConvertedData[ADC_CHANNEL_VREFINT]);
//			log_printf("Temperature = %d\r\n", aTEMPERATURE);
//			adc_value = Vcc_Power / 4095 * ADCxConvertedData[ADC_CHANNEL_8];
//			log_printf("pb0_value = %d\r\n", adc_value);
			/*	ת����ɺ���������������ο���ѹ����VCC��	*/
//			adc_value = APP_ADCConvert(ADC_CHANNEL_9, ADC_VREFBUF_1P5V);
//			log_printf("pb1_value = %d\r\n", adc_value);
		}
		break;
    }
}
#endif

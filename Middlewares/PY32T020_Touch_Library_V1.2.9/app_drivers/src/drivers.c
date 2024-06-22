#include "drivers.h"
volatile uint32_t EXTI_Flag;
#if (APP_TM1624_ENABLE && TM1624_DISP_TEST)
uint8_t TM1624_Show[14];
uint16_t TM1624_Time;
#endif
#if (APP_TM1640_ENABLE && TM1640_DISP_TEST)
uint8_t TM1640_Show[16];
uint16_t TM1640_Time;
#endif
#if APP_IWDG_ENABLE
void IWDG_Init(void)
{
	/* Enable LSI */
	LL_RCC_LSI_Enable();
	while (LL_RCC_LSI_IsReady() == 0U) {;}
	/* Enable IWDG */
	LL_IWDG_Enable(IWDG);
	/* Enable write access to IWDG_PR, IWDG_RLR and IWDG_WINR registers */
	LL_IWDG_EnableWriteAccess(IWDG);
	/* Set IWDG prescaler */
	LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_32); /* T=1MS */
	/* Set IWDG reload value */
	LL_IWDG_SetReloadCounter(IWDG, IDWG_RELOAD_VALUE); /* 1ms*1000=1s */
	/* Check if all flags Prescaler, Reload & Window Value Update are reset or not */
	while (LL_IWDG_IsReady(IWDG) == 0U) 
	{
		;
	}
	/* Reloads IWDG counter with value defined in the reload register */
	LL_IWDG_ReloadCounter(IWDG);
}
#endif
/********************************************************
**	������	��app_drivers_init
**	����	�����ڸ��ײ�������ʼ��
**	����	����
**	����	����
*********************************************************/
void app_drivers_init(void)
{
	#if APP_UART_ENABLE
	/*	��ʼ������ģ��	*/
	#if UART1_ENABLE
	UART1_Init(UART1_BAUDRATE);
	#endif
	#if UART2_ENABLE
	UART2_Init(UART2_BAUDRATE);
	#endif
	#if UART3_ENABLE
	UART3_Init(UART3_BAUDRATE);
	#endif
	#endif
	#if	OB_USER_OPTION
	/*	����FLASHѡ���ֽ�	*/
	uint8_t config = 0; 
	if(READ_BIT(FLASH->OPTR, FLASH_OPTR_NRST_MODE) != OB_GPIO_PIN_MODE)
		config = 1;
	else if(READ_BIT(FLASH->OPTR, FLASH_OPTR_BOR_EN) != OB_BOR_EN)
		config = 1;
	else if(READ_BIT(FLASH->OPTR, FLASH_OPTR_BOR_LEV) != OB_BOR_LEVEL)
		config = 1;
	else if(READ_BIT(FLASH->OPTR, FLASH_OPTR_IWDG_SW) != OB_IWDG_MODE)
		config = 1;
	else if(READ_BIT(FLASH->OPTR, FLASH_OPTR_IWDG_STOP) != OB_IWDG_STOP)
		config = 1;
	if(config)
	{
		/* OPTION Program */
		FLASH_OBProgramInitTypeDef OBInitCfg = {0};
		FLASH_Unlock();        /* Unlock Flash */
		FLASH_OB_Unlock();     /* Unlock Option */
		OBInitCfg.OptionType = OPTIONBYTE_USER;
		OBInitCfg.USERType = (OB_USER_BOR_EN | OB_USER_BOR_LEV | OB_USER_IWDG_SW | OB_USER_IWDG_STOP | OB_USER_NRST_MODE);
		OBInitCfg.USERConfig = (OB_BOR_EN | OB_BOR_LEVEL | OB_IWDG_MODE | OB_IWDG_STOP | OB_GPIO_PIN_MODE);
		/* Option Program */
		FLASH_OBProgram(&OBInitCfg);
		FLASH_Lock();      /* Lock Flash */
		FLASH_OB_Lock();   /* Lock Option */
		/* Option Launch */
		FLASH_OB_Launch();
	}	
	#endif
	#if APP_IWDG_ENABLE
	IWDG_Init();
	#endif
	#if APP_SMG_ENABLE
	/*	�����ɨ���ʼ��	*/
    SMG_Init();
	#endif
	#if APP_BEEP_ENABLE
	/*	��������ʼ��	*/
	BEEP_Init();
	#endif
	#if APP_IR_RECEIVED_ENABLE
	IR_Received_Init();
	#endif
	#if APP_TIM1_ENABLE
	/*	��ʱ��1��ʼ��	*/
	TIM1_Init();
	#elif APP_TIM1_PWM_ENABLE
	/*	��ʱ��1����PWM�����ʼ��	*/
	TIM1_PWM_Init();
	#endif
	#if APP_TIM14_ENABLE
	/*	��ʱ��14��ʼ��	*/
	TIM14_Init();
	#elif APP_TIM14_PWM_ENABLE
	/*	��ʱ��14����PWM�����ʼ��	*/
	TIM14_PWM_Init();
	#endif
	#if APP_ADC_ENABLE
	/*	ADCģ���ʼ��	*/
	ADC_Init();
	#endif
	#if APP_SPI_LED_ENABLE
	/*	ʹ��SPI����W2812B�����ʼ��	*/
	SPI_LED_Init();
	argb_init();
	#endif
	#if APP_USER_OTP_ENABLE
	/*	�û����ݶ�ȡ	*/
	#if LVD_WRITE_USER_DATA
	if(User_Cache_Read(0,&power_count,1))
	{
		log_printf("power_count:%d\r\n",power_count);
	}
	else
	{
		log_printf("first power\r\n");
		power_count = 0;
	}
	#endif
	#endif
	#if APP_TM1624_ENABLE
	TM1624_Init();
	#endif
	#if APP_TM1640_ENABLE
	TM1640_Init();
	#endif
}	
/********************************************************
**	������	void app_drivers_loop(void)
**	����	����������while(1)����ѯ�����ڴ����ģ�������
**	����	����
**	����	����
*********************************************************/
void app_drivers_loop(void)
{
	#if APP_IR_RECEIVED_ENABLE
	Ir_TypeDef remote;
	if(IR_Press(&remote))
	{
		log_printf("address:0X%X ",remote.ir_address);	//�յ��ĵ�ַ
		log_printf("command:0X%X ",remote.ir_command);	//�յ�������
		log_printf("count:%d\r\n",remote.ir_count);		//�յ��Ĵ���
	}
	#endif
	/*	�ⲿ�жϱ�־λ	*/
	if(EXTI_Flag != 0)
	{
		log_printf("EXTI_Flag:0X%X\r\n",EXTI_Flag);
		EXTI_Flag = 0;
	}
	#if APP_IWDG_ENABLE
	/* ι���Ź� */
	LL_IWDG_ReloadCounter(IWDG);
	#endif
	#if APP_ADC_ENABLE
	ADC_Loop();
	#endif
	#if APP_UART_ENABLE
	UART_Loop();
	#endif
	#if (APP_TM1624_ENABLE && TM1624_DISP_TEST)
	if(TM1624_Time > 500)
	{
		uint8_t i;
		TM1624_Time = 0;
		for(i = 0;i < 14;i++)
		{
			TM1624_Show[i] = ~TM1624_Show[i];
		}
		TM1624_Display_Update(TM1624_Show,14,TM1624_ON | TM1624_DISP_DIM);
	}
	#endif
	#if (APP_TM1640_ENABLE && TM1640_DISP_TEST)
	if(TM1640_Time > 500)
	{
		uint8_t i;
		TM1640_Time = 0;
		for(i = 0;i < 16;i++)
		{
			TM1640_Show[i] = ~TM1640_Show[i];
		}
		TM1640_Display_Update(TM1640_Show,16,TM1640_ON | TM1640_DISP_DIM);
	}
	#endif
}
/********************************************************
**	������	void app_drivers_timer(void)
**	����	����ϵͳ�δ�ʱ���н��лص�����������Ϊ1ms
**	����	����
**	����	����
*********************************************************/
void app_drivers_timer(void)
{
	#if (APP_TM1624_ENABLE && TM1624_DISP_TEST)
	TM1624_Time++;
	#endif
	#if (APP_TM1640_ENABLE && TM1640_DISP_TEST)
	TM1640_Time++;
	#endif
	#if APP_SMG_ENABLE
	SMG_Scan();
	#endif
	#if APP_BEEP_ENABLE
	BEEP_Timeout();
	#endif
	#if APP_ADC_ENABLE
	adc_tick++;
	#endif
	#if APP_UART_ENABLE
	UART_TimeOut();
	#endif
	#if APP_SPI_LED_ENABLE
	argb_time++;
	#endif
}
/********************************************************
**	������	void EXTI0_15_IRQHandlerCallback(uint32_t PR)
**	����	���ⲿ�ж�0��15�Ļص��������˺����������ж��ڣ������ں����ڽ�����ʱ�Ȳ���
**	����	��PR���������жϱ�־��BIT0Ϊ�ⲿ�ж�0��BIT1Ϊ�ⲿ�ж�1���Դ�����
**	����	����
*********************************************************/
void EXTI0_15_IRQHandlerCallback(uint32_t PR)
{
	uint8_t i;
	for(i = 0;i < 16;i++)
	{
		/* Handle EXTI interrupt request */
		if(PR & (1 << i))
		{
			/*	�����ⲿ�ж�		*/
			EXTI_Flag |= 1 << i;
		}
	}
}
#if APP_TIM1_ENABLE
/********************************************************
**	������	void TIM1_PeriodElapsedCallback(void)
**	����	����ʱ��1�ص�����������ʱ�¼�
**	����	����
**	����	����
*********************************************************/
void TIM1_PeriodElapsedCallback(void)
{
	#if (APP_IR_RECEIVED_ENABLE == 1 && D_IR_TIM == 1)
	IR_Received_Scan();
	#endif
}
#endif
#if APP_TIM14_ENABLE
/********************************************************
**	������	void TIM14_PeriodElapsedCallback(void)
**	����	����ʱ��14�ص�����������ʱ�¼�
**	����	����
**	����	����
*********************************************************/
void TIM14_PeriodElapsedCallback(void)
{
	#if (APP_IR_RECEIVED_ENABLE == 1 && D_IR_TIM == 14)
	IR_Received_Scan();
	#endif
}
#endif
/********************************************************
**	������	void nop_delay_xus(uint16_t nus)
**	����	�����ڲ���׼����ʱ
**	����	��nus,��Ҫ��ʱ��ʱ��
**	����	����
*********************************************************/
void nop_delay_xus(uint16_t nus)
{
    __IO uint32_t wait_loop_index = (nus * fac_us);
    while (wait_loop_index != 0U)
    {
        wait_loop_index--;
    }
}
/********************************************************
**	������	void delay_us(uint16_t nus)
**	����	��ʹ��ϵͳ�δ�ʱ����ʱnus
**	����	��nus,��Ҫ��ʱ��ʱ��
**	����	����
*********************************************************/
void delay_us(uint16_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
	uint32_t reload;
    reload = SysTick->LOAD; // LOAD��ֵ
    ticks = nus * fac_us;            // ��Ҫ�Ľ�����
    told = SysTick->VAL; // �ս���ʱ�ļ�����ֵ
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; // ����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
        }
    };
}
/********************************************************
**	������	void delay_ms(uint8_t nms)
**	����	��ʹ��ϵͳ�δ�ʱ����ʱnms
**	����	��nus,��Ҫ��ʱ��ʱ��
**	����	����
*********************************************************/
void delay_ms(uint16_t nms)
{
    for(uint16_t i = 0;i < nms;i++)
	{
		delay_us(1000);
	}
}

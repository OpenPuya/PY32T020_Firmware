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
**	函数名	：app_drivers_init
**	描述	：用于各底层驱动初始化
**	传入	：无
**	返回	：无
*********************************************************/
void app_drivers_init(void)
{
	#if APP_UART_ENABLE
	/*	初始化串口模块	*/
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
	/*	配置FLASH选项字节	*/
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
	/*	数码管扫描初始化	*/
    SMG_Init();
	#endif
	#if APP_BEEP_ENABLE
	/*	蜂鸣器初始化	*/
	BEEP_Init();
	#endif
	#if APP_IR_RECEIVED_ENABLE
	IR_Received_Init();
	#endif
	#if APP_TIM1_ENABLE
	/*	定时器1初始化	*/
	TIM1_Init();
	#elif APP_TIM1_PWM_ENABLE
	/*	定时器1用做PWM输出初始化	*/
	TIM1_PWM_Init();
	#endif
	#if APP_TIM14_ENABLE
	/*	定时器14初始化	*/
	TIM14_Init();
	#elif APP_TIM14_PWM_ENABLE
	/*	定时器14用做PWM输出初始化	*/
	TIM14_PWM_Init();
	#endif
	#if APP_ADC_ENABLE
	/*	ADC模块初始化	*/
	ADC_Init();
	#endif
	#if APP_SPI_LED_ENABLE
	/*	使用SPI驱动W2812B灯珠初始化	*/
	SPI_LED_Init();
	argb_init();
	#endif
	#if APP_USER_OTP_ENABLE
	/*	用户数据读取	*/
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
**	函数名	void app_drivers_loop(void)
**	描述	：在主函数while(1)中轮询，用于处理各模块的任务
**	传入	：无
**	返回	：无
*********************************************************/
void app_drivers_loop(void)
{
	#if APP_IR_RECEIVED_ENABLE
	Ir_TypeDef remote;
	if(IR_Press(&remote))
	{
		log_printf("address:0X%X ",remote.ir_address);	//收到的地址
		log_printf("command:0X%X ",remote.ir_command);	//收到的命令
		log_printf("count:%d\r\n",remote.ir_count);		//收到的次数
	}
	#endif
	/*	外部中断标志位	*/
	if(EXTI_Flag != 0)
	{
		log_printf("EXTI_Flag:0X%X\r\n",EXTI_Flag);
		EXTI_Flag = 0;
	}
	#if APP_IWDG_ENABLE
	/* 喂看门狗 */
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
**	函数名	void app_drivers_timer(void)
**	描述	：在系统滴答定时器中进行回调，调用周期为1ms
**	传入	：无
**	返回	：无
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
**	函数名	void EXTI0_15_IRQHandlerCallback(uint32_t PR)
**	描述	：外部中断0到15的回调函数，此函数工作在中断内，请勿在函数内进行延时等操作
**	传入	：PR：产生的中断标志，BIT0为外部中断0，BIT1为外部中断1，以此类推
**	返回	：无
*********************************************************/
void EXTI0_15_IRQHandlerCallback(uint32_t PR)
{
	uint8_t i;
	for(i = 0;i < 16;i++)
	{
		/* Handle EXTI interrupt request */
		if(PR & (1 << i))
		{
			/*	产生外部中断		*/
			EXTI_Flag |= 1 << i;
		}
	}
}
#if APP_TIM1_ENABLE
/********************************************************
**	函数名	void TIM1_PeriodElapsedCallback(void)
**	描述	：定时器1回调函数，处理定时事件
**	传入	：无
**	返回	：无
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
**	函数名	void TIM14_PeriodElapsedCallback(void)
**	描述	：定时器14回调函数，处理定时事件
**	传入	：无
**	返回	：无
*********************************************************/
void TIM14_PeriodElapsedCallback(void)
{
	#if (APP_IR_RECEIVED_ENABLE == 1 && D_IR_TIM == 14)
	IR_Received_Scan();
	#endif
}
#endif
/********************************************************
**	函数名	void nop_delay_xus(uint16_t nus)
**	描述	：用于不精准的延时
**	传入	：nus,需要延时的时间
**	返回	：无
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
**	函数名	void delay_us(uint16_t nus)
**	描述	：使用系统滴答定时器延时nus
**	传入	：nus,需要延时的时间
**	返回	：无
*********************************************************/
void delay_us(uint16_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
	uint32_t reload;
    reload = SysTick->LOAD; // LOAD的值
    ticks = nus * fac_us;            // 需要的节拍数
    told = SysTick->VAL; // 刚进入时的计数器值
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; // 这里注意一下SYSTICK是一个递减的计数器就可以了.
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // 时间超过/等于要延迟的时间,则退出.
        }
    };
}
/********************************************************
**	函数名	void delay_ms(uint8_t nms)
**	描述	：使用系统滴答定时器延时nms
**	传入	：nus,需要延时的时间
**	返回	：无
*********************************************************/
void delay_ms(uint16_t nms)
{
    for(uint16_t i = 0;i < nms;i++)
	{
		delay_us(1000);
	}
}

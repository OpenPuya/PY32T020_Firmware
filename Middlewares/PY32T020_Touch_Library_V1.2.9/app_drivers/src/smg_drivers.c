/**
 ******************************************************************************
 * @file    main.c
 * @author  MCU Application Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
 * All rights reserved.</center></h2>
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "smg_drivers.h"
#if APP_SMG_ENABLE
static uint8_t smg_run;
static uint8_t smg_com;
uint8_t smg_data[COM_COUNT];
/**
 * @brief 数码管IO
 */
const uint8_t SEG_PIN[8] = {	SEG0_GPIO_PIN, SEG1_GPIO_PIN, SEG2_GPIO_PIN, SEG3_GPIO_PIN, 
								SEG4_GPIO_PIN, SEG5_GPIO_PIN, SEG6_GPIO_PIN, SEG7_GPIO_PIN};
const uint8_t COM_PIN[5] = {	COM0_GPIO_PIN, COM1_GPIO_PIN, COM2_GPIO_PIN, COM3_GPIO_PIN, COM4_GPIO_PIN};
/********************************************************
**	函数名	void SMG_Init(void)
**	描述	数码管IO口初始化，
**	传入	：无
**	返回	：无
*********************************************************/
void SMG_Init(void)
{
    uint8_t i;
    smg_com = 0;
    for (i = 0; i < SEG_COUNT; i++)
    {
		GPIO_Init(SEG_PIN[i],OUTPUT|PUSHPULL);
		GPIO_ClearBit(SEG_PIN[i]);
    }
    /*Configure the COM IO*/
    for (i = 0; i < COM_COUNT; i++)
    {
		GPIO_Init(COM_PIN[i],OUTPUT|PUSHPULL);
		GPIO_SetBit(COM_PIN[i]);
		if(COM_PIN[i] == PA11)
			SET_BIT(SYSCFG->IOCFG, SYSCFG_IOCFG_PA_EHS_0);
		else if(COM_PIN[i] == PA12)
			SET_BIT(SYSCFG->IOCFG, SYSCFG_IOCFG_PA_EHS_1);
		else if(COM_PIN[i] == PA13)
			SET_BIT(SYSCFG->IOCFG, SYSCFG_IOCFG_PA_EHS_2);
		else if(COM_PIN[i] == PA14)
			SET_BIT(SYSCFG->IOCFG, SYSCFG_IOCFG_PA_EHS_3);
		else if(COM_PIN[i] == PA15)
			SET_BIT(SYSCFG->IOCFG, SYSCFG_IOCFG_PA_EHS_4);
    }
	SMG_Default();
    smg_run = 1;
}
/********************************************************
**	函数名	void SMG_Sleep(void)
**	描述	数码管休眠，设置IO口状态
**	传入	：无
**	返回	：无
*********************************************************/
void SMG_Sleep(void)
{
	uint8_t i;
	smg_run = 0;
	GPIO_SetBit(COM_PIN[smg_com]);
	for (i = 0; i < SEG_COUNT; i++)
    {
		GPIO_ClearBit(SEG_PIN[i]);
    }
}
/********************************************************
**	函数名	void SMG_Wake(void)
**	描述	数码管退出休眠，继续显示
**	传入	：无
**	返回	：无
*********************************************************/
void SMG_Wake(void)
{
	smg_run = 1;
}
/********************************************************
**	函数名	void SMG_Scan(void)
**	描述	数码管扫描
**	传入	：无
**	返回	：无
*********************************************************/
void SMG_Scan(void)
{
    uint8_t data, i;
    if (smg_run == 0)
        return;
    GPIO_SetBit(COM_PIN[smg_com]);
    smg_com++;
    if (smg_com >= COM_COUNT)
        smg_com = 0;
    data = smg_data[smg_com];
    for (i = 0; i < SEG_COUNT; i++)
    {
        if (data & 0X01)
			GPIO_SetBit(SEG_PIN[i]);
        else
			GPIO_ClearBit(SEG_PIN[i]);
        data >>= 1;
    }
	GPIO_ClearBit(COM_PIN[smg_com]);
}
#endif

/**
  ******************************************************************************
  * @file    py32t020_it.c
  * @author  MCU Application Team
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
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
#include "main.h"
#include "py32t020_it.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint16_t ADCCoversionValue[3];
extern uint8_t ADCCoversionFlag;
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers         */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/* PY32T020 Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

void ADC_COMP_IRQHandler(void)
{
  /* Check status flag bit of analog watchdog */
  if(LL_ADC_IsActiveFlag_AWD(ADC1) != 0 && LL_ADC_IsEnabledIT_AWD(ADC1))
  {
    /* Clear status flag bit of analog watchdog */
    LL_ADC_ClearFlag_AWD(ADC1);

    /* Call analog watchdog IT handler program */
    APP_AdcAWDCallback();
  }

  static uint8_t channel = 0;
  // EOC中断
  if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0 && LL_ADC_IsEnabledIT_EOC(ADC1))
  {
    /* Clear flag ADC group regular end of unitary conversion */
    LL_ADC_ClearFlag_EOC(ADC1);

    ADCCoversionValue[channel] = LL_ADC_REG_ReadConversionData12(ADC1);
    channel++;
  }

  // EOS中断
  if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0 && LL_ADC_IsEnabledIT_EOS(ADC1))
  {
    /* Clear flag ADC group regular end of sequence conversions */
    LL_ADC_ClearFlag_EOS(ADC1);

    ADCCoversionFlag = 1;
    channel = 0;
  }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

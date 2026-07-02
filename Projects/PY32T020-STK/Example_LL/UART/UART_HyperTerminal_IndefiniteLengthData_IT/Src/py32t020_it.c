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

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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

/**
  * @brief This function handles UART3 interrupt.
  */
void UART3_IRQHandler(void)
{
  /* receive data register not empty */
  if ((LL_UART_IsActiveFlag_RXNE(UART3) != RESET) && (LL_UART_IsEnabledIT_RXNE(UART3) != RESET))
  {
    /* Receive data */
    aRxBuffer[cRxIndex] = (uint8_t)(UART3->DR & (uint8_t)0x00FF);

    /* Wait SR_TXE bit set 1 */
    while(LL_UART_IsActiveFlag_TXE(UART3) == RESET)
    {
    }
    
    /* Send received data */
    UART3->DR = aRxBuffer[cRxIndex];
    
    cRxIndex++;
    if(cRxIndex > (RX_MAX_LEN - 1))
    {
      cRxIndex = (RX_MAX_LEN - 1);
    }
  }
  
  if(LL_UART_IsActiveFlag_ORE(UART3) == SET)
  {
    /* Clearing the ORE bit */
    LL_UART_ClearFlag_ORE(UART3);
    
    /* Error callback function */
    APP_UsartErrorCallback();
  }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

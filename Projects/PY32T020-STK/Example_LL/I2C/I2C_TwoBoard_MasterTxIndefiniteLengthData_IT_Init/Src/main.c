/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
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
#include "py32t020xx_ll_Start_Kit.h"

/* Private define ------------------------------------------------------------*/
#define I2C_ADDRESS        0xA0     /* Local/Slave address */
#define I2C_SPEEDCLOCK     100000   /* Communication speed : 100K */
#define I2C_STATE_READY    0        /* Ready state */
#define I2C_STATE_BUSY_TX  1        /* Transmission state */
#define I2C_STATE_BUSY_RX  2        /* Reception state */

/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[100] = {0};

uint8_t         *pBuffPtr   = NULL;
__IO uint16_t   XferCount   = 0;
__IO uint32_t   Devaddress  = 0;
__IO uint32_t   State       = I2C_STATE_READY;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigI2cMaster(void);
static void APP_MasterTransmit_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  uint32_t i = 0;
  
  /* Configure system clock */
  APP_SystemClockConfig();
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);
  
  /* Configure I2C */
  APP_ConfigI2cMaster();

  /* Wait for the button to be pressed */
  while(BSP_PB_GetState(BUTTON_KEY) == 0)
  {
  }
  
  for(i = 0; i < 10; i++)
  {
    aTxBuffer[i] = i;
  }
  /* Master transmits data */
  APP_MasterTransmit_IT(I2C_ADDRESS, (uint8_t *)aTxBuffer, 10);
  
  /* Wait for master to finish sending data */
  while (State != I2C_STATE_READY);
  
  /* Delay purpose: Wait for the slave to receive and print the received data */
  LL_mDelay(200);
  for(i = 0; i < 100; i++)
  {
    aTxBuffer[i] = i + 1;
  }
  /* Master transmits data */
  APP_MasterTransmit_IT(I2C_ADDRESS, (uint8_t *)aTxBuffer, 100);
  
  /* Wait for master to finish sending data */
  while (State != I2C_STATE_READY);
  
  /* Delay purpose: Wait for the slave to receive and print the received data */
  LL_mDelay(200);
  for(i = 0; i < 10; i++)
  {
    aTxBuffer[i] = i;
  }
  /* Master transmits data */
  APP_MasterTransmit_IT(I2C_ADDRESS, (uint8_t *)aTxBuffer, 10);
  
  /* Wait for master to finish sending data */
  while (State != I2C_STATE_READY);
  
  while (1)
  {
  }
}

/**
  * @brief  Configure Systemclock
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
  /* Enable HSI */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB divider: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* HSISYS used as SYSCLK clock source  */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }

  /* Set APB1 divider */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}

/**
  * @brief  Configure I2C peripheral
  * @param  None
  * @retval None
  */
static void APP_ConfigI2cMaster(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /* Enable I2C1 peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* Set PA11 to SCL pin , Select alternate function mode
     and fast output speed. output type is Selected open-drain,
     Enable I/O pull up*/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF4_I2C;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Set PA8 to SDA pin , Select alternate function mode
     and fast output speed. output type is Selected open-drain,
     Enable I/O pull up*/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF4_I2C;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Enable I2C Peripheral interruption request */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* Reset I2C peripheral */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);

  /* Initialize I2C1 peripheral */
  I2C_InitStruct.ClockSpeed      = I2C_SPEEDCLOCK;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.DataHoldTimeSel = LL_I2C_DATA_HOLD_TIME_HARDWARE;
  I2C_InitStruct.OwnAddress1     = I2C_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);

  /* Enable clock stretching */
  /* Reset value is clock stretching enabled */
  /* LL_I2C_EnableClockStretching(I2C1); */

  /* Enable general call  */
  /* Reset value is general call disabled */
  /* LL_I2C_EnableGeneralCall(I2C1); */
}

/**
  * @brief  I2C Send data with interrupt
  * @param  DevAddress：Slave address
  * @param  pData：pData pointer to Send buffer
  * @param  Size：Size Amount of data to be sent
  * @retval None
  */
static void APP_MasterTransmit_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  /* Clear pos bit */
  LL_I2C_DisableBitPOS(I2C1);

  /* Update the slave address, data to be sent, data size, and status to global variables */
  pBuffPtr    = pData;
  XferCount   = Size;
  Devaddress  = DevAddress;
  State       = I2C_STATE_BUSY_TX;

  /* Generate Start signal */
  LL_I2C_GenerateStartCondition(I2C1);

  /* Enable event interrupt、buffer interrupt and error interrupt */
  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_BUF(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);
}

/**
  * @brief  I2C interrupt callback function
  * @param  None
  * @retval None
  */
void APP_MasterIRQCallback(void)
{
  /* Set SB flag */
  if ((LL_I2C_IsActiveFlag_SB(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
  {
    /* Send slave address + direction bit */
    if (State == I2C_STATE_BUSY_TX)
    {
      LL_I2C_TransmitData8(I2C1, (Devaddress & (uint8_t)(~0x01)));
    }
    else
    {
      LL_I2C_TransmitData8(I2C1, (Devaddress | 0x1));
    }
  }
  /* Set ADDR flag */
  else if ((LL_I2C_IsActiveFlag_ADDR(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
  {
    if (State == I2C_STATE_BUSY_RX)
    {
      if (XferCount == 0U)
      {
        LL_I2C_ClearFlag_ADDR(I2C1);
        LL_I2C_GenerateStopCondition(I2C1);
      }
      else if (XferCount == 1U)
      {
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
        LL_I2C_ClearFlag_ADDR(I2C1);
        LL_I2C_GenerateStopCondition(I2C1);
      }
      else if (XferCount == 2U)
      {
        LL_I2C_EnableBitPOS(I2C1);
        LL_I2C_ClearFlag_ADDR(I2C1);
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
      }
      else
      {
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
        LL_I2C_ClearFlag_ADDR(I2C1);
      }
    }
    else
    {
      LL_I2C_ClearFlag_ADDR(I2C1);
    }
  }
  /* Master send direction */
  else if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE)
  {
    /* Set TXE flag, BTF flag is not set */
    if ((LL_I2C_IsActiveFlag_TXE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
    {
      if (XferCount == 0U)
      {
        LL_I2C_DisableIT_BUF(I2C1);
      }
      else
      {
        LL_I2C_TransmitData8(I2C1, *pBuffPtr);
        pBuffPtr++;
        XferCount--;
      }
    }
    /* Set BTF flag */
    else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
    {
      if (XferCount != 0U)
      {
        LL_I2C_TransmitData8(I2C1, *pBuffPtr);
        pBuffPtr++;
        XferCount--;
      }
      else
      {
        LL_I2C_DisableIT_EVT(I2C1);
        LL_I2C_DisableIT_BUF(I2C1);
        LL_I2C_DisableIT_ERR(I2C1);
        
        LL_I2C_GenerateStopCondition(I2C1);
        State = I2C_STATE_READY;
        
      }
    }
  }
}

/**
  * @brief  Error handling function
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file:Pointer to the source file name
  * @param  line:assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add His own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

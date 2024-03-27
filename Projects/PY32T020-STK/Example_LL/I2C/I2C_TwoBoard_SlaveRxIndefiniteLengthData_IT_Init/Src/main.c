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

/* Private variables ---------------------------------------------------------*/
#define RX_MAX_LEN 200     /* Single frame data, maximum received data length */
uint32_t RxLen = 0;        /* Single frame data, actual received data length */
uint8_t RxBuffer[RX_MAX_LEN] = {0}; /* Receive buffer */
uint8_t RevOkFlag = 0;     /* Single frame data received completion flag */

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigI2cSlave(void);
static void APP_SlaveReceive_IT(void);

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
  
  /* Initialize UART */
  BSP_UART_Config();
  
  /* Configure I2C */
  APP_ConfigI2cSlave();
  
  /* Slave receives data */
  APP_SlaveReceive_IT();
  
  while (1)
  {
    if (RevOkFlag == 1)
    {
      for (i = 0; i < RxLen; i++)
      {
        printf("%d ", RxBuffer[i]);
      }
      RxLen = 0;
      RevOkFlag = 0;
    }
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
  * @brief  I2C configuration function
  * @param  None
  * @retval None
  */
static void APP_ConfigI2cSlave(void)
{
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /* Enable I2C1 peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* Set PA11 to SCL pin , Select alternate function mode
     and fast output speed. output type is Selected open-drain,
     Enable I/O pull up*/
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

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

  /* I2C initialization */
  LL_I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.ClockSpeed      = I2C_SPEEDCLOCK;
  I2C_InitStruct.DutyCycle       = LL_I2C_DUTYCYCLE_16_9;
  I2C_InitStruct.DataHoldTimeSel = LL_I2C_DATA_HOLD_TIME_HARDWARE;
  I2C_InitStruct.OwnAddress1     = I2C_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_NACK;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  
  /* Enable clock stretching */
  /* Reset value is clock stretching enabled */
  /* LL_I2C_EnableClockStretching(I2C1); */
  
  /* Enable general call */
  /* Reset value is general call disabled */
  /* LL_I2C_EnableGeneralCall(I2C1); */
}

/**
  * @brief  I2C reception function
  * @param  pData：Pointer to data to be received
  * @param  Size：Size of data to be received
  * @retval None
  */
static void APP_SlaveReceive_IT(void)
{
  /* Clear POS bit */
  LL_I2C_DisableBitPOS(I2C1);
  
  /* Enable acknowledge */
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
  
  /* Enable interrupt */
  LL_I2C_EnableIT_EVT(I2C1);
  LL_I2C_EnableIT_BUF(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);
}

/**
  * @brief  I2C interrupt callback function
  * @param  None
  * @retval None
  */
void APP_SlaveIRQCallback(void)
{
  /* Set ADDR flag */
  if ((LL_I2C_IsActiveFlag_ADDR(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
  {
    LL_I2C_ClearFlag_ADDR(I2C1);
  }
  /* Set STOP flag */
  else if (LL_I2C_IsActiveFlag_STOP(I2C1) == 1)
  {
    
    LL_I2C_ClearFlag_STOP(I2C1);
    
    RevOkFlag = 1;
  }
  /* Slave Receive */
  else
  {
    /* Set RXNE flag, BTF flag is not set */
    if ((LL_I2C_IsActiveFlag_RXNE(I2C1) == 1) && (LL_I2C_IsEnabledIT_BUF(I2C1) == 1) && (LL_I2C_IsActiveFlag_BTF(I2C1) == 0))
    {
        RxBuffer[RxLen++] = LL_I2C_ReceiveData8(I2C1);
    }
    /* Set BTF flag */
    else if ((LL_I2C_IsActiveFlag_BTF(I2C1) == 1) && (LL_I2C_IsEnabledIT_EVT(I2C1) == 1))
    {
        RxBuffer[RxLen++] = LL_I2C_ReceiveData8(I2C1);
    }
  }
}

/**
  * @brief  After the I2C master receives the last byte, send NACK to the slave, slave NACK 
  * @param  None
  * @retval None
  */
void APP_SlaveIRQCallback_NACK(void)
{
  if ((LL_I2C_IsActiveFlag_AF(I2C1) == 1) && (LL_I2C_IsEnabledIT_ERR(I2C1) == 1))
  {
      LL_I2C_ClearFlag_AF(I2C1);
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
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
/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[] = "UART Test";
uint8_t aRxBuffer[30] = {0};

uint8_t *TxBuff = NULL;
__IO uint16_t TxSize = 0;
__IO uint16_t TxCount = 0;

uint8_t *RxBuff = NULL;
__IO uint16_t RxSize = 0;
__IO uint16_t RxCount = 0;

__IO ITStatus UartReady = RESET;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_ConfigUART(UART_TypeDef *UARTx);
static void APP_UartTransmit_IT(UART_TypeDef *UARTx, uint8_t *pData, uint16_t Size);
static void APP_UartReceive_IT(UART_TypeDef *UARTx, uint8_t *pData, uint16_t Size);

/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Configure Systemclock */
  APP_SystemClockConfig();

  /* Configure UART */
  APP_ConfigUART(UART3);

  /* Send string:"UART Test"，and wait send complete */
  APP_UartTransmit_IT(UART3, (uint8_t*)aTxBuffer, sizeof(aTxBuffer)-1);
  while (UartReady != SET)
  {
  }
  UartReady = RESET;

  while (1)
  {
    /* receive data */
    APP_UartReceive_IT(UART3, (uint8_t *)aRxBuffer, 12);
    while (UartReady != SET)
    {
    }
    UartReady = RESET;

    /* Transmit data */
    APP_UartTransmit_IT(UART3, (uint8_t*)aRxBuffer, 12);
    while (UartReady != SET)
    {
    }
    UartReady = RESET;
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
  * @brief  UART configuration functions
  * @param  UARTx：UART Instance，This parameter can be one of the following values:UART3
  * @retval None
  */
static void APP_ConfigUART(UART_TypeDef *UARTx)
{
  /* Enable clock, initialize GPIO, enable NVIC interrupt */
  if (UARTx == UART3)
  {
    /* Enable GPIOA clock */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    /* Enable UART3 peripheral clock */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UART3);

    /* Initialize PA2 */
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    /* Select pin 2 */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    /* Select alternate mode */
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    /* Set output speed */
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    /* Set output type to push pull */
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    /* Enable pull up */
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    /* Set alternate function to UART3 function  */
    GPIO_InitStruct.Alternate = LL_GPIO_AF1_UART3;
    /* Initialize GPIOA */
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    /* Select pin 3 */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    /* Set alternate function to UART3 function */
    GPIO_InitStruct.Alternate = LL_GPIO_AF1_UART3;
    /* Initialize GPIOA */
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    /* Set UART3 interrupt priority  */
    NVIC_SetPriority(UART3_IRQn,0);
    /* Enable UART3 interrupt request */
    NVIC_EnableIRQ(UART3_IRQn);
  }

  /* Set UART feature */
  LL_UART_InitTypeDef UART_InitStruct = {0};
  /* Set baud rate */
  UART_InitStruct.BaudRate = 115200;
  /* set word length to 8 bits: Start bit, 8 data bits, n stop bits */
  UART_InitStruct.DataWidth = LL_UART_DATAWIDTH_8B;
  /* 1 stop bit */
  UART_InitStruct.StopBits = LL_UART_STOPBITS_1;
  /* Parity control disabled  */
  UART_InitStruct.Parity = LL_UART_PARITY_NONE;
  /* MSB first disable */
  UART_InitStruct.BitOrder = LL_UART_BITORDER_LSBFIRST;

  /* Initialize UART */
  LL_UART_Init(UARTx, &UART_InitStruct);
}

/**
  * @brief  UART transmission function
  * @param  UARTx：UART Instance，This parameter can be one of the following values:UART3
  * @param  pData：Pointer to transmission buffer
  * @param  Size：Size of transmission buffer
  * @retval None
  */
static void APP_UartTransmit_IT(UART_TypeDef *UARTx, uint8_t *pData, uint16_t Size)
{
  TxBuff = pData;
  TxSize = Size;
  TxCount = Size;

  /* Enable transmit data register empty interrupt */
  LL_UART_EnableIT_TDRE(UARTx);
  /* Enable Busy Error Interrupt */
  LL_UART_EnableIT_BUSYERR(UARTx);
}

/**
  * @brief  UART receive function
  * @param  UARTx：UART Instance，This parameter can be one of the following values:UART3
  * @param  pData：Pointer to receive buffer
  * @param  Size：Size of receive buffer
  * @retval None
  */
static void APP_UartReceive_IT(UART_TypeDef *UARTx, uint8_t *pData, uint16_t Size)
{
  RxBuff = pData;
  RxSize = Size;
  RxCount = Size;

  /* Enable line status error interrupt */
  LL_UART_EnableIT_LS(UARTx);
  /* Enable Busy Error Interrupt */
  LL_UART_EnableIT_BUSYERR(UARTx);
  /* Enable receive data register not empty interrupt */
  LL_UART_EnableIT_RXNE(UARTx);
}

/**
  * @brief  UART interrupt handler function
  * @param  UARTx：UART Instance，This parameter can be one of the following values:UART3
  * @retval None
  */
void APP_UartIRQCallback(UART_TypeDef *UARTx)
{
  /* The receive data register is not empty */
  uint32_t errorflags = ((LL_UART_IsActiveFlag_PE(UARTx) | LL_UART_IsActiveFlag_FE(UARTx)   |\
                          LL_UART_IsActiveFlag_ORE(UARTx) | LL_UART_IsActiveFlag_BRI(UARTx)) &&\
                         (LL_UART_IsEnabledIT_LS(UARTx))) ||\
                        ((LL_UART_IsActiveFlag_BUSY_ERR(UARTx)) && (LL_UART_IsEnabledIT_BUSYERRIE(UARTx)));
  if (errorflags == RESET)
  {
    if ((LL_UART_IsActiveFlag_RXNE(UARTx) != RESET) && (LL_UART_IsEnabledIT_RXNE(UARTx) != RESET))
    {
      *RxBuff = LL_UART_ReceiveData(UARTx);
       RxBuff++;

      if (--RxCount == 0U)
      {
        LL_UART_DisableIT_RXNE(UARTx);
        LL_UART_DisableIT_LS(UARTx);
        LL_UART_DisableIT_BUSYERR(UARTx);

        UartReady = SET;
      }
      return;
    }
  }

  /* An error occurred */
  if (errorflags != RESET)
  {
    APP_ErrorHandler();
  }

  /* The transmit data register is empty */
  if ((LL_UART_IsActiveFlag_TDRE(UARTx) != RESET) && (LL_UART_IsEnabledIT_TDRE(UARTx) != RESET))
  {
    LL_UART_TransmitData(UARTx, *TxBuff);
    TxBuff++;

    if (--TxCount == 0U)
    {
      LL_UART_DisableIT_TDRE(UARTx);
      LL_UART_DisableIT_BUSYERR(UARTx);
      
      UartReady = SET;
    }
    
    return;
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

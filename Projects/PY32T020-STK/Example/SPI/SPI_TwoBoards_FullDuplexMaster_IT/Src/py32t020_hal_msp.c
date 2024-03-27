/**
  ******************************************************************************
  * @file    py32t020_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
  * @brief Initialize global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize MSP for SPI.
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();                   /* GPIOA clock enable */
  __HAL_RCC_SPI1_CLK_ENABLE();                    /* SPI1 clock enable */
  
  /*
    PA4   ------> NSS
    PA5   ------> SCK
    PA6   ------> MISO
    PA7   ------> MOSI
  */
  /*SCK*/
  GPIO_InitStruct.Pin        = GPIO_PIN_5;
  if (hspi->Init.CLKPolarity == SPI_POLARITY_LOW)
  {
    GPIO_InitStruct.Pull     = GPIO_PULLDOWN;
  }
  else
  {
    GPIO_InitStruct.Pull     = GPIO_PULLUP;
  }
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed      = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate  = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI NSS*/
  GPIO_InitStruct.Pin        = GPIO_PIN_4;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull       = GPIO_PULLUP;
  GPIO_InitStruct.Speed      = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate  = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* MISO/MOSI*/
  GPIO_InitStruct.Pin        = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed      = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate  = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Interrupt Configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

/**
  * @brief  De-initialize SPI's MSP
  * @param  hspi：SPI handle
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  /* Reset SPI peripherals */
  __HAL_RCC_SPI1_FORCE_RESET();
  __HAL_RCC_SPI1_RELEASE_RESET();

  /* Turn off peripherals and GPIO clock */
  /* Unconfigure SPI SCK*/
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

  HAL_NVIC_DisableIRQ(SPI1_IRQn);
}
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

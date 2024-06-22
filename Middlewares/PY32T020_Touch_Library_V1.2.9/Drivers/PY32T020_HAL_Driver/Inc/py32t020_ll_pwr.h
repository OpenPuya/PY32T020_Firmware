/**
  ******************************************************************************
  * @file    py32t020_ll_pwr.h
  * @author  MCU Application Team
  * @brief   Header file of PWR LL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PY32T020_LL_PWR_H
#define __PY32T020_LL_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32t0xx.h"

/** @addtogroup PY32T020_LL_Driver
  * @{
  */

#if defined(PWR)

/** @defgroup PWR_LL PWR
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_LL_EC_SRAM_RETENTION_VOLTAGE SRAM RETENTION VOLTAGE
  * @{
  */
#define LL_PWR_SRAM_RETENTION_VOLT_NORMAL     0x00000000U                                            /* Set SRAM voltage in stop mode */
#define LL_PWR_SRAM_RETENTION_VOLT_STOP       (PWR_CR1_SRAM_RETV_CTRL_1                          )   /* Set SRAM voltage in stop mode */
#define LL_PWR_SRAM_RETENTION_VOLT_DLP        (PWR_CR1_SRAM_RETV_CTRL_1                          )   /* Set SRAM voltage in deep lowpower mode */
#define LL_PWR_SRAM_RETENTION_VOLT_HIBERNATE  (PWR_CR1_SRAM_RETV_CTRL_1 | PWR_CR1_SRAM_RETV_CTRL_0)  /* Set SRAM voltage in hibernate mode */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_HSION_MODE WAKEUP HSI ON MODE
  * @{
  */
#define LL_PWR_WAKEUP_HSION_AFTER_MR       0x00000000U         /* Wake up from the STOP mode, After the MR becomes stable, enable HSI */
#define LL_PWR_WAKEUP_HSION_IMMEDIATE      PWR_CR1_HSION_CTRL  /* Wake up from the STOP mode, Enable HSI immediately */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_LOW_POWER_REGULATOR_MODE LOW POWER REGULATOR MODE
  * @{
  */
#define LL_PWR_LPR_MODE_MR         0x00000000U                 /* MR mode */
#define LL_PWR_LPR_MODE_LPR        PWR_CR1_LPR_0               /* Low Power Run mode */
#define LL_PWR_LPR_MODE_DLPR       PWR_CR1_LPR_1               /* Deep Low Power Run mode */
/**
  * @}
  */

/** @defgroup PWR_LL_EC_WAKEUP_FLASH_DELAY WAKEUP FLASH DELAY
  * @{
  */
#define LL_PWR_WAKEUP_FLASH_DELAY_0US      (PWR_CR1_FLS_SLPTIME_1 | PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, Enable falsh immediately*/
#define LL_PWR_WAKEUP_FLASH_DELAY_2US      (                        PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, Delay 2us enable falsh*/
#define LL_PWR_WAKEUP_FLASH_DELAY_3US      (PWR_CR1_FLS_SLPTIME_1                        ) /* Wake up from the STOP mode, Delay 3us enable falsh*/
#define LL_PWR_WAKEUP_FLASH_DELAY_5US      0x00000000U                                     /* Wake up from the STOP mode, Delay 5us enable falsh*/
/**
  * @}
  */

/** @defgroup PWR_LL_EC_GPIO_BIT GPIO BIT
  * @{
  */

/**
  * @}
  */

/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Macros PWR Exported Macros
  * @{
  */

/** @defgroup PWR_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in PWR register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_PWR_WriteReg(__REG__, __VALUE__) WRITE_REG(PWR->__REG__, (__VALUE__))

/**
  * @brief  Read a value in PWR register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_PWR_ReadReg(__REG__) READ_REG(PWR->__REG__)
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup PWR_LL_Exported_Functions PWR Exported Functions
  * @{
  */

/** @defgroup PWR_LL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Set the Low power regulator mode.
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_PWR_LPR_MODE_MR
  *         @arg @ref LL_PWR_LPR_MODE_LPR
  *         @arg @ref LL_PWR_LPR_MODE_DLPR
  * @note   Depending on devices and packages, Deep Low Power Run mode may not be available.
  *         Refer to device datasheet for Deep Low Power Run mode availability.
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetLprMode(uint32_t Mode)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_LPR, Mode);
}

/**
  * @brief  Get the Low power regulator mode.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_LPR_MODE_MR
  *         @arg @ref LL_PWR_LPR_MODE_LPR
  *         @arg @ref LL_PWR_LPR_MODE_DLPR
  */
__STATIC_INLINE uint32_t LL_PWR_GetLprMode(void)
{
  return (READ_BIT(PWR->CR1, PWR_CR1_LPR));
}

/**
  * @brief  Enable access to the backup domain
  * @rmtoll CR1          DBP           LL_PWR_EnableBkUpAccess
  * @retval None
  */
__STATIC_INLINE void LL_PWR_EnableBkUpAccess(void)
{
  SET_BIT(PWR->CR1, PWR_CR1_DBP);
}

/**
  * @brief  Disable access to the backup domain
  * @rmtoll CR1          DBP           LL_PWR_DisableBkUpAccess
  * @retval None
  */
__STATIC_INLINE void LL_PWR_DisableBkUpAccess(void)
{
  CLEAR_BIT(PWR->CR1, PWR_CR1_DBP);
}

/**
  * @brief  Check if the backup domain is enabled
  * @rmtoll CR1          DBP           LL_PWR_IsEnabledBkUpAccess
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_PWR_IsEnabledBkUpAccess(void)
{
  return (READ_BIT(PWR->CR1, PWR_CR1_DBP) == (PWR_CR1_DBP));
}

/**
  * @brief  Set the HSI turn on mode after wake up 
  * @rmtoll CR1          HSION_CTRL          LL_PWR_SetWakeUpHSIOnMode
  * @param  HsiOnMode This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_HSION_AFTER_MR
  *         @arg @ref LL_PWR_WAKEUP_HSION_IMMEDIATE
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetWakeUpHSIOnMode(uint32_t HsiOnMode)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_HSION_CTRL, HsiOnMode);
}

/**
  * @brief  Get the HSI turn on mode after wake up
  * @rmtoll CR1          HSION_CTRL          LL_PWR_GetWakeUpHSIOnMode
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_HSION_AFTER_MR
  *         @arg @ref LL_PWR_WAKEUP_HSION_IMMEDIATE
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpHSIOnMode(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_HSION_CTRL));
}

/**
  * @brief  Set the SRAM retention voltage in stop mode
  * @rmtoll CR1          SRAM_RETV          LL_PWR_SetSramRetentionVolt
  * @param  RetentionVolt This parameter can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_NORMAL
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_STOP
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_DLP
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_HIBERNATE
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetSramRetentionVolt(uint32_t RetentionVolt)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_SRAM_RETV_CTRL, RetentionVolt);
}

/**
  * @brief  Get the SRAM retention voltage in stop mode
  * @rmtoll CR1          SRAM_RETV          LL_PWR_GetSramRetentionVolt
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_NORMAL
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_STOP
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_DLP
  *         @arg @ref LL_PWR_SRAM_RETENTION_VOLT_HIBERNATE
  */
__STATIC_INLINE uint32_t LL_PWR_GetSramRetentionVolt(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_SRAM_RETV_CTRL));
}

/**
  * @brief  Set the flash delay time after wake up 
  * @rmtoll CR1          FLS_SLPTIME          LL_PWR_SetWakeUpFlashDelay
  * @param  FlashDelay This parameter can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_0US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_2US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_3US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_5US
  * @retval None
  */
__STATIC_INLINE void LL_PWR_SetWakeUpFlashDelay(uint32_t FlashDelay)
{
  MODIFY_REG(PWR->CR1, PWR_CR1_FLS_SLPTIME, FlashDelay);
}

/**
  * @brief  Get the flash delay time after wake up 
  * @rmtoll CR1          FLS_SLPTIME          LL_PWR_GetWakeUpFlashDelay
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_0US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_2US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_3US
  *         @arg @ref LL_PWR_WAKEUP_FLASH_DELAY_5US
  */
__STATIC_INLINE uint32_t LL_PWR_GetWakeUpFlashDelay(void)
{
  return (uint32_t)(READ_BIT(PWR->CR1, PWR_CR1_FLS_SLPTIME));
}

/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup PWR_LL_EF_Init De-initialization function
  * @{
  */
ErrorStatus LL_PWR_DeInit(void);
/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(PWR) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32T020_LL_PWR_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

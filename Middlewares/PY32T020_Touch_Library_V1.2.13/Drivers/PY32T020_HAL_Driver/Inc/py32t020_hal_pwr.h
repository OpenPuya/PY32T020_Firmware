/**
  ******************************************************************************
  * @file    py32t020_hal_pwr.h
  * @author  MCU Application Team
  * @brief   Header file of PWR HAL module.
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
#ifndef __PY32T020_HAL_PWR_H
#define __PY32T020_HAL_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32t020_hal_def.h"

/** @addtogroup PY32T020_HAL_Driver
  * @{
  */

/** @defgroup PWR PWR
  * @brief PWR HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PWR_Exported_Types PWR Exported Types
  * @{
  */

/**
  * @brief  PWR Stop configuration structure definition
  */
typedef struct
{
  uint32_t WakeUpHsiEnableTime;     /*!< WakeUpHsiEnableTime: Set the flash delay time after wake up.
                                         This parameter can be a value of @ref PWR_STOP_WakeUp_HSIEN_Timing. */    
  uint32_t SramRetentionVolt;       /*!< SramRetentionVolt: Set the SRAM retention voltage in stop mode.
                                         This parameter can be a value of @ref PWR_SRAM_RETENTIONE_VOLTAGE_CONTROL. */
  uint32_t FlashDelay;              /*!< FlsahDelay: Set the flash delay time after wake up.
                                         This parameter can be a value of @ref PWR_STOP_WakeUp_Flash_Delay. */
} PWR_StopModeConfigTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_Regulator_state_in_SLEEP_STOP_mode  PWR regulator mode
  * @{
  */
#define PWR_MAINREGULATOR_ON                (0x00000000u)    /*!< Regulator in main mode      */
#define PWR_LOWPOWERREGULATOR_ON            PWR_CR1_LPR_0    /*!< Regulator in low-power mode */
#define PWR_DEEPLOWPOWERREGULATOR_ON        PWR_CR1_LPR_1    /*!< Regulator in deep-low-power mode */

/**
  * @}
  */

/** @defgroup PWR_SLEEP_mode_entry  PWR SLEEP mode entry
  * @{
  */
#define PWR_SLEEPENTRY_WFI                  ((uint8_t)0x01u)        /*!< Wait For Interruption instruction to enter Sleep mode */
#define PWR_SLEEPENTRY_WFE                  ((uint8_t)0x02u)        /*!< Wait For Event instruction to enter Sleep mode        */
/**
  * @}
  */

/** @defgroup PWR_STOP_mode_entry  PWR STOP mode entry
  * @{
  */
#define PWR_STOPENTRY_WFI                   ((uint8_t)0x01u)        /*!< Wait For Interruption instruction to enter Stop mode */
#define PWR_STOPENTRY_WFE                   ((uint8_t)0x02u)        /*!< Wait For Event instruction to enter Stop mode        */
/**
  * @}
  */

/** @defgroup PWR_STOP_WakeUp_HSIEN_Timing  PWR STOP mode WakeUp HSI Enable Timing.
  * @{
  */
#define PWR_WAKEUP_HSIEN_AFTER_MR       0x00000000U         /* Wake up from the STOP mode, After the MR becomes stable, enable HSI */
#define PWR_WAKEUP_HSIEN_IMMEDIATE      PWR_CR1_HSION_CTRL  /* Wake up from the STOP mode, Enable HSI immediately */
/**
  * @}
  */

/** @defgroup PWR_SRAM_RETENTIONE_VOLTAGE_CONTROL SRAM RETENTIONE VOLTAGE CONTROL
  * @{
  */
#define PWR_SRAM_RETENTION_VOLT_NORMAL     0x00000000U                                           /* Set SRAM voltage in normal mode */
#define PWR_SRAM_RETENTION_VOLT_STOP       (PWR_CR1_SRAM_RETV_CTRL_1                           ) /* Set SRAM voltage in stop mode */
#define PWR_SRAM_RETENTION_VOLT_DLP        (PWR_CR1_SRAM_RETV_CTRL_1                           ) /* Set SRAM voltage in deep lowpower mode */
#define PWR_SRAM_RETENTION_VOLT_HIBERNATE  (PWR_CR1_SRAM_RETV_CTRL_1 | PWR_CR1_SRAM_RETV_CTRL_0) /* Set SRAM voltage in hibernate mode */
/**
  * @}
  */

/** @defgroup PWR_STOP_WakeUp_Flash_Delay  PWR STOP WakeUp Flash Delay.
  * @{
  */
#define PWR_WAKEUP_FLASH_DELAY_5US      0x00000000U                                     /* Wake up from the STOP mode, Delay 5us enable flash*/
#define PWR_WAKEUP_FLASH_DELAY_2US      (                        PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, Delay 2us enable flash*/
#define PWR_WAKEUP_FLASH_DELAY_3US      (PWR_CR1_FLS_SLPTIME_1                        ) /* Wake up from the STOP mode, Delay 3us enable flash*/
#define PWR_WAKEUP_FLASH_DELAY_0US      (PWR_CR1_FLS_SLPTIME_1 | PWR_CR1_FLS_SLPTIME_0) /* Wake up from the STOP mode, From Sram*/

/**
  * @}
  */

/**
  * @}
  */

/* Private macros --------------------------------------------------------*/
/** @defgroup PWR_Private_Macros  PWR Private Macros
  * @{
  */
#define IS_PWR_REGULATOR(REGULATOR)               (((REGULATOR) == PWR_MAINREGULATOR_ON)     || \
                                                   ((REGULATOR) == PWR_LOWPOWERREGULATOR_ON) || \
                                                   ((REGULATOR) == PWR_DEEPLOWPOWERREGULATOR_ON))

#define IS_PWR_SLEEP_ENTRY(ENTRY)                 (((ENTRY) == PWR_SLEEPENTRY_WFI) || \
                                                   ((ENTRY) == PWR_SLEEPENTRY_WFE))

#define IS_PWR_STOP_ENTRY(ENTRY)                  (((ENTRY) == PWR_STOPENTRY_WFI) || \
                                                   ((ENTRY) == PWR_STOPENTRY_WFE))

#define IS_PWR_WAKEUP_HSIEN_TIMING(TIMING)        (((TIMING) == PWR_WAKEUP_HSIEN_AFTER_MR) || \
                                                   ((TIMING) == PWR_WAKEUP_HSIEN_IMMEDIATE))

#define IS_PWR_SRAM_RETENTION_VOLT(VOLT)          (((VOLT) == PWR_SRAM_RETENTION_VOLT_NORMAL)    || \
                                                   ((VOLT) == PWR_SRAM_RETENTION_VOLT_STOP)      || \
                                                   ((VOLT) == PWR_SRAM_RETENTION_VOLT_DLP)       || \
                                                   ((VOLT) == PWR_SRAM_RETENTION_VOLT_HIBERNATE))

#define IS_PWR_WAKEUP_FLASH_DELAY(DELAY)          (((DELAY) == PWR_WAKEUP_FLASH_DELAY_5US) || \
                                                   ((DELAY) == PWR_WAKEUP_FLASH_DELAY_2US) || \
                                                   ((DELAY) == PWR_WAKEUP_FLASH_DELAY_3US) || \
                                                   ((DELAY) == PWR_WAKEUP_FLASH_DELAY_0US))
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PWR_Exported_Functions  PWR Exported Functions
  * @{
  */

/** @defgroup PWR_Exported_Functions_Group1  Initialization and de-initialization functions
  * @{
  */
/* Initialization and de-initialization functions *******************************/
void              HAL_PWR_DeInit(void);
/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group2  Peripheral Control functions
  * @{
  */
/* Peripheral Control functions  ************************************************/
void              HAL_PWR_EnableBkUpAccess(void);
void              HAL_PWR_DisableBkUpAccess(void);
HAL_StatusTypeDef HAL_PWR_ConfigStopMode(PWR_StopModeConfigTypeDef *sStopModeConfig);

/* Low Power modes configuration functions ************************************/
void              HAL_PWR_EnterSLEEPMode(uint8_t SLEEPEntry);
void              HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void              HAL_PWR_EnableSleepOnExit(void);
void              HAL_PWR_DisableSleepOnExit(void);
void              HAL_PWR_EnableSEVOnPend(void);
void              HAL_PWR_DisableSEVOnPend(void);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32T020_HAL_PWR_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

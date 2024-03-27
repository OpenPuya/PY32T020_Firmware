/**
  ******************************************************************************
  * @file    py32t0xx_hal.h
  * @author  MCU Application Team
  * @brief   This file contains all the functions prototypes for the HAL
  *          module driver.
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
#ifndef __PY32T020_HAL_H
#define __PY32T020_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32t020_hal_conf.h"

/** @addtogroup PY32T020_HAL_Driver
  * @{
  */

/** @addtogroup HAL
  * @{
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup HAL_Exported_Constants HAL Exported Constants
  * @{
  */

/** @defgroup HAL_TICK_FREQ Tick Frequency
  * @{
  */
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/** @addtogroup HAL_Exported_Variables
  *@{
  */
extern uint32_t uwTickPrio;
extern uint32_t uwTickFreq;
/**
  * @}
  */

/** @defgroup SYSCFG_Exported_Constants SYSCFG Exported Constants
  * @{
  */

/** @defgroup SYSCFG_BootMode Boot Mode
  * @{
  */
#define SYSCFG_BOOT_MAINFLASH          0x00000000U                      /*!< Main Flash memory mapped at 0x0000 0000   */
#define SYSCFG_BOOT_SYSTEMFLASH        SYSCFG_CFGR1_MEM_MODE_0          /*!< System Flash memory mapped at 0x0000 0000 */
#define SYSCFG_BOOT_SRAM               (SYSCFG_CFGR1_MEM_MODE_1 | SYSCFG_CFGR1_MEM_MODE_0)  /*!< Embedded SRAM mapped at 0x0000 0000 */
/**
  * @}
  */

/** @defgroup SYSTEM_CH1_SRC TIM1 CH1 SOURCE
  * @{
  */
#define SYSCFG_CH1_SRC_TIM1_GPIO          0x00000000U
#define SYSCFG_CH1_SRC_TIM1_COMP1         SYSCFG_CFGR1_TIM1_IC1_SRC_0
#define SYSCFG_CH1_SRC_TIM1_COMP2         SYSCFG_CFGR1_TIM1_IC1_SRC_1
/**
  * @}
  */

/** @defgroup SYSCFG_CFGR1_ETR_SRC_TIM1 TIM1 ETR SRC
  * @{
  */

#define SYSCFG_ETR_SRC_TIM1_GPIO               0x00000000
#define SYSCFG_ETR_SRC_TIM1_COMP1              SYSCFG_CFGR1_ETR_SRC_TIM1_0
#define SYSCFG_ETR_SRC_TIM1_COMP2              SYSCFG_CFGR1_ETR_SRC_TIM1_1
#define SYSCFG_ETR_SRC_TIM1_ADC                (SYSCFG_CFGR1_ETR_SRC_TIM1_1 | SYSCFG_CFGR1_ETR_SRC_TIM1_0)
/**
  * @}
  */

/** @defgroup SYSCFG_ENSEG SEG SIGNAL 
  * @{
  */
#define SYSCFG_ENSEG_PA0         SYSCFG_IOCFG_PA_ENSEG_0
#define SYSCFG_ENSEG_PA1         SYSCFG_IOCFG_PA_ENSEG_1
#define SYSCFG_ENSEG_PA5         SYSCFG_IOCFG_PA_ENSEG_2
#define SYSCFG_ENSEG_PA6         SYSCFG_IOCFG_PA_ENSEG_3
#define SYSCFG_ENSEG_PA7         SYSCFG_IOCFG_PA_ENSEG_4
#define SYSCFG_ENSEG_PA8         SYSCFG_IOCFG_PA_ENSEG_5
#define SYSCFG_ENSEG_PB2         SYSCFG_IOCFG_PB_ENSEG_0
#define SYSCFG_ENSEG_PB3         SYSCFG_IOCFG_PB_ENSEG_1
/**
  * @}
  */

/** @defgroup SYSCFG_IODRVP I2C VCC SIGNAL 
  * @{
  */
#define SYSCFG_IODRVP_PA2        SYSCFG_IOCFG_PA_IODRVP_0
#define SYSCFG_IODRVP_PA7        SYSCFG_IOCFG_PA_IODRVP_1
/**
  * @}
  */

/** @defgroup SYSCFG_EIIC I2C PIN SIGNAL 
  * @{
  */
#define SYSCFG_EIIC_PA8          SYSCFG_IOCFG_PA_EIIC_0
#define SYSCFG_EIIC_PA11         SYSCFG_IOCFG_PA_EIIC_1
#define SYSCFG_EIIC_PF2          SYSCFG_IOCFG_PF_EIIC_0
#define SYSCFG_EIIC_PF3          SYSCFG_IOCFG_PF_EIIC_1
#define SYSCFG_EIIC_PF4          SYSCFG_IOCFG_PF_EIIC_2
/**
  * @}
  */

/** @defgroup SYSCFG_EHS LED PIN HIGH DRIVER SIGNAL 
  * @{
  */
#define SYSCFG_EHS_PA11          SYSCFG_IOCFG_PA_EHS_0
#define SYSCFG_EHS_PA12          SYSCFG_IOCFG_PA_EHS_1
#define SYSCFG_EHS_PA13          SYSCFG_IOCFG_PA_EHS_2
#define SYSCFG_EHS_PA14          SYSCFG_IOCFG_PA_EHS_3
#define SYSCFG_EHS_PA15          SYSCFG_IOCFG_PA_EHS_4
/**
  * @}
  */

/** @defgroup SYSCFG_PU_IIC I2C PIN PULL-UP 
  * @{
  */
#define SYSCFG_PU_IIC_PF3         SYSCFG_IOCFG_PF_PU_IIC_0
#define SYSCFG_PU_IIC_PF4         SYSCFG_IOCFG_PF_PU_IIC_1
/**
  * @}
  */

#define SYSCFG_IOCFG_MASK        (0x00FF7FDFu) /* PIN mask for assert test */

/** @defgroup SYSCFG_Px_IORP (SYSCFG_Px_IORP where x can be A,B,F)
  * @{
  */
#define SYSCFG_GPIO_IORP_OPEN                 0x00000000
#define SYSCFG_GPIO_IORP_10K                  SYSCFG_PA_IORP_PA0_IORP_0
#define SYSCFG_GPIO_IORP_20K                  SYSCFG_PA_IORP_PA0_IORP_1
#define SYSCFG_GPIO_IORP_40K                  (SYSCFG_PA_IORP_PA0_IORP_0 | SYSCFG_PA_IORP_PA0_IORP_1)
/**
  * @}
  */
  
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup HAL_Exported_Macros HAL Exported Macros
  * @{
  */

/** @defgroup DBGMCU_Freeze_Unfreeze Freeze Unfreeze Peripherals in Debug mode
  * @brief   Freeze/Unfreeze Peripherals in Debug mode
  * Note:
  *       Debug registers DBGMCU_IDCODE and DBGMCU_CR are accessible only in
  *       debug mode (not accessible by the user software in normal mode).
  *       Refer to errata sheet of these devices for more details.
  * @{
  */
  
/* Peripherals on APB1 */

#if defined(DBGMCU_APB_FZ1_DBG_IWDG_STOP)
#define __HAL_DBGMCU_FREEZE_IWDG()           (DBGMCU->APBFZ1 |= (DBGMCU_APB_FZ1_DBG_IWDG_STOP))
#define __HAL_DBGMCU_UNFREEZE_IWDG()         (DBGMCU->APBFZ1 &= ~(DBGMCU_APB_FZ1_DBG_IWDG_STOP))
#endif /* DBGMCU_APB_FZ1_DBG_IWDG_STOP */

#if defined(DBGMCU_APB_FZ1_DBG_RTC_STOP)
#define __HAL_DBGMCU_FREEZE_RTC()            (DBGMCU->APBFZ1 |= (DBGMCU_APB_FZ1_DBG_RTC_STOP))
#define __HAL_DBGMCU_UNFREEZE_RTC()          (DBGMCU->APBFZ1 &= ~(DBGMCU_APB_FZ1_DBG_RTC_STOP))
#endif /* DBGMCU_APB_FZ1_DBG_RTC_STOP */

#if defined(DBGMCU_APB_FZ2_DBG_TIM1_STOP)
#define __HAL_DBGMCU_FREEZE_TIM1()           (DBGMCU->APBFZ2 |= (DBGMCU_APB_FZ2_DBG_TIM1_STOP))
#define __HAL_DBGMCU_UNFREEZE_TIM1()         (DBGMCU->APBFZ2 &= ~(DBGMCU_APB_FZ2_DBG_TIM1_STOP))
#endif /* DBGMCU_APB_FZ2_DBG_TIM1_STOP */

#if defined(DBGMCU_APB_FZ2_DBG_TIM14_STOP)
#define __HAL_DBGMCU_FREEZE_TIM14()          (DBGMCU->APBFZ2 |= (DBGMCU_APB_FZ2_DBG_TIM14_STOP))
#define __HAL_DBGMCU_UNFREEZE_TIM14()        (DBGMCU->APBFZ2 &= ~(DBGMCU_APB_FZ2_DBG_TIM14_STOP))
#endif /* DBGMCU_APB_FZ2_DBG_TIM14_STOP */
/**
  * @}
  */



/** @defgroup SYSCFG_Exported_Macros SYSCFG Exported Macros
  * @{
  */

/** @brief  Enable or disable COMP1 output as TIM1 ocref_Clr input.
  */
#define __HAL_SYSCFG_COMP1_OCREF_CLR_TIM1_ENABLE()           SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1)
#define __HAL_SYSCFG_COMP1_OCREF_CLR_TIM1_DISABLE()          CLEAR_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1)

/** @brief  Enable or disable COMP2 output as TIM1 ocref_Clr input.
  */
#define __HAL_SYSCFG_COMP2_OCREF_CLR_TIM1_ENABLE()           SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1)
#define __HAL_SYSCFG_COMP2_OCREF_CLR_TIM1_DISABLE()          CLEAR_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1)

/** @brief  SYSCFG Break Cortex-M0+ Lockup lock.
  *         Enables and locks the connection of Cortex-M0+ LOCKUP (Hardfault) output to TIM1 Break input
  * @note   The selected configuration is locked and can be unlocked only by system reset.
  */
#define __HAL_SYSCFG_BREAK_LOCKUP_LOCK()        SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_LOCKUP_LOCK)

#if defined(SYSCFG_CFGR2_COMP1_BRK_TIM1)
/** @brief  COMP1 as Timer1 Break input
  */
#define __HAL_SYSCFG_COMP1_BREAK_TIM1()           SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP1_BRK_TIM1)
#endif

#if defined(SYSCFG_CFGR2_COMP2_BRK_TIM1)
/** @brief  COMP2 as Timer1 Break input
  */
#define __HAL_SYSCFG_COMP2_BREAK_TIM1()           SET_BIT(SYSCFG->CFGR2, SYSCFG_CFGR2_COMP2_BRK_TIM1)
#endif
/**
  * @}
  */

/**
  * @}
  */

/* Private Macros -------------------------------------------------------------*/

/** @defgroup HAL_Private_Macros HAL Private Macros
  * @{
  */
#define IS_TICKFREQ(FREQ) (((FREQ) == HAL_TICK_FREQ_10HZ)  || \
                           ((FREQ) == HAL_TICK_FREQ_100HZ) || \
                           ((FREQ) == HAL_TICK_FREQ_1KHZ))
                           
#define IS_GPIO_IORP(IORP) (((IORP) == SYSCFG_GPIO_IORP_OPEN) || \
                            ((IORP) == SYSCFG_GPIO_IORP_10K)  || \
                            ((IORP) == SYSCFG_GPIO_IORP_20K)  || \
                            ((IORP) == SYSCFG_GPIO_IORP_40K))
                            
#define IS_SYSCFG_GPIO_PIN(PORTPIN) ((((PORTPIN) & SYSCFG_IOCFG_MASK)) != 0x00u || \
                                     (((PORTPIN) & ~SYSCFG_IOCFG_MASK) == 0x00u))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup HAL_Exported_Functions HAL Exported Functions
  * @{
  */

/** @addtogroup HAL_Exported_Functions_Group1
  * @{
  */

/* Initialization and de-initialization functions  ******************************/
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
/**
  * @}
  */

/** @addtogroup HAL_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  ************************************************/
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(uint32_t Freq);
uint32_t HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetDEVID(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
/**
  * @}
  */

/** @addtogroup HAL_Exported_Functions_Group3
  * @{
  */
/* HAL Debug functions  *********************************************************/
void HAL_DBGMCU_EnableDBGMCUSleepMode(void);
void HAL_DBGMCU_DisableDBGMCUSleepMode(void);
void HAL_DBGMCU_EnableDBGMCUStopMode(void);
void HAL_DBGMCU_DisableDBGMCUStopMode(void);
/**
  * @}
  */

/** @addtogroup HAL_Exported_Functions_Group4
  * @{
  */
/* SYSCFG configuration functions  **********************************************/
void HAL_SYSCFG_SetRemapMemory(uint32_t Memory);
uint32_t HAL_SYSCFG_GetRemapMemory(void);

void HAL_SYSCFG_SetTIM1CH1Source(uint32_t Source);
uint32_t HAL_SYSCFG_GetTIM1CH1Source(void);

void HAL_SYSCFG_TIM1ETRSource(uint32_t ETRSource);

void HAL_SYSCFG_EnableGPIONoiseFilter(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
void HAL_SYSCFG_DisableGPIONoiseFilter(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

void HAL_SYSCFG_EnableGPIOAnalog2(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
void HAL_SYSCFG_DisableGPIOAnalog2(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

void HAL_SYSCFG_EnableI2CPinSignal(uint32_t PORT_Pin);
void HAL_SYSCFG_DisableI2CPinSignal(uint32_t PORT_Pin);

void HAL_SYSCFG_EnableLEDPinHighDrvSignal(uint32_t PORT_Pin);
void HAL_SYSCFG_DisableLEDPinHighDrvSignal(uint32_t PORT_Pin);

void HAL_SYSCFG_EnableI2CPinPullUp(uint32_t PORT_Pin);
void HAL_SYSCFG_DisableI2CPinPullUp(uint32_t PORT_Pin);

void HAL_SYSCFG_EnableI2CVccIoDrvSignal(uint32_t PORT_Pin);
void HAL_SYSCFG_DisableI2CVccIoDrvSignal(uint32_t PORT_Pin);

void HAL_SYSCFG_EnableEnsegSignal(uint32_t PORT_Pin);
void HAL_SYSCFG_DisableEnsegSignal(uint32_t PORT_Pin);

void HAL_SYSCFG_SetGpioPullResistanceValue(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,uint16_t UpDown_Level);
/**
  * @}
  */

/**
  * @}
  */


/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup HAL_Private_Variables HAL Private Variables
  * @{
  */
/**
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/** @defgroup HAL_Private_Constants HAL Private Constants
  * @{
  */
/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32T020_HAL_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

/**
  ******************************************************************************
  * @file    py32t020_ll_system.h
  * @author  MCU Application Team
  * @brief   Header file of SYSTEM LL module.
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
#ifndef __PY32T020_LL_SYSTEM_H
#define __PY32T020_LL_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32t0xx.h"

/** @addtogroup PY32T020_LL_Driver
  * @{
  */

#if defined (FLASH) || defined (SYSCFG) || defined (DBGMCU)

/** @defgroup SYSTEM_LL SYSTEM
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup SYSTEM_LL_Exported_Constants SYSTEM Exported Constants
  * @{
  */

/** @defgroup SYSTEM_LL_EC_REMAP SYSCFG REMAP
  * @{
  */
#define LL_SYSCFG_REMAP_FLASH               0x00000000U                                           /*!< Main Flash memory mapped at 0x00000000 */
#define LL_SYSCFG_REMAP_SYSTEMFLASH         SYSCFG_CFGR1_MEM_MODE_0                               /*!< System Flash memory mapped at 0x00000000 */
#define LL_SYSCFG_REMAP_SRAM                (SYSCFG_CFGR1_MEM_MODE_1 | SYSCFG_CFGR1_MEM_MODE_0)   /*!< Embedded SRAM mapped at 0x00000000 */
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_OCREF_CLR SYSCFG TIM1 OCREF SOURCE  
  * @{
  */
#if defined(SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1)
#define LL_SYSCFG_OCREF_CLR_TIM1_COMP1         SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1
#endif
#if defined(SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1)
#define LL_SYSCFG_OCREF_CLR_TIM1_COMP2         SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1
#endif
/**
  * @}
  */


/** @defgroup SYSTEM_LL_EC_CH1_SRC SYSCFG TIM1 CH1 SRC
  * @{
  */
#define LL_SYSCFG_CH1_SRC_TIM1_GPIO          0x00000000U
#if defined(COMP1)
#define LL_SYSCFG_CH1_SRC_TIM1_COMP1         SYSCFG_CFGR1_TIM1_IC1_SRC_0
#endif
#if defined(COMP2)
#define LL_SYSCFG_CH1_SRC_TIM1_COMP2         SYSCFG_CFGR1_TIM1_IC1_SRC_1
#endif
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_TIMBREAK SYSCFG TIMER BREAK INPUT
  * @{
  */
#if defined(SYSCFG_CFGR2_LOCKUP_LOCK)
#define LL_SYSCFG_TIMBREAK_LOCKUP_TO_ALL      SYSCFG_CFGR2_LOCKUP_LOCK
#endif
#if defined(SYSCFG_CFGR2_COMP1_BRK_TIM1)
#define LL_SYSCFG_TIMBREAK_COMP1_TO_TIM1      SYSCFG_CFGR2_COMP1_BRK_TIM1
#endif
#if defined(SYSCFG_CFGR2_COMP2_BRK_TIM1)
#define LL_SYSCFG_TIMBREAK_COMP2_TO_TIM1      SYSCFG_CFGR2_COMP2_BRK_TIM1
#endif
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_ETR_SRC SYSCFG TIM1 ETR SOURCE
  * @{
  */
#define LL_SYSCFG_ETR_SRC_TIM1_GPIO          0x00000000U
#if defined(COMP1)
#define LL_SYSCFG_ETR_SRC_TIM1_COMP1         SYSCFG_CFGR1_ETR_SRC_TIM1_0
#endif
#if defined(COMP2)
#define LL_SYSCFG_ETR_SRC_TIM1_COMP2         SYSCFG_CFGR1_ETR_SRC_TIM1_1
#endif
#if defined(ADC)
#define LL_SYSCFG_ETR_SRC_TIM1_ADC           (SYSCFG_CFGR1_ETR_SRC_TIM1_0 | SYSCFG_CFGR1_ETR_SRC_TIM1_1)
#endif
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_GPIO_PORT SYSCFG GPIO PORT
  * @{
  */
#define LL_SYSCFG_GPIO_PORTA              0x00000000U
#define LL_SYSCFG_GPIO_PORTB              0x00000008U
#define LL_SYSCFG_GPIO_PORTF              0x00000010U 
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_GPIO_PIN SYSCFG GPIO PIN
  * @{
  */
#define LL_SYSCFG_GPIO_PIN_0              0x00000001U
#define LL_SYSCFG_GPIO_PIN_1              0x00000002U
#define LL_SYSCFG_GPIO_PIN_2              0x00000004U
#define LL_SYSCFG_GPIO_PIN_3              0x00000008U
#define LL_SYSCFG_GPIO_PIN_4              0x00000010U
#define LL_SYSCFG_GPIO_PIN_5              0x00000020U
#define LL_SYSCFG_GPIO_PIN_6              0x00000040U
#define LL_SYSCFG_GPIO_PIN_7              0x00000080U
#define LL_SYSCFG_GPIO_PIN_8              0x00000100U  
#define LL_SYSCFG_GPIO_PIN_9              0x00000200U
#define LL_SYSCFG_GPIO_PIN_10             0x00000400U
#define LL_SYSCFG_GPIO_PIN_11             0x00000800U
#define LL_SYSCFG_GPIO_PIN_12             0x00001000U
#define LL_SYSCFG_GPIO_PIN_13             0x00002000U
#define LL_SYSCFG_GPIO_PIN_14             0x00004000U
#define LL_SYSCFG_GPIO_PIN_15             0x00008000U
/**
  * @}
  */
  
/** @defgroup SYSCFG_ENSEG SEG SIGNAL 
  * @{
  */
#define LL_SYSCFG_ENSEG_PA0         SYSCFG_IOCFG_PA_ENSEG_0
#define LL_SYSCFG_ENSEG_PA1         SYSCFG_IOCFG_PA_ENSEG_1
#define LL_SYSCFG_ENSEG_PA5         SYSCFG_IOCFG_PA_ENSEG_2
#define LL_SYSCFG_ENSEG_PA6         SYSCFG_IOCFG_PA_ENSEG_3
#define LL_SYSCFG_ENSEG_PA7         SYSCFG_IOCFG_PA_ENSEG_4
#define LL_SYSCFG_ENSEG_PA8         SYSCFG_IOCFG_PA_ENSEG_5
#define LL_SYSCFG_ENSEG_PB2         SYSCFG_IOCFG_PB_ENSEG_0
#define LL_SYSCFG_ENSEG_PB3         SYSCFG_IOCFG_PB_ENSEG_0
/**
  * @}
  */

/** @defgroup SYSCFG_IODRVP I2C VCC SIGNAL 
  * @{
  */
#define LL_SYSCFG_IODRVP_PA2        SYSCFG_IOCFG_PA_IODRVP_0
#define LL_SYSCFG_IODRVP_PA7        SYSCFG_IOCFG_PA_IODRVP_1
/**
  * @}
  */

/** @defgroup SYSCFG_EIIC I2C PIN SIGNAL 
  * @{
  */
#define LL_SYSCFG_EIIC_PA8          SYSCFG_IOCFG_PA_EIIC_0
#define LL_SYSCFG_EIIC_PA11         SYSCFG_IOCFG_PA_EIIC_1
#define LL_SYSCFG_EIIC_PF2          SYSCFG_IOCFG_PF_EIIC_0
#define LL_SYSCFG_EIIC_PF3          SYSCFG_IOCFG_PF_EIIC_1
#define LL_SYSCFG_EIIC_PF4          SYSCFG_IOCFG_PF_EIIC_2
/**
  * @}
  */

/** @defgroup SYSCFG_EHS LED PIN HIGH DRIVER SIGNAL 
  * @{
  */
#define LL_SYSCFG_EHS_PA11          SYSCFG_IOCFG_PA_EHS_0
#define LL_SYSCFG_EHS_PA12          SYSCFG_IOCFG_PA_EHS_1
#define LL_SYSCFG_EHS_PA13          SYSCFG_IOCFG_PA_EHS_2
#define LL_SYSCFG_EHS_PA14          SYSCFG_IOCFG_PA_EHS_3
#define LL_SYSCFG_EHS_PA15          SYSCFG_IOCFG_PA_EHS_4
/**
  * @}
  */

/** @defgroup SYSCFG_PU_IIC I2C PIN PULL-UP 
  * @{
  */
#define LL_SYSCFG_PU_IIC_PF3         SYSCFG_IOCFG_PF_PU_IIC_0
#define LL_SYSCFG_PU_IIC_PF4         SYSCFG_IOCFG_PF_PU_IIC_1
/**
  * @}
  */

/** @defgroup SYSCFG_LL_Px_IORP (SYSCFG_LL_Px_IORP where x can be A,B,F) 
  * @{
  */
#define LL_SYSCFG_GPIO_IORP_OPEN          0x00000000
#define LL_SYSCFG_GPIO_IORP_10K           SYSCFG_PA_IORP_PA0_IORP_0
#define LL_SYSCFG_GPIO_IORP_20K           SYSCFG_PA_IORP_PA0_IORP_1
#define LL_SYSCFG_GPIO_IORP_40K           (SYSCFG_PA_IORP_PA0_IORP_0 | SYSCFG_PA_IORP_PA0_IORP_1)
/**
  * @}
  */
  
/** @defgroup SYSTEM_LL_EC_LATENCY FLASH LATENCY
  * @{
  */
#define LL_FLASH_LATENCY_0                 0x00000000U               /*!< FLASH Zero Latency cycle */
#if defined(FLASH_ACR_LATENCY_0)
#define LL_FLASH_LATENCY_1                 FLASH_ACR_LATENCY_0       /*!< FLASH One Latency cycle */
#endif
#if defined(FLASH_ACR_LATENCY_1)
#define LL_FLASH_LATENCY_2                 FLASH_ACR_LATENCY_1       /*!< FLASH Three Latency cycle */ 
#endif
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_USRLOCK FLASH USRLOCK
  * @{
  */
#if defined(FLASH_SR_USRLOCK)
#define LL_FLASH_SR_USRLOCK                 FLASH_SR_USRLOCK         /*!< FLASH Read userdata flag */ 
#endif
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_APB1_GRP1_STOP_IP  DBGMCU APB1 GRP1 STOP IP
  * @{
  */
#if defined(DBGMCU_APB_FZ1_DBG_IWDG_STOP)
#define LL_DBGMCU_APB1_GRP1_IWDG_STOP      DBGMCU_APB_FZ1_DBG_IWDG_STOP        /*!< Debug Independent Watchdog stopped when Core is halted */
#endif
#if defined(DBGMCU_APB_FZ1_DBG_RTC_STOP)
#define LL_DBGMCU_APB1_GRP1_RTC_STOP       DBGMCU_APB_FZ1_DBG_RTC_STOP        /*!< RTC counter stopped when Core is halted */
#endif
/**
  * @}
  */

/** @defgroup SYSTEM_LL_EC_APB1_GRP2_STOP_IP DBGMCU APB1 GRP2 STOP IP
  * @{
  */
#if defined(DBGMCU_APB_FZ2_DBG_TIM1_STOP)
#define LL_DBGMCU_APB1_GRP2_TIM1_STOP      DBGMCU_APB_FZ2_DBG_TIM1_STOP        /*!< TIM1 counter stopped when core is halted */
#endif
#if defined(DBGMCU_APB_FZ2_DBG_TIM14_STOP)
#define LL_DBGMCU_APB1_GRP2_TIM14_STOP     DBGMCU_APB_FZ2_DBG_TIM14_STOP       /*!< TIM14 counter stopped when core is halted */
#endif
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @defgroup SYSTEM_LL_Exported_Functions SYSTEM Exported Functions
  * @{
  */

/** @defgroup SYSTEM_LL_EF_SYSCFG SYSCFG
  * @{
  */

/**
  * @brief  Set memory mapping at address 0x00000000
  * @rmtoll SYSCFG_CFGR1 MEM_MODE      LL_SYSCFG_SetRemapMemory
  * @param  Memory This parameter can be one of the following values:
  *         @arg @ref LL_SYSCFG_REMAP_FLASH
  *         @arg @ref LL_SYSCFG_REMAP_SYSTEMFLASH
  *         @arg @ref LL_SYSCFG_REMAP_SRAM
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_SetRemapMemory(uint32_t Memory)
{
  MODIFY_REG(SYSCFG->CFGR1, SYSCFG_CFGR1_MEM_MODE, Memory);
}

/**
  * @brief  Get memory mapping at address 0x00000000
  * @rmtoll SYSCFG_CFGR1 MEM_MODE      LL_SYSCFG_GetRemapMemory
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SYSCFG_REMAP_FLASH
  *         @arg @ref LL_SYSCFG_REMAP_SYSTEMFLASH
  *         @arg @ref LL_SYSCFG_REMAP_SRAM
  */
__STATIC_INLINE uint32_t LL_SYSCFG_GetRemapMemory(void)
{
  return (uint32_t)(READ_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_MEM_MODE));
}

/**
  * @brief  Set TIM1 CH1 Input Source.
  * @param  Source TIM1 CH1 Input Source.
  *          This parameter can be one of the following values:
  *            @arg LL_SYSCFG_CH1_SRC_TIM1_GPIO
  *            @arg LL_SYSCFG_CH1_SRC_TIM1_COMP1
  *            @arg LL_SYSCFG_CH1_SRC_TIM1_COMP2
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_SetTIM1CH1Source(uint32_t Source)
{
  MODIFY_REG(SYSCFG->CFGR1, SYSCFG_CFGR1_TIM1_IC1_SRC, Source);  
}

/**
  * @brief  Get TIM1 CH1 Input Source.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SYSCFG_CH1_SRC_TIM1_GPIO
  *         @arg @ref LL_SYSCFG_CH1_SRC_TIM1_COMP1
  *         @arg @ref LL_SYSCFG_CH1_SRC_TIM1_COMP2
  */
__STATIC_INLINE uint32_t LL_SYSCFG_GetTIM1CH1Source(void)
{
  return (uint32_t)(READ_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_TIM1_IC1_SRC));
}

/**
  * @brief  Enables COMPx as TIMx break input
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  TIMBreakInputs This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_TIMBREAK_LOCKUP_TO_ALL
  *         @arg @ref LL_SYSCFG_TIMBREAK_COMP1_TO_TIM1
  *         @arg @ref LL_SYSCFG_TIMBREAK_COMP2_TO_TIM1
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableTIMBreakInputs(uint32_t TIMBreakInputs)
{
  SET_BIT(SYSCFG->CFGR2, TIMBreakInputs);
}

/**
  * @brief  Disables COMPx as TIMx break input
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  TIMBreakInputs This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_TIMBREAK_LOCKUP_TO_ALL
  *         @arg @ref LL_SYSCFG_TIMBREAK_COMP1_TO_TIM1
  *         @arg @ref LL_SYSCFG_TIMBREAK_COMP2_TO_TIM1
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableTIMBreakInputs(uint32_t TIMBreakInputs)
{
  CLEAR_BIT(SYSCFG->CFGR2, TIMBreakInputs);
}

/**
  * @brief  Indicate if COMPx as TIMx break input
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  TIMBreakInputs This parameter can be one of the following values:
  *         @arg @ref LL_SYSCFG_TIMBREAK_LOCKUP_TO_ALL
  *         @arg @ref LL_SYSCFG_TIMBREAK_COMP1_TO_TIM1
  *         @arg @ref LL_SYSCFG_TIMBREAK_COMP2_TO_TIM1
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledTIMBreakInputs(uint32_t TIMBreakInputs)
{
  return ((READ_BIT(SYSCFG->CFGR2, TIMBreakInputs) == (TIMBreakInputs)) ? 1UL : 0UL);
}

/**
  * @brief  Enables COMPx as TIMx Ocref input
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  TIMOcrefInputs This parameter can be one of the following values:
  *         @arg @ref LL_SYSCFG_OCREF_CLR_TIM1_COMP1
  *         @arg @ref LL_SYSCFG_OCREF_CLR_TIM1_COMP2
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableTIMOcrefInputs(uint32_t TIMOcrefInputs)
{
  SET_BIT(SYSCFG->CFGR2, TIMOcrefInputs);
}

/**
  * @brief  Disables COMPx as TIMx Ocref input
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  TIMOcrefInputs This parameter can be one of the following values:
  *         @arg @ref LL_SYSCFG_OCREF_CLR_TIM1_COMP1
  *         @arg @ref LL_SYSCFG_OCREF_CLR_TIM1_COMP2
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableTIMOcrefInputs(uint32_t TIMOcrefInputs)
{
  CLEAR_BIT(SYSCFG->CFGR2, TIMOcrefInputs);
}

/**
  * @brief  Indicate if COMPx as TIMx Ocref input
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  TIMOcrefInputs This parameter can be one of the following values:
  *         @arg @ref LL_SYSCFG_OCREF_CLR_TIM1_COMP1
  *         @arg @ref LL_SYSCFG_OCREF_CLR_TIM1_COMP2
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledTIMOcrefInputs(uint32_t TIMOcrefInputs)
{
  return ((READ_BIT(SYSCFG->CFGR2, TIMOcrefInputs) == (TIMOcrefInputs)) ? 1UL : 0UL);
}

/**
  * @brief  Set the TIMER1 ETR input source
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  source This parameter can be one of the following values:
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_GPIO
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_COMP1
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_COMP2
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_ADC
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_SetTIM1ETRSource(uint32_t source)
{
  MODIFY_REG(SYSCFG->CFGR1, SYSCFG_CFGR1_ETR_SRC_TIM1, source); 
}

/**
  * @brief  Get the TIMER1 ETR input source
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_GPIO
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_COMP1
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_COMP2
  *         @arg @ref LL_SYSCFG_ETR_SRC_TIM1_ADC
  * @retval None
  */
__STATIC_INLINE uint32_t LL_SYSCFG_GetTIM1ETRSource(void)
{
  return (uint32_t)(READ_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_ETR_SRC_TIM1)); 
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_ENS   GPIO Filter
  * @{
  */
/**
  * @brief  Enable GPIO Filter
  * @note   Depending on devices and packages, some IOs may not be available.
  *         Refer to device datasheet for IOs availability.
  * @param  GPIOPort This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PORTA
  *         @arg @ref LL_SYSCFG_GPIO_PORTB
  *         @arg @ref LL_SYSCFG_GPIO_PORTF
  * @param  GPIOPin This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PIN_0
  *         @arg @ref LL_SYSCFG_GPIO_PIN_1
  *         @arg @ref LL_SYSCFG_GPIO_PIN_2
  *         @arg @ref LL_SYSCFG_GPIO_PIN_3
  *         @arg @ref LL_SYSCFG_GPIO_PIN_4
  *         @arg @ref LL_SYSCFG_GPIO_PIN_5
  *         @arg @ref LL_SYSCFG_GPIO_PIN_6
  *         @arg @ref LL_SYSCFG_GPIO_PIN_7
  *         @arg @ref LL_SYSCFG_GPIO_PIN_8    
  *         @arg @ref LL_SYSCFG_GPIO_PIN_9
  *         @arg @ref LL_SYSCFG_GPIO_PIN_10
  *         @arg @ref LL_SYSCFG_GPIO_PIN_11
  *         @arg @ref LL_SYSCFG_GPIO_PIN_12
  *         @arg @ref LL_SYSCFG_GPIO_PIN_13
  *         @arg @ref LL_SYSCFG_GPIO_PIN_14
  *         @arg @ref LL_SYSCFG_GPIO_PIN_15
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableGPIOFilter(uint32_t GPIOPort, uint32_t GPIOPin) 
{
  if(GPIOPort == LL_SYSCFG_GPIO_PORTA)
  {
    SET_BIT(SYSCFG->PAENS, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTB)
  {
    SET_BIT(SYSCFG->PBENS, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTF)
  {
    SET_BIT(SYSCFG->PFENS, GPIOPin);
  }
  else
  {
    
  }
}

/**
  * @brief  Disable GPIO Filter
  * @note   Depending on devices and packages, some IOs may not be available.
  *         Refer to device datasheet for IOs availability.
  * @param  GPIOPort This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PORTA
  *         @arg @ref LL_SYSCFG_GPIO_PORTB
  *         @arg @ref LL_SYSCFG_GPIO_PORTF
  * @param  GPIOPin This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PIN_0
  *         @arg @ref LL_SYSCFG_GPIO_PIN_1
  *         @arg @ref LL_SYSCFG_GPIO_PIN_2
  *         @arg @ref LL_SYSCFG_GPIO_PIN_3
  *         @arg @ref LL_SYSCFG_GPIO_PIN_4
  *         @arg @ref LL_SYSCFG_GPIO_PIN_5
  *         @arg @ref LL_SYSCFG_GPIO_PIN_6
  *         @arg @ref LL_SYSCFG_GPIO_PIN_7
  *         @arg @ref LL_SYSCFG_GPIO_PIN_8    
  *         @arg @ref LL_SYSCFG_GPIO_PIN_9
  *         @arg @ref LL_SYSCFG_GPIO_PIN_10
  *         @arg @ref LL_SYSCFG_GPIO_PIN_11
  *         @arg @ref LL_SYSCFG_GPIO_PIN_12
  *         @arg @ref LL_SYSCFG_GPIO_PIN_13
  *         @arg @ref LL_SYSCFG_GPIO_PIN_14
  *         @arg @ref LL_SYSCFG_GPIO_PIN_15
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableGPIOFilter(uint32_t GPIOPort, uint32_t GPIOPin) 
{
  if(GPIOPort == LL_SYSCFG_GPIO_PORTA)
  {
    CLEAR_BIT(SYSCFG->PAENS, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTB)
  {
    CLEAR_BIT(SYSCFG->PBENS, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTF)
  {
    CLEAR_BIT(SYSCFG->PFENS, GPIOPin);
  }
  else
  {
    
  }
}

/**
  * @brief  Indicate if enable the GPIO State in GPIO Filter
  * @param  GPIOPort This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PORTA
  *         @arg @ref LL_SYSCFG_GPIO_PORTB
  *         @arg @ref LL_SYSCFG_GPIO_PORTF
  * @param  GPIOPin This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PIN_0
  *         @arg @ref LL_SYSCFG_GPIO_PIN_1
  *         @arg @ref LL_SYSCFG_GPIO_PIN_2
  *         @arg @ref LL_SYSCFG_GPIO_PIN_3
  *         @arg @ref LL_SYSCFG_GPIO_PIN_4
  *         @arg @ref LL_SYSCFG_GPIO_PIN_5
  *         @arg @ref LL_SYSCFG_GPIO_PIN_6
  *         @arg @ref LL_SYSCFG_GPIO_PIN_7
  *         @arg @ref LL_SYSCFG_GPIO_PIN_8
  *         @arg @ref LL_SYSCFG_GPIO_PIN_9
  *         @arg @ref LL_SYSCFG_GPIO_PIN_10
  *         @arg @ref LL_SYSCFG_GPIO_PIN_11
  *         @arg @ref LL_SYSCFG_GPIO_PIN_12
  *         @arg @ref LL_SYSCFG_GPIO_PIN_13
  *         @arg @ref LL_SYSCFG_GPIO_PIN_14
  *         @arg @ref LL_SYSCFG_GPIO_PIN_15
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledGPIOFilter(uint32_t GPIOPort, uint32_t GPIOPin)
{
  if(GPIOPort == LL_SYSCFG_GPIO_PORTA)
  {
    return ((READ_BIT(SYSCFG->PAENS, GPIOPin) == (GPIOPin)) ? 1UL : 0UL);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTB)
  {
    return ((READ_BIT(SYSCFG->PBENS, GPIOPin) == (GPIOPin)) ? 1UL : 0UL);
  }
  else
  {
    return ((READ_BIT(SYSCFG->PFENS, GPIOPin) == (GPIOPin)) ? 1UL : 0UL);
  }
}


/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_ANA2EN   GPIO Analog2
  * @{
  */
/**
  * @brief  Enable GPIO Analog2
  * @note   Depending on devices and packages, some IOs may not be available.
  *         Refer to device datasheet for IOs availability.
  * @param  GPIOPort This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PORTA
  *         @arg @ref LL_SYSCFG_GPIO_PORTB
  *         @arg @ref LL_SYSCFG_GPIO_PORTF
  * @param  GPIOPin This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PIN_0
  *         @arg @ref LL_SYSCFG_GPIO_PIN_1
  *         @arg @ref LL_SYSCFG_GPIO_PIN_2
  *         @arg @ref LL_SYSCFG_GPIO_PIN_3
  *         @arg @ref LL_SYSCFG_GPIO_PIN_4
  *         @arg @ref LL_SYSCFG_GPIO_PIN_5
  *         @arg @ref LL_SYSCFG_GPIO_PIN_6
  *         @arg @ref LL_SYSCFG_GPIO_PIN_7
  *         @arg @ref LL_SYSCFG_GPIO_PIN_8    
  *         @arg @ref LL_SYSCFG_GPIO_PIN_9
  *         @arg @ref LL_SYSCFG_GPIO_PIN_10
  *         @arg @ref LL_SYSCFG_GPIO_PIN_11
  *         @arg @ref LL_SYSCFG_GPIO_PIN_12
  *         @arg @ref LL_SYSCFG_GPIO_PIN_13
  *         @arg @ref LL_SYSCFG_GPIO_PIN_14
  *         @arg @ref LL_SYSCFG_GPIO_PIN_15
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableGPIOAnalog2(uint32_t GPIOPort, uint32_t GPIOPin) 
{
  if(GPIOPort == LL_SYSCFG_GPIO_PORTA)
  {
    SET_BIT(SYSCFG->PAANA2EN, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTB)
  {
    SET_BIT(SYSCFG->PBANA2EN, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTF)
  {
    SET_BIT(SYSCFG->PFANA2EN, GPIOPin);
  }
  else
  {
    
  }
}

/**
  * @brief  Disable GPIO Filter
  * @note   Depending on devices and packages, some IOs may not be available.
  *         Refer to device datasheet for IOs availability.
  * @param  GPIOPort This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PORTA
  *         @arg @ref LL_SYSCFG_GPIO_PORTB
  *         @arg @ref LL_SYSCFG_GPIO_PORTF
  * @param  GPIOPin This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PIN_0
  *         @arg @ref LL_SYSCFG_GPIO_PIN_1
  *         @arg @ref LL_SYSCFG_GPIO_PIN_2
  *         @arg @ref LL_SYSCFG_GPIO_PIN_3
  *         @arg @ref LL_SYSCFG_GPIO_PIN_4
  *         @arg @ref LL_SYSCFG_GPIO_PIN_5
  *         @arg @ref LL_SYSCFG_GPIO_PIN_6
  *         @arg @ref LL_SYSCFG_GPIO_PIN_7
  *         @arg @ref LL_SYSCFG_GPIO_PIN_8    
  *         @arg @ref LL_SYSCFG_GPIO_PIN_9
  *         @arg @ref LL_SYSCFG_GPIO_PIN_10
  *         @arg @ref LL_SYSCFG_GPIO_PIN_11
  *         @arg @ref LL_SYSCFG_GPIO_PIN_12
  *         @arg @ref LL_SYSCFG_GPIO_PIN_13
  *         @arg @ref LL_SYSCFG_GPIO_PIN_14
  *         @arg @ref LL_SYSCFG_GPIO_PIN_15
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableGPIOAnalog2(uint32_t GPIOPort, uint32_t GPIOPin) 
{
  if(GPIOPort == LL_SYSCFG_GPIO_PORTA)
  {
    CLEAR_BIT(SYSCFG->PAANA2EN, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTB)
  {
    CLEAR_BIT(SYSCFG->PBANA2EN, GPIOPin);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTF)
  {
    CLEAR_BIT(SYSCFG->PFANA2EN, GPIOPin);
  }
  else
  {
    
  }
}

/**
  * @brief  Indicate if enable the GPIO State in GPIO Analog
  * @param  GPIOPort This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PORTA
  *         @arg @ref LL_SYSCFG_GPIO_PORTB
  *         @arg @ref LL_SYSCFG_GPIO_PORTF
  * @param  GPIOPin This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PIN_0
  *         @arg @ref LL_SYSCFG_GPIO_PIN_1
  *         @arg @ref LL_SYSCFG_GPIO_PIN_2
  *         @arg @ref LL_SYSCFG_GPIO_PIN_3
  *         @arg @ref LL_SYSCFG_GPIO_PIN_4
  *         @arg @ref LL_SYSCFG_GPIO_PIN_5
  *         @arg @ref LL_SYSCFG_GPIO_PIN_6
  *         @arg @ref LL_SYSCFG_GPIO_PIN_7
  *         @arg @ref LL_SYSCFG_GPIO_PIN_8
  *         @arg @ref LL_SYSCFG_GPIO_PIN_9
  *         @arg @ref LL_SYSCFG_GPIO_PIN_10
  *         @arg @ref LL_SYSCFG_GPIO_PIN_11
  *         @arg @ref LL_SYSCFG_GPIO_PIN_12
  *         @arg @ref LL_SYSCFG_GPIO_PIN_13
  *         @arg @ref LL_SYSCFG_GPIO_PIN_14
  *         @arg @ref LL_SYSCFG_GPIO_PIN_15
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledGPIOAnalog2(uint32_t GPIOPort, uint32_t GPIOPin)
{
  if(GPIOPort == LL_SYSCFG_GPIO_PORTA)
  {
    return ((READ_BIT(SYSCFG->PAANA2EN, GPIOPin) == (GPIOPin)) ? 1UL : 0UL);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTB)
  {
    return ((READ_BIT(SYSCFG->PBANA2EN, GPIOPin) == (GPIOPin)) ? 1UL : 0UL);
  }
  else
  {
    return ((READ_BIT(SYSCFG->PFANA2EN, GPIOPin) == (GPIOPin)) ? 1UL : 0UL);
  }
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_EIIC   GPIO EIICSignal 
  * @{
  */
/**
  * @brief  Enable I2C pin Signal.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_EIIC_PA8
  *            @arg LL_SYSCFG_EIIC_PA11
  *            @arg LL_SYSCFG_EIIC_PF2
  *            @arg LL_SYSCFG_EIIC_PF3
  *            @arg LL_SYSCFG_EIIC_PF4
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableI2CPinSignal(uint32_t PORT_Pin)
{
  SET_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Disable I2C pin Signal.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_EIIC_PA8
  *            @arg LL_SYSCFG_EIIC_PA11
  *            @arg LL_SYSCFG_EIIC_PF2
  *            @arg LL_SYSCFG_EIIC_PF3
  *            @arg LL_SYSCFG_EIIC_PF4
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableI2CPinSignal(uint32_t PORT_Pin)
{
  CLEAR_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Indicate if enable the GPIO State.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_EIIC_PA8
  *            @arg LL_SYSCFG_EIIC_PA11
  *            @arg LL_SYSCFG_EIIC_PF2
  *            @arg LL_SYSCFG_EIIC_PF3
  *            @arg LL_SYSCFG_EIIC_PF4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledI2CPinSignal(uint32_t PORT_Pin)
{
  return ((READ_BIT(SYSCFG->IOCFG, PORT_Pin) == (PORT_Pin)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_PUIIC   GPIO I2cPullUp 
  * @{
  */

/**
  * @brief  Enable I2C pin pull-up.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_PU_IIC_PF3
  *            @arg LL_SYSCFG_PU_IIC_PF4
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableI2CPinPullUp(uint32_t PORT_Pin)
{
  SET_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Disable I2C pin pull-up.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_PU_IIC_PF3
  *            @arg LL_SYSCFG_PU_IIC_PF4
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableI2CPinPullUp(uint32_t PORT_Pin)
{
  CLEAR_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Indicate if enable the GPIO State.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_PU_IIC_PF3
  *            @arg LL_SYSCFG_PU_IIC_PF4
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledI2CPinPullUp(uint32_t PORT_Pin)
{
  return ((READ_BIT(SYSCFG->IOCFG, PORT_Pin) == (PORT_Pin)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_EHS   GPIO EhsSignal 
  * @{
  */

/**
  * @brief  Enable high driving capability of LED pins.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_EHS_PA11
  *            @arg LL_SYSCFG_EHS_PA12
  *            @arg LL_SYSCFG_EHS_PA13
  *            @arg LL_SYSCFG_EHS_PA14
  *            @arg LL_SYSCFG_EHS_PA15
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableLEDPinHighDrvSignal(uint32_t PORT_Pin)
{
  SET_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Disable high driving capability of LED pins.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_EHS_PA11
  *            @arg LL_SYSCFG_EHS_PA12
  *            @arg LL_SYSCFG_EHS_PA13
  *            @arg LL_SYSCFG_EHS_PA14
  *            @arg LL_SYSCFG_EHS_PA15
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableLEDPinHighDrvSignal(uint32_t PORT_Pin)
{
  CLEAR_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Indicate if enable the GPIO State.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_EHS_PA11
  *            @arg LL_SYSCFG_EHS_PA12
  *            @arg LL_SYSCFG_EHS_PA13
  *            @arg LL_SYSCFG_EHS_PA14
  *            @arg LL_SYSCFG_EHS_PA15
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledLEDPinHighDrvSignal(uint32_t PORT_Pin)
{
  return ((READ_BIT(SYSCFG->IOCFG, PORT_Pin) == (PORT_Pin)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_IODRVP   GPIO I2cVccDriver 
  * @{
  */

/**
  * @brief  Enable I2C Vcc Driver Config.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_IODRVP_PA2
  *            @arg LL_SYSCFG_IODRVP_PA7
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableI2CVccIoDrvSignal(uint32_t PORT_Pin)
{
  SET_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Disable I2C Vcc Driver Config.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_IODRVP_PA2
  *            @arg LL_SYSCFG_IODRVP_PA7
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableI2CVccIoDrvSignal(uint32_t PORT_Pin)
{
  CLEAR_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Indicate if enable the GPIO State.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_IODRVP_PA2
  *            @arg LL_SYSCFG_IODRVP_PA7
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledI2CVccIoDrvSignal(uint32_t PORT_Pin)
{
  return ((READ_BIT(SYSCFG->IOCFG, PORT_Pin) == (PORT_Pin)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_ENSEG   GPIO Enable SEG 
  * @{
  */

/**
  * @brief  Enable SEG mode config.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_ENSEG_PA0
  *            @arg LL_SYSCFG_ENSEG_PA1
  *            @arg LL_SYSCFG_ENSEG_PA5
  *            @arg LL_SYSCFG_ENSEG_PA6
  *            @arg LL_SYSCFG_ENSEG_PA7
  *            @arg LL_SYSCFG_ENSEG_PA8
  *            @arg LL_SYSCFG_ENSEG_PB2
  *            @arg LL_SYSCFG_ENSEG_PB3
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_EnableEnsegSignal(uint32_t PORT_Pin)
{
  SET_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Disable SEG mode config.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_ENSEG_PA0
  *            @arg LL_SYSCFG_ENSEG_PA1
  *            @arg LL_SYSCFG_ENSEG_PA5
  *            @arg LL_SYSCFG_ENSEG_PA6
  *            @arg LL_SYSCFG_ENSEG_PA7
  *            @arg LL_SYSCFG_ENSEG_PA8
  *            @arg LL_SYSCFG_ENSEG_PB2
  *            @arg LL_SYSCFG_ENSEG_PB3
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_DisableEnsegSignal(uint32_t PORT_Pin)
{
  CLEAR_BIT(SYSCFG->IOCFG, PORT_Pin);
}

/**
  * @brief  Indicate if enable the GPIO State.
  * @param  PORT_Pin specifies the pin
  *         This parameter can be any combination of the following values:
  *            @arg LL_SYSCFG_ENSEG_PA0
  *            @arg LL_SYSCFG_ENSEG_PA1
  *            @arg LL_SYSCFG_ENSEG_PA5
  *            @arg LL_SYSCFG_ENSEG_PA6
  *            @arg LL_SYSCFG_ENSEG_PA7
  *            @arg LL_SYSCFG_ENSEG_PA8
  *            @arg LL_SYSCFG_ENSEG_PB2
  *            @arg LL_SYSCFG_ENSEG_PB3
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_SYSCFG_IsEnabledEnsegSignal(uint32_t PORT_Pin)
{
  return ((READ_BIT(SYSCFG->IOCFG, PORT_Pin) == (PORT_Pin)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_GPIO_IOPR   GPIO IOPR 
  * @{
  */
/**
  * @brief  SET GPIO IOPR
  * @note   Depending on devices and packages, some IOs may not be available.
  *         Refer to device datasheet for IOs availability.
  * @param  GPIOPort This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PORTA
  *         @arg @ref LL_SYSCFG_GPIO_PORTB
  *         @arg @ref LL_SYSCFG_GPIO_PORTF
  * @param  GPIOPin This parameter can be a combination of the following values:
  *         @arg @ref LL_SYSCFG_GPIO_PIN_0
  *         @arg @ref LL_SYSCFG_GPIO_PIN_1
  *         @arg @ref LL_SYSCFG_GPIO_PIN_2
  *         @arg @ref LL_SYSCFG_GPIO_PIN_3
  *         @arg @ref LL_SYSCFG_GPIO_PIN_4
  *         @arg @ref LL_SYSCFG_GPIO_PIN_5
  *         @arg @ref LL_SYSCFG_GPIO_PIN_6
  *         @arg @ref LL_SYSCFG_GPIO_PIN_7
  *         @arg @ref LL_SYSCFG_GPIO_PIN_8
  *         @arg @ref LL_SYSCFG_GPIO_PIN_9
  *         @arg @ref LL_SYSCFG_GPIO_PIN_10
  *         @arg @ref LL_SYSCFG_GPIO_PIN_11
  *         @arg @ref LL_SYSCFG_GPIO_PIN_12
  *         @arg @ref LL_SYSCFG_GPIO_PIN_13
  *         @arg @ref LL_SYSCFG_GPIO_PIN_14
  *         @arg @ref LL_SYSCFG_GPIO_PIN_15
  * @param  UpDown_Level This parameter can be a combination of the following values:     
  *         @arg @arg LL_SYSCFG_GPIO_IORP_OPEN: pull-up/pull-down is open
*           @arg @arg LL_SYSCFG_GPIO_IORP_10K:  pull-up/pull-down resistance 10K
*           @arg @arg LL_SYSCFG_GPIO_IORP_20K:  pull-up/pull-down resistance 20K
*           @arg @arg LL_SYSCFG_GPIO_IORP_40K:  pull-up/pull-down resistance 40K
  * @retval None
  */
__STATIC_INLINE void LL_SYSCFG_SetGpioPullResistanceValue(uint32_t GPIOPort, uint32_t GPIOPin, uint16_t UpDown_Level) 
{
  if(GPIOPort == LL_SYSCFG_GPIO_PORTA)
  {
    MODIFY_REG(SYSCFG->PAIORP, SYSCFG_PA_IORP_PA0_IORP << POSITION_VAL(GPIOPin)*2, UpDown_Level << POSITION_VAL(GPIOPin)*2);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTB)
  {
    MODIFY_REG(SYSCFG->PBIORP, SYSCFG_PB_IORP_PB0_IORP << POSITION_VAL(GPIOPin)*2, UpDown_Level << POSITION_VAL(GPIOPin)*2);
  }
  else if(GPIOPort == LL_SYSCFG_GPIO_PORTF)
  {
    MODIFY_REG(SYSCFG->PFIORP, SYSCFG_PF_IORP_PF0_IORP << POSITION_VAL(GPIOPin)*2, UpDown_Level << POSITION_VAL(GPIOPin)*2);
  }
  else
  {
    
  }
}

/**
  * @}
  */

/** @defgroup SYSTEM_LL_EF_FLASH FLASH Latency
  * @{
  */

/**
  * @brief  Set FLASH Latency
  * @rmtoll FLASH_ACR    FLASH_ACR_LATENCY       LL_FLASH_SetLatency
  * @param  Latency This parameter can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0
  *         @arg @ref LL_FLASH_LATENCY_1
  *         @arg @ref LL_FLASH_LATENCY_2 
  * @retval None
  */
__STATIC_INLINE void LL_FLASH_SetLatency(uint32_t Latency)
{
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, Latency);
}

/**
  * @brief  Get FLASH Latency
  * @rmtoll FLASH_ACR    FLASH_ACR_LATENCY       LL_FLASH_GetLatency
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_FLASH_LATENCY_0
  *         @arg @ref LL_FLASH_LATENCY_1
  *         @arg @ref LL_FLASH_LATENCY_2 
  */
__STATIC_INLINE uint32_t LL_FLASH_GetLatency(void)
{
  return (uint32_t)(READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY));
}

/**
  * @brief  Return the USRLOCK identifier 
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_FLASH_GetUsrLock(void)
{
  return (uint32_t)(READ_BIT(FLASH->SR, FLASH_SR_USRLOCK));
}

/**
  * @}
  */


/** @defgroup SYSTEM_LL_EF_DBGMCU DBGMCU
  * @{
  */

/**
  * @brief  Return the device identifier
  * @retval Values between Min_Data=0x00 and Max_Data=0xFFF
  */
__STATIC_INLINE uint32_t LL_DBGMCU_GetDeviceID(void)
{
  return (uint32_t)(READ_BIT(DBGMCU->IDCODE, DBGMCU_IDCODE_DEV_ID));
}

/**
  * @brief  Enable the Debug Module during STOP mode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGStopMode(void)
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Disable the Debug Module during STOP mode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGStopMode(void)
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);
}

/**
  * @brief  Indicate if enable the Debug Module during STOP mode
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_IsEnabledDBGStopMode(void)
{
  return ((READ_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP) == (DBGMCU_CR_DBG_STOP)) ? 1UL : 0UL);
}

/**
  * @brief  Enable the Debug Module during SLEEP mode
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_EnableDBGSleepMode(void) 
{
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Disable the Debug Module during SLEEP mode 
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_DisableDBGSleepMode(void) 
{
  CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);
}

/**
  * @brief  Indicate if enable the Debug Module during SLEEP mode
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_IsEnabledDBGSleepMode(void) 
{
  return ((READ_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP) == (DBGMCU_CR_DBG_SLEEP)) ? 1UL : 0UL);
}

/**
  * @brief  Freeze APB1 peripherals (group1 peripherals)
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP1_IWDG_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_RTC_STOP
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP1_FreezePeriph(uint32_t Periphs)
{
  SET_BIT(DBGMCU->APBFZ1, Periphs);
}

/**
  * @brief  Unfreeze APB1 peripherals (group1 peripherals)
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP1_IWDG_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_RTC_STOP
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP1_UnFreezePeriph(uint32_t Periphs)
{
  CLEAR_BIT(DBGMCU->APBFZ1, Periphs);
}

/**
  * @brief  Indicate if Freeze APB1 peripherals (group1 peripherals)
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  Periphs This parameter can be one of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP1_IWDG_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP1_RTC_STOP
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_APB1_GRP1_IsFreezePeriph(uint32_t Periphs)
{
  return ((READ_BIT(DBGMCU->APBFZ1, Periphs) == (Periphs)) ? 1UL : 0UL);
}

/**
  * @brief  Freeze APB1 peripherals(group2 peripherals)
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP2_TIM1_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP2_TIM14_STOP
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP2_FreezePeriph(uint32_t Periphs)
{
  SET_BIT(DBGMCU->APBFZ2, Periphs);
}

/**
  * @brief  Unfreeze APB1 peripherals(group2 peripherals)
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP2_TIM1_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP2_TIM14_STOP
  * @retval None
  */
__STATIC_INLINE void LL_DBGMCU_APB1_GRP2_UnFreezePeriph(uint32_t Periphs)
{
  CLEAR_BIT(DBGMCU->APBFZ2, Periphs);
}

/**
  * @brief  Indicate if Freeze APB1 peripherals (group2 peripherals)
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripherals availability.
  * @param  Periphs This parameter can be one of the following values:
  *         @arg @ref LL_DBGMCU_APB1_GRP2_TIM1_STOP
  *         @arg @ref LL_DBGMCU_APB1_GRP2_TIM14_STOP
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DBGMCU_APB1_GRP2_IsFreezePeriph(uint32_t Periphs)
{
  return ((READ_BIT(DBGMCU->APBFZ2, Periphs) == (Periphs)) ? 1UL : 0UL);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (FLASH) || defined (SYSCFG) || defined (DBGMCU) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* PY32T020_LL_SYSTEM_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

/**
  ******************************************************************************
  * @file    py32t020_hal_rcc_ex.h
  * @author  MCU Application Team
  * @brief   Header file of RCC HAL Extended module.
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
#ifndef __PY32T020_HAL_RCC_EX_H
#define __PY32T020_HAL_RCC_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32t020_hal_def.h"

/** @addtogroup PY32T020_HAL_Driver
  * @{
  */

/** @addtogroup RCCEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Types RCCEx Exported Types
  * @{
  */

/**
  * @brief  RCC extended clocks structure definition
  */
typedef struct
{
  uint32_t PeriphClockSelection;   /*!< The Extended Clock to be configured.
                                        This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */

#if defined(RCC_CCIPR_COMP1SEL)
  uint32_t Comp1ClockSelection;    /*!< Specifies COMP1 clock source.
                                        This parameter can be a value of @ref RCCEx_COMP1_Clock_Source */
#endif

#if defined(RCC_CCIPR_COMP2SEL)
  uint32_t Comp2ClockSelection;    /*!< Specifies COMP2 clock source.
                                        This parameter can be a value of @ref RCCEx_COMP2_Clock_Source */
#endif

#if defined(RCC_CCIPR_IWDGSEL)
  uint32_t IWDGClockSelection;    /*!< Specifies IWDG clock source
                                        This parameter can be a value of @ref RCCEx_IWDG_Clock_Source */
#endif

#if defined(RCC_BDCR_RTCSEL)
  uint32_t RTCClockSelection;      /*!< Specifies RTC clock source.
                                        This parameter can be a value of @ref RCC_RTC_Clock_Source */
#endif
} RCC_PeriphCLKInitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Constants RCCEx Exported Constants
  * @{
  */
/** @defgroup RCCEx_LSC_Clock_Source Low Speed Clock Source
  * @{
  */
#define RCC_LSCSOURCE_LSI             0x00000000U           /*!< LSI selection for low speed clock output */
#if defined(RCC_LSE_SUPPORT)
#define RCC_LSCSOURCE_LSE             RCC_BDCR_LSCSEL       /*!< LSE selection for low speed clock output */
#endif
/**
  * @}
  */

/** @defgroup RCCEx_Periph_Clock_Selection Periph Clock Selection
  * @{
  */
#if defined(RCC_CCIPR_COMP1SEL)
#define RCC_PERIPHCLK_COMP1            0x00000002U
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
#define RCC_PERIPHCLK_COMP2            0x00000020U
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_IWDGSEL)
#define RCC_PERIPHCLK_IWDG             0x00000200U
#endif /* RCC_CCIPR_IWDGSEL */

#if defined(RCC_BDCR_RTCSEL)
#define RCC_PERIPHCLK_RTC              0x00020000U
#endif /* RCC_BDCR_RTCSEL */

/**
  * @}
  */

#if defined(RCC_CCIPR_COMP1SEL)
/** @defgroup RCCEx_COMP1_Clock_Source RCC COMP1 Clock Source
  * @{
  */
#define RCC_COMP1CLKSOURCE_PCLK        0x00000000U                                      /*!< APB clock selected as COMP1 clock */
#define RCC_COMP1CLKSOURCE_LSC         RCC_CCIPR_COMP1SEL                               /*!< LSC clock selected as COMP1 clock */
/**
  * @}
  */
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
/** @defgroup RCCEx_COMP2_Clock_Source RCC COMP2 Clock Source
  * @{
  */
#define RCC_COMP2CLKSOURCE_PCLK        0x00000000U                                      /*!< APB clock selected as COMP2 clock */
#define RCC_COMP2CLKSOURCE_LSC         RCC_CCIPR_COMP2SEL                               /*!< LSC clock selected as COMP2 clock */
/**
  * @}
  */
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_IWDGSEL)
/** @defgroup RCCEx_IWDG_Clock_Source RCC IWDG Clock Source
  * @{
  */
#define RCC_IWDGCLKSOURCE_LSI         0x00000000U            /*!< LSI clock selected as IWDG clock */
#if defined(RCC_LSE_SUPPORT)
#define RCC_IWDGCLKSOURCE_LSE        RCC_CCIPR_IWDGSEL       /*!< LSE clock selected as IWDG clock */
#endif
/**
  * @}
  */
#endif /* RCC_CCIPR_IWDGSEL */

/** @defgroup RCCEx_TIM_PCLK_Frequency_Control TIM PCLK Frequency Control
  * @{
  */
#define RCC_TIMPCLK_MUL2       0x00000000U               /*!< TIMER PCLK is twice the system PCLK, but the frequency will not exceed HCLK */
#define RCC_TIMPCLK_MUL1       RCC_CCIPR_TIMCLK_CTRL     /*!< TIMER PCLK is the system PCLK */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup RCCEx_Exported_Macros RCCEx Exported Macros
 * @{
 */

#if defined(RCC_CCIPR_COMP1SEL)
/** @brief  Macro to configure the COMP1 clock (COMP1CLK).
  *
  * @param  __COMP1_CLKSOURCE__ specifies the COMP1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_COMP1CLKSOURCE_PCLK   PCLK selected as COMP1 clock
  *            @arg @ref RCC_COMP1CLKSOURCE_LSC    LSC selected as COMP1 clock
  */
#define __HAL_RCC_COMP1_CONFIG(__COMP1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_COMP1SEL, (uint32_t)(__COMP1_CLKSOURCE__))

/** @brief  Macro to get the COMP1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_COMP1CLKSOURCE_PCLK   PCLK selected as COMP1 clock
  *            @arg @ref RCC_COMP1CLKSOURCE_LSC    LSC selected as COMP1 clock
  */
#define __HAL_RCC_GET_COMP1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_COMP1SEL)))
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
/** @brief  Macro to configure the COMP2 clock (COMP2CLK).
  *
  * @param  __COMP2_CLKSOURCE__ specifies the COMP2 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_COMP2CLKSOURCE_PCLK   PCLK selected as COMP2 clock
  *            @arg @ref RCC_COMP2CLKSOURCE_LSC  LSC selected as COMP2 clock
  */
#define __HAL_RCC_COMP2_CONFIG(__COMP2_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_COMP2SEL, (uint32_t)(__COMP2_CLKSOURCE__))

/** @brief  Macro to get the COMP2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_COMP2CLKSOURCE_PCLK   PCLK selected as COMP2 clock
  *            @arg @ref RCC_COMP2CLKSOURCE_LSC  LSC selected as COMP2 clock
  */
#define __HAL_RCC_GET_COMP2_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_COMP2SEL)))
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_IWDGSEL)
/** @brief  Macro to configure the IWDG clock (IWDGCLK).
  *
  * @param  __IWDG_CLKSOURCE__ specifies the IWDG clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_IWDGCLKSOURCE_LSI  LSI  selected as IWDG clock
  *            @arg @ref RCC_IWDGCLKSOURCE_LSE  LSE  selected as IWDG clock
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  */
#define __HAL_RCC_IWDG_CONFIG(__IWDG_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_IWDGSEL, (uint32_t)(__IWDG_CLKSOURCE__))

/** @brief  Macro to get the IWDG clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_IWDGCLKSOURCE_LSI  LSI selected as IWDG clock
  *            @arg @ref RCC_IWDGCLKSOURCE_LSE  LSE selected as IWDG clock
  * @note   Depending on devices and packages, some clocks may not be available.
  *         Refer to device datasheet for clocks availability.
  */
#define __HAL_RCC_GET_IWDG_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_IWDGSEL)))
#endif /* RCC_CCIPR_IWDGSEL */

/** @brief  Macro to configure the TIMER PCLK frequency control.
  * @param  __TIMPCLK_MUL__ TIMPCLK multiple frequency factor.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_TIMPCLK_MUL2 TIMER PCLK is twice the system PCLK, but the frequency will not exceed HCLK
  *            @arg @ref RCC_TIMPCLK_MUL1 TIMER PCLK is the system PCLK
  */
#define __HAL_RCC_TIMPCLK_CONFIG(__TIMPCLK_MUL__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_TIMCLK_CTRL, (uint32_t)(__TIMPCLK_MUL__))

/** @brief  Macro to get the TIMPCLK multiple frequency factor.
  * @retval The multiple frequency factor can be one of the following values:
  *            @arg @ref RCC_TIMPCLK_MUL2 TIMER PCLK is twice the system PCLK, but the frequency will not exceed HCLK
  *            @arg @ref RCC_TIMPCLK_MUL1 TIMER PCLK is the system PCLK
  */
#define __HAL_RCC_GET_TIMPCLK_MUL() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_TIMCLK_CTRL)))

/** @defgroup RCCEx_Flags_Interrupts_Management Flags Interrupts Management
  * @brief macros to manage the specified RCC Flags and interrupts.
  * @{
  */



/**
  * @}
  */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCCEx_Exported_Functions
  * @{
  */

/** @addtogroup RCCEx_Exported_Functions_Group1
  * @{
  */

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

/**
  * @}
  */

/** @addtogroup RCCEx_Exported_Functions_Group2
  * @{
  */
void HAL_RCCEx_SetLSCSource(uint32_t LSCSource);
uint32_t HAL_RCCEx_GetLSCSource(void);
/**
  * @}
  */


/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RCCEx_Private_Macros RCCEx Private Macros
  * @{
  */
#define IS_RCC_LSCSOURCE(__SOURCE__) (((__SOURCE__) == RCC_LSCSOURCE_LSI) || \
                                      ((__SOURCE__) == RCC_LSCSOURCE_LSE))

#if defined(RCC_CCIPR_COMP1SEL)
#define IS_RCC_COMP1CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_COMP1CLKSOURCE_PCLK)  || \
                ((__SOURCE__) == RCC_COMP1CLKSOURCE_LSC))
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
#define IS_RCC_COMP2CLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_COMP2CLKSOURCE_PCLK)  || \
                ((__SOURCE__) == RCC_COMP2CLKSOURCE_LSC))
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_IWDGSEL)
#define IS_RCC_IWDGCLKSOURCE(__SOURCE__)  \
               (((__SOURCE__) == RCC_IWDGCLKSOURCE_LSI)  || \
                ((__SOURCE__) == RCC_IWDGCLKSOURCE_LSE))
#endif /* RCC_CCIPR_IWDGSEL */

#define IS_RCC_PERIPHCLOCK(__SELECTION__)  \
  ((((__SELECTION__) & RCC_PERIPHCLK_COMP1)   == RCC_PERIPHCLK_COMP1)   || \
   (((__SELECTION__) & RCC_PERIPHCLK_COMP2)   == RCC_PERIPHCLK_COMP2)   || \
   (((__SELECTION__) & RCC_PERIPHCLK_IWDG)    == RCC_PERIPHCLK_IWDG)    || \
   (((__SELECTION__) & RCC_PERIPHCLK_RTC)     == RCC_PERIPHCLK_RTC))
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

#endif /* __PY32T020_HAL_RCC_EX_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

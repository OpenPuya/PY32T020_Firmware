/**
  ******************************************************************************
  * @file    py32t020_hal_rcc_ex.c
  * @author  MCU Application Team
  * @brief   Extended RCC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities RCC extended peripheral:
  *           + Extended Peripheral Control functions
  *           + Extended Clock management functions
  *
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
#include "py32t0xx_hal.h"

/** @addtogroup PY32T020_HAL_Driver
  * @{
  */

/** @defgroup RCCEx RCCEx
  * @brief RCC Extended HAL module driver
  * @{
  */

#ifdef HAL_RCC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup RCCEx_Exported_Functions RCCEx Exported Functions
  * @{
  */

/** @defgroup RCCEx_Exported_Functions_Group1 Extended Peripheral Control functions
 *  @brief  Extended Peripheral Control functions
 *
@verbatim
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RCC Clocks
    frequencies.
    [..]
    (@) Important note: Care must be taken when @ref HAL_RCCEx_PeriphCLKConfig() is used to
        select the RTC clock source; in this case the Backup domain will be reset in
        order to modify the RTC Clock source, as consequence RTC registers (including
        the backup registers) and RCC_BDCR register are set to their reset values.

@endverbatim
  * @{
  */
/**
  * @brief  Initialize the RCC extended peripherals clocks according to the specified
  *         parameters in the @ref RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit  pointer to a @ref RCC_PeriphCLKInitTypeDef structure that
  *         contains a field PeriphClockSelection which can be a combination of the following values:
  *            @arg @ref RCC_PERIPHCLK_RTC     RTC peripheral clock  (1)
  *            @arg @ref RCC_PERIPHCLK_COMP1   COMP1 peripheral clock  (1)
  *            @arg @ref RCC_PERIPHCLK_COMP2   COMP2 peripheral clock  (1)
  *            @arg @ref RCC_PERIPHCLK_IWDG    IWDG peripheral clock  (1)
  *
  * @note   (1) Peripherals maybe not available on some devices
  * @note   Care must be taken when @ref HAL_RCCEx_PeriphCLKConfig() is used to select
  *         the RTC clock source: in this case the access to Backup domain is enabled.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
#if defined(RCC_BDCR_RTCSEL)
  uint32_t tickstart = 0U, temp_reg = 0U;
  FlagStatus       pwrclkchanged = RESET;
#endif
  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));
#if defined(RCC_BDCR_RTCSEL)
  /*------------------------------- RTC Configuration ------------------------*/
  if ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC))
  {
    /* check for RTC Parameters used to output RTCCLK */
    assert_param(IS_RCC_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* As soon as function is called to change RTC clock source, activation of the
       power domain is done. */
    /* Requires to enable write access to Backup Domain of necessary */
    if (__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if (HAL_IS_BIT_CLR(PWR->CR1, PWR_CR1_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR1, PWR_CR1_DBP);

      /* Wait for Backup domain Write protection disable */
      tickstart = HAL_GetTick();

      while (HAL_IS_BIT_CLR(PWR->CR1, PWR_CR1_DBP))
      {
        if ((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return HAL_TIMEOUT;
        }
      }
    }

    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    temp_reg = (RCC->BDCR & RCC_BDCR_RTCSEL);
    if ((temp_reg != 0x00000000U) && (temp_reg != (PeriphClkInit->RTCClockSelection & RCC_BDCR_RTCSEL)))
    {
      /* Store the content of BDCR register before the reset of Backup Domain */
      temp_reg = (RCC->BDCR & ~(RCC_BDCR_RTCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __HAL_RCC_BACKUPRESET_FORCE();
      __HAL_RCC_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCR register */
      RCC->BDCR = temp_reg;

#if defined(RCC_LSE_SUPPORT)
      /* Wait for LSERDY if LSE was enabled */
      if (HAL_IS_BIT_SET(temp_reg, RCC_BDCR_LSEON))
      {
        /* Get Start Tick */
        tickstart = HAL_GetTick();

        /* Wait till LSE is ready */
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if ((HAL_GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE)
          {
            return HAL_TIMEOUT;
          }
        }
      }
#endif
    }
    __HAL_RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);

    /* Require to disable power clock if necessary */
    if (pwrclkchanged == SET)
    {
      __HAL_RCC_PWR_CLK_DISABLE();
    }
  }
#endif /*RCC_BDCR_RTCSEL*/

#if defined(RCC_CCIPR_COMP1SEL)
  /*-------------------------- COMP1 clock source configuration -------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_COMP1) == RCC_PERIPHCLK_COMP1)
  {
    /* Check the parameters */
    assert_param(IS_RCC_COMP1CLKSOURCE(PeriphClkInit->Comp1ClockSelection));

    /* Configure the COMP1 clock source */
    __HAL_RCC_COMP1_CONFIG(PeriphClkInit->Comp1ClockSelection);
  }
#endif /* RCC_CCIPR_COMP1SEL */

#if defined(RCC_CCIPR_COMP2SEL)
  /*-------------------------- COMP2 clock source configuration -------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_COMP2) == RCC_PERIPHCLK_COMP2)
  {
    /* Check the parameters */
    assert_param(IS_RCC_COMP2CLKSOURCE(PeriphClkInit->Comp2ClockSelection));

    /* Configure the COMP2 clock source */
    __HAL_RCC_COMP2_CONFIG(PeriphClkInit->Comp2ClockSelection);
  }
#endif /* RCC_CCIPR_COMP2SEL */

#if defined(RCC_CCIPR_IWDGSEL)
  /*-------------------------- IWDG clock source configuration -------------------*/
  if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_IWDG) == (RCC_PERIPHCLK_IWDG))
  {
    assert_param(IS_RCC_IWDGCLKSOURCE(PeriphClkInit->IWDGClockSelection));
    __HAL_RCC_IWDG_CONFIG(PeriphClkInit->IWDGClockSelection);
  }
#endif /* RCC_CCIPR_IWDGSEL */

  return HAL_OK;
}

/**
  * @brief  Get the RCC_ClkInitStruct according to the internal RCC configuration registers.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         returns the configuration information for the Extended Peripherals
  *         clocks: COMP1, COMP2, RTC, IWDG
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripheral availability.
  * @retval None
  */
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = 0;
#if defined(RCC_CCIPR_COMP1SEL)
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_COMP1;
#endif /* RCC_CCIPR_COMP1SEL */
#if defined(RCC_CCIPR_COMP2SEL)
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_COMP2;
#endif /* RCC_CCIPR_COMP2SEL */
#if defined(RCC_CCIPR_IWDGSEL)
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_IWDG;
#endif /* RCC_CCIPR_IWDGSEL */
#if defined(RCC_BDCR_RTCSEL)
  PeriphClkInit->PeriphClockSelection |= RCC_PERIPHCLK_RTC;
#endif /* RCC_BDCR_RTCSEL */

#if defined(RCC_CCIPR_COMP1SEL)
  /* Get the COMP1 clock source --------------------------------------------*/
  PeriphClkInit->Comp1ClockSelection  = __HAL_RCC_GET_COMP1_SOURCE();
#endif /* RCC_CCIPR_COMP1SEL */
#if defined(RCC_CCIPR_COMP2SEL)
  /* Get the COMP2 clock source ---------------------------------------------*/
  PeriphClkInit->Comp2ClockSelection  = __HAL_RCC_GET_COMP2_SOURCE();
#endif /* RCC_CCIPR_COMP2SEL */
#if defined(RCC_CCIPR_IWDGSEL)
  /* Get the IWDG clock source ---------------------------------------------*/
  PeriphClkInit->IWDGClockSelection   = __HAL_RCC_GET_IWDG_SOURCE();
#endif /* RCC_CCIPR_IWDGSEL */
#if defined(RCC_BDCR_RTCSEL)
  /* Get the RTC clock source ------------------------------------------------*/
  PeriphClkInit->RTCClockSelection    = __HAL_RCC_GET_RTC_SOURCE();
#endif /* RCC_BDCR_RTCSEL */
}

/**
  * @brief  Return the peripheral clock frequency for peripherals with clock source
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk  Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_PERIPHCLK_RTC     RTC peripheral clock
  *            @arg @ref RCC_PERIPHCLK_COMP1   COMP1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_COMP2   COMP2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_IWDG    IWDG peripheral clock
  * @note   Depending on devices and packages, some Peripherals may not be available.
  *         Refer to device datasheet for Peripheral availability.
  * @retval Frequency in Hz
  */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  uint32_t frequency = 0U;
  uint32_t srcclk;

  /* Check the parameters */
  assert_param(IS_RCC_PERIPHCLOCK(PeriphClk));

#if defined(RCC_BDCR_RTCSEL)
  if (PeriphClk == RCC_PERIPHCLK_RTC)
  {
    /* Get the current RTC source */
    srcclk = __HAL_RCC_GET_RTC_SOURCE();

    /* Check if LSI is ready and if RTC clock selection is LSI */
    if ((HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)) && (srcclk == RCC_RTCCLKSOURCE_LSI))
    {
      frequency = LSI_VALUE;
    }
#if defined(RCC_LSE_SUPPORT)
    /* Check if LSE is ready and if RTC clock selection is LSE */
    else if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) && (srcclk == RCC_RTCCLKSOURCE_LSE))
    {
      frequency = LSE_VALUE;
    }
#endif
    /* Check if HSE is ready  and if RTC clock selection is HSE_DIV32*/
    else if ((HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSERDY)) &&(srcclk == RCC_RTCCLKSOURCE_HSE_DIV32))
    {
      frequency = HSE_VALUE / 32U;
    }
    /* Check if HSE is ready  and if RTC clock selection is HSE_DIV128*/
    else if ((HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSERDY)) &&(srcclk == RCC_RTCCLKSOURCE_HSE_DIV128))
    {
      frequency = HSE_VALUE / 128U;
    }
    /* Check if HSE is ready  and if RTC clock selection is HSE_DIV8*/
    else if ((HAL_IS_BIT_SET(RCC->CR, RCC_CR_HSERDY)) &&(srcclk == RCC_RTCCLKSOURCE_HSE_DIV8))
    {
      frequency = HSE_VALUE / 8U;
    }
    /* Clock not enabled for RTC*/
    else
    {
      /* Nothing to do as frequency already initialized to 0U */
    }
  }
  else
  {
#endif
    /* Other external peripheral clock source than RTC */

    switch (PeriphClk)
    {
#if defined(RCC_CCIPR_COMP1SEL)
    case RCC_PERIPHCLK_COMP1:
      /* Get the current COMP1 source */
      srcclk = __HAL_RCC_GET_COMP1_SOURCE();

      if (srcclk == RCC_COMP1CLKSOURCE_PCLK)            /* PCLK1 */
      {
        frequency = HAL_RCC_GetPCLK1Freq();
      }
#if defined(RCC_LSE_SUPPORT)
      else if ((HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)) && (HAL_IS_BIT_CLR(RCC->BDCR, RCC_BDCR_LSCSEL)) \
                && (srcclk == RCC_COMP1CLKSOURCE_LSC))
      {
        frequency = LSI_VALUE;
      }
      else if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCSEL)) \
                && (srcclk == RCC_COMP1CLKSOURCE_LSC))
      {
        frequency = LSE_VALUE;
      }
      /* Clock not enabled for COMP1 */
      else
      {
        /* Nothing to do as frequency already initialized to 0U */
      }
#else
      else
      {
        frequency = LSI_VALUE;
      }
#endif
      break;
#endif

#if defined(RCC_CCIPR_COMP2SEL)
    case RCC_PERIPHCLK_COMP2:
      /* Get the current COMP2 source */
      srcclk = __HAL_RCC_GET_COMP2_SOURCE();

      if (srcclk == RCC_COMP2CLKSOURCE_PCLK)            /* PCLK1 */
      {
        frequency = HAL_RCC_GetPCLK1Freq();
      }
#if defined(RCC_LSE_SUPPORT)
      else if ((HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)) && (HAL_IS_BIT_CLR(RCC->BDCR, RCC_BDCR_LSCSEL)) \
                && (srcclk == RCC_COMP2CLKSOURCE_LSC))
      {
        frequency = LSI_VALUE;
      }
      else if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) && (HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSCSEL)) \
                && (srcclk == RCC_COMP2CLKSOURCE_LSC))
      {
        frequency = LSE_VALUE;
      }
      /* Clock not enabled for COMP2 */
      else
      {
        /* Nothing to do as frequency already initialized to 0U */
      }
#else
      else
      {
        frequency = LSI_VALUE;
      }
#endif
      break;
#endif

#if defined(RCC_CCIPR_IWDGSEL)
    case RCC_PERIPHCLK_IWDG:
      /* Get the current IWDG source */
      srcclk = __HAL_RCC_GET_IWDG_SOURCE();

      if ((HAL_IS_BIT_SET(RCC->CSR, RCC_CSR_LSIRDY)) && (srcclk == RCC_IWDGCLKSOURCE_LSI))
      {
        frequency = LSI_VALUE;
      }
#if defined(RCC_LSE_SUPPORT)
      else if ((HAL_IS_BIT_SET(RCC->BDCR, RCC_BDCR_LSERDY)) && (srcclk == RCC_IWDGCLKSOURCE_LSE))
      {
        frequency = LSE_VALUE;
      }
#endif
      /* Clock not enabled for IWDG */
      else
      {
        /* Nothing to do as frequency already initialized to 0U */
      }
      break;
#endif /* RCC_CCIPR_IWDGSEL */

    default:
      break;
    }
#if defined(RCC_BDCR_RTCSEL)
  }
#endif
  return (frequency);
}

/**
  * @}
  */

/** @defgroup RCCEx_Exported_Functions_Group2 Extended Clock management functions
 *  @brief  Extended Clock management functions
 *
@verbatim
 ===============================================================================
                ##### Extended clock management functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the
    activation or deactivation of LSE CSS, Low speed clock output and
    clock after wake-up from STOP mode.
@endverbatim
  * @{
  */

/**
  * @brief  Set the Low Speed clock source.
  * @param  LSCSource  specifies the Low Speed clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_LSCSOURCE_LSI  LSI clock selected as LSC source
  *            @arg @ref RCC_LSCSOURCE_LSE  LSE clock selected as LSC source
  * @retval None
  */
void HAL_RCCEx_SetLSCSource(uint32_t LSCSource)
{
  FlagStatus       pwrclkchanged = RESET;
  FlagStatus       backupchanged = RESET;
  
  /* Check the parameters */
  assert_param(IS_RCC_LSCSOURCE(LSCSource));

  /* Update LSCSEL clock source in Backup Domain control register */
  if (__HAL_RCC_PWR_IS_CLK_DISABLED())
  {
    __HAL_RCC_PWR_CLK_ENABLE();
    pwrclkchanged = SET;
  }
  if (HAL_IS_BIT_CLR(PWR->CR1, PWR_CR1_DBP))
  {

    HAL_PWR_EnableBkUpAccess();
    backupchanged = SET;
  }

  MODIFY_REG(RCC->BDCR, RCC_BDCR_LSCSEL, LSCSource);

  if (backupchanged == SET)
  {
    HAL_PWR_DisableBkUpAccess();
  }
  if (pwrclkchanged == SET)
  {
    __HAL_RCC_PWR_CLK_DISABLE();
  }
}

/**
  * @brief  Get the Low Speed clock source.
  * @retval Returned value can be one of the following values:
  *         @arg @ref RCC_LSCSOURCE_LSI
  *         @arg @ref RCC_LSCSOURCE_LSE
  */
uint32_t HAL_RCCEx_GetLSCSource(void)
{
  uint32_t         LSCSource;

  LSCSource = READ_BIT(RCC->BDCR, RCC_BDCR_LSCSEL);

  return LSCSource;
}
/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_RCC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

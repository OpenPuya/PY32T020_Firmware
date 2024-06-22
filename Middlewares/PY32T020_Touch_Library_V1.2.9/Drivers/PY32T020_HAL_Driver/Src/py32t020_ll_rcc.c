/**
  ******************************************************************************
  * @file    py32t020_ll_rcc.c
  * @author  MCU Application Team
  * @brief   RCC LL module driver.
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
#if defined(USE_FULL_LL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "py32t020_ll_rcc.h"
#ifdef  USE_FULL_ASSERT
  #include "py32_assert.h"
#else
  #define assert_param(expr) ((void)0U)
#endif
/** @addtogroup PY32T020_LL_Driver
  * @{
  */

#if defined(RCC)

/** @addtogroup RCC_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @addtogroup RCC_LL_Private_Macros
  * @{
  */
#define RCC_HSI10M                            10000000U                   /*!< The frequency value of the HSI10M */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup RCC_LL_Private_Macros
  * @{
  */
#define IS_LL_RCC_MCO_CLKSOURCE(__VALUE__)  (((__VALUE__) == LL_RCC_MCO_CLKSOURCE))

#if (defined(RCC_CCIPR_COMP1SEL) && defined(RCC_CCIPR_COMP2SEL))
#define IS_LL_RCC_COMP_CLKSOURCE(__VALUE__)  (((__VALUE__) == LL_RCC_COMP1_CLKSOURCE) || \
                                              ((__VALUE__) == LL_RCC_COMP2_CLKSOURCE))
#endif

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup RCC_LL_Private_Functions RCC Private functions
  * @{
  */
uint32_t RCC_GetSystemClockFreq(void);
uint32_t RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency);
uint32_t RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency);
/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCC_LL_Exported_Functions
  * @{
  */

/** @addtogroup RCC_LL_EF_Init
  * @{
  */

/**
  * @brief  Reset the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *         - HSI ON and used as system clock source
  *         - HSE OFF
  *         - AHB and APB1 prescaler set to 1.
  *         - CSS, MCO OFF
  *         - All interrupts disabled
  * @note   This function does not modify the configuration of the
  *         - Peripheral clocks
  *         - LSI, LSE and RTC clocks
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RCC registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus LL_RCC_DeInit(void)
{
  /* Set HSION bit and wait for HSI READY bit */
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1U)
  {}

  /* Set HSI_FS, HSITRIM bits to default value*/
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);

  /* Reset CFGR register */
  LL_RCC_WriteReg(CFGR, 0x00000000U);

  /* Wait till SYSCLK is HSISYS */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {}

  /* Reset whole CR register but HSI in 2 steps in case HSEBYP is set */
  LL_RCC_WriteReg(CR, RCC_CR_HSION);
  while (LL_RCC_HSE_IsReady() != 0U)
  {}
  LL_RCC_WriteReg(CR, RCC_CR_HSION);

  /* Disable all interrupts */
  LL_RCC_WriteReg(CIER, 0x00000000U);

  /* Clear all interrupts flags */
  LL_RCC_WriteReg(CICR, 0xFFFFFFFFU);

  return SUCCESS;
}

/**
  * @}
  */

/** @addtogroup RCC_LL_EF_Get_Freq
  * @brief  Return the frequencies of different on chip clocks;  System, AHB and APB1 buses clocks
  *         and different peripheral clocks available on the device.
  * @note   If SYSCLK source is HSI, function returns values based on HSI_VALUE divided by HSI division factor(**)
  * @note   If SYSCLK source is HSE, function returns values based on HSE_VALUE(***)
  * @note   (**) HSI_VALUE is a constant defined in this file (default value
  *              8 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  * @note   (***) HSE_VALUE is a constant defined in this file (default value
  *               8 MHz), user has to ensure that HSE_VALUE is same as the real
  *               frequency of the crystal used. Otherwise, this function may
  *               have wrong result.
  * @note   The result of this function could be incorrect when using fractional
  *         value for HSE crystal.
  * @note   This function can be used by the user application to compute the
  *         baud-rate for the communication peripherals or configure other parameters.
  * @{
  */

/**
  * @brief  Return the frequencies of different on chip clocks;  System, AHB and APB1 buses clocks
  * @note   Each time SYSCLK, HCLK and/or PCLK1 clock changes, this function
  *         must be called to update structure fields. Otherwise, any
  *         configuration based on this function will be incorrect.
  * @param  RCC_Clocks pointer to a @ref LL_RCC_ClocksTypeDef structure which will hold the clocks frequencies
  * @retval None
  */
void LL_RCC_GetSystemClocksFreq(LL_RCC_ClocksTypeDef *RCC_Clocks)
{
  /* Get SYSCLK frequency */
  RCC_Clocks->SYSCLK_Frequency = RCC_GetSystemClockFreq();

  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency   = RCC_GetHCLKClockFreq(RCC_Clocks->SYSCLK_Frequency);

  /* PCLK1 clock frequency */
  RCC_Clocks->PCLK1_Frequency  = RCC_GetPCLK1ClockFreq(RCC_Clocks->HCLK_Frequency);
}

/**
  * @brief  Return MCO clock frequency
  * @param  MCOx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO_CLKSOURCE
  * @retval MCO clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (HSE, LSI or LSE) is not ready
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NA indicates that no clock source selected
  */
uint32_t LL_RCC_GetMCOClockFreq(uint32_t MCOx)
{
  uint32_t mco_frequency = LL_RCC_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  assert_param(IS_LL_RCC_MCO_CLKSOURCE(MCOx));

  switch (LL_RCC_GetMCOClockSource(MCOx))
  {
  case LL_RCC_MCOSOURCE_SYSCLK:         /* MCO Clock is SYSCLK */
    mco_frequency = SystemCoreClock;
    break;
  case LL_RCC_MCOSOURCE_HSI10M:         /* MCO Clock is HSI10M */
    mco_frequency = RCC_HSI10M;
    break;
  case LL_RCC_MCOSOURCE_HSI:            /* MCO Clock is HSI */
    mco_frequency = LL_RCC_HSI_GetFreq();
    break;
  case LL_RCC_MCOSOURCE_HSE:            /* MCO Clock is HSE */
    if (LL_RCC_HSE_IsReady() == 1U)
    {
      mco_frequency = HSE_VALUE;
    }
    break;
  case LL_RCC_MCOSOURCE_LSI:            /* MCO Clock is LSI */
    if (LL_RCC_LSI_IsReady() == 1U)
    {
      mco_frequency = LSI_VALUE;
    }
    break;
#if defined(RCC_LSE_SUPPORT)
  case LL_RCC_MCOSOURCE_LSE:            /* MCO Clock is LSE */
    if (LL_RCC_LSE_IsReady() == 1U)
    {
      mco_frequency = LSE_VALUE;
    }
    break;
#endif
    case LL_RCC_MCOSOURCE_HCLK:         /* MCO Clock is HCLK */
      mco_frequency = RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq());
      break;
    case LL_RCC_MCOSOURCE_PCLK:         /* MCO Clock is PCLK */
      mco_frequency = RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()));
      break;
    case LL_RCC_MCOSOURCE_HCLK_SRC:     /* MCO Clock is HCLK_SRC */
      mco_frequency = RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq());
      break;
    case LL_RCC_MCOSOURCE_PCLK_UNGATE:  /* MCO Clock is PCLK_UNGATE */
      mco_frequency = RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()));
      break;
  case LL_RCC_MCOSOURCE_NOCLOCK:        /* No clock used as MCO clock source */
  default:
    mco_frequency = LL_RCC_PERIPH_FREQUENCY_NA;
    return mco_frequency;
  }

  mco_frequency = mco_frequency / (1U << (LL_RCC_GetMCODiv(MCOx) >> RCC_CFGR_MCOPRE_Pos));

  return mco_frequency;
}

#if defined(RCC_BDCR_LSCSEL)
/**
  * @brief  Return LSC clock frequency
  * @retval LSC clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (LSI or LSE) is not ready
  */
uint32_t LL_RCC_GetLSCClockFreq(void)
{
#if defined(RCC_LSE_SUPPORT)
  uint32_t lsc_frequency = LL_RCC_PERIPH_FREQUENCY_NO;

  switch (LL_RCC_LSC_GetSource())
  {
  case LL_RCC_LSC_CLKSOURCE_LSE:    /* LSC Clock is LSE Osc. */
    if (LL_RCC_LSE_IsReady() == 1U)
    {
      lsc_frequency = LSE_VALUE;
    }
    break;

  case LL_RCC_LSC_CLKSOURCE_LSI:    /* LSC Clock is LSI Osc. */
  default:
    if (LL_RCC_LSI_IsReady() == 1U)
    {
      lsc_frequency = LSI_VALUE;
    }
    break;
  }
  return lsc_frequency;
#else
  return LSI_VALUE;
#endif
}
#endif

#if defined(COMP1)
/**
  * @brief  Return COMP clock frequency
  * @param  COMPx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_COMP1_CLKSOURCE
  *         @arg @ref LL_RCC_COMP2_CLKSOURCE
  * @retval COMP clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (PCLK1, LSI or LSE) is not ready
  */
uint32_t LL_RCC_GetCOMPClockFreq(uint32_t COMPx)
{
  uint32_t comp_frequency = LL_RCC_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  assert_param(IS_LL_RCC_COMP_CLKSOURCE(COMPx));

  if (COMPx == LL_RCC_COMP1_CLKSOURCE)
  {
    /* COMP1CLK clock frequency */
    switch (LL_RCC_GetCOMPClockSource(COMPx))
    {
    case LL_RCC_COMP1_CLKSOURCE_LSC:    /* COMP1 Clock is LSC */
      comp_frequency = LL_RCC_GetLSCClockFreq();
      break;

    case LL_RCC_COMP1_CLKSOURCE_PCLK1:  /* COMP1 Clock is PCLK1 */
    default:
      comp_frequency = RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()));
      break;
    }
  }
#if defined(COMP2)
  else
  {
    /* COMP2CLK clock frequency */
    switch (LL_RCC_GetCOMPClockSource(COMPx))
    {
    case LL_RCC_COMP2_CLKSOURCE_LSC:    /* COMP2 Clock is LSC */
      comp_frequency = LL_RCC_GetLSCClockFreq();
      break;

    case LL_RCC_COMP2_CLKSOURCE_PCLK1:  /* COMP2 Clock is PCLK1 */
    default:
      comp_frequency = RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()));
      break;
    }
  }
#endif
  return comp_frequency;
}
#endif

#if defined(RCC_CCIPR_IWDGSEL)
/**
  * @brief  Return IWDG clock frequency
  * @retval IWDG clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (LSI or LSE) is not ready
  */
uint32_t LL_RCC_GetIWDGClockFreq(void)
{
  uint32_t iwdg_frequency = LL_RCC_PERIPH_FREQUENCY_NO;

  switch (LL_RCC_GetIWDGClockSource())
  {
  #if defined(RCC_LSE_SUPPORT)
  case LL_RCC_IWDG_CLKSOURCE_LSE:    /* IWDG Clock is LSE Osc. */
    if (LL_RCC_LSE_IsReady() == 1U)
    {
      iwdg_frequency = LSE_VALUE;
    }
    break;
  #endif
  case LL_RCC_IWDG_CLKSOURCE_LSI:    /* IWDG Clock is LSI Osc. */
  default:
    if (LL_RCC_LSI_IsReady() == 1U)
    {
      iwdg_frequency = LSI_VALUE;
    }
    break;
  }
  return iwdg_frequency;
}
#endif

#if defined(RCC_BDCR_RTCSEL)
/**
  * @brief  Return RTC clock frequency
  * @retval RTC clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillators (LSI, LSE or HSE) are not ready
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NA indicates that no clock source selected
  */
uint32_t LL_RCC_GetRTCClockFreq(void)
{
  uint32_t rtc_frequency = LL_RCC_PERIPH_FREQUENCY_NO;

  /* RTCCLK clock frequency */
  switch (LL_RCC_GetRTCClockSource())
  {
#if defined(RCC_LSE_SUPPORT)
  case LL_RCC_RTC_CLKSOURCE_LSE:              /* LSE clock used as RTC clock source */
    if (LL_RCC_LSE_IsReady() == 1U)
    {
      rtc_frequency = LSE_VALUE;
    }
    break;
#endif
  case LL_RCC_RTC_CLKSOURCE_LSI:              /* LSI clock used as RTC clock source */
    if (LL_RCC_LSI_IsReady() == 1U)
    {
      rtc_frequency = LSI_VALUE;
    }
    break;
  case LL_RCC_RTC_CLKSOURCE_HSE_DIV32:        /* HSE/32 clock used as RTC clock source */
    if (LL_RCC_HSE_IsReady() == 1U)
    {
      rtc_frequency = HSE_VALUE / 32U;
    }
    break;
  case LL_RCC_RTC_CLKSOURCE_HSE_DIV128:       /* HSE/128 clock used as RTC clock source */
    if (LL_RCC_HSE_IsReady() == 1U)
    {
      rtc_frequency = HSE_VALUE / 128U;
    }
    break;
  case LL_RCC_RTC_CLKSOURCE_HSE_DIV8:         /* HSE/8 clock used as RTC clock source */
    if (LL_RCC_HSE_IsReady() == 1U)
    {
      rtc_frequency = HSE_VALUE / 8U;
    }
    break;

  case LL_RCC_RTC_CLKSOURCE_NONE:             /* No clock used as RTC clock source */
  default:
    rtc_frequency = LL_RCC_PERIPH_FREQUENCY_NA;
    break;
  }
  return rtc_frequency;
}
#endif
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup RCC_LL_Private_Functions
  * @{
  */

/**
  * @brief  Return SYSTEM clock frequency
  * @retval SYSTEM clock frequency (in Hz)
  */
uint32_t RCC_GetSystemClockFreq(void)
{
  uint32_t frequency;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (LL_RCC_GetSysClkSource())
  {
  case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:     /* HSE used as system clock  source */
    frequency = HSE_VALUE;
    break;
  case LL_RCC_SYS_CLKSOURCE_STATUS_LSI:     /* LSI used as system clock  source */
    frequency = LSI_VALUE;
    break;
#if defined(RCC_LSE_SUPPORT)
  case LL_RCC_SYS_CLKSOURCE_STATUS_LSE:     /* LSE used as system clock  source */
    frequency = LSE_VALUE;
    break;
#endif
  case LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS:  /* HSISYS used as system clock  source */
  default:
    frequency = __LL_RCC_CALC_HSI_FREQ(LL_RCC_GetHSIDiv());
    break;
  }

  return frequency;
}

/**
  * @brief  Return HCLK clock frequency
  * @param  SYSCLK_Frequency SYSCLK clock frequency
  * @retval HCLK clock frequency (in Hz)
  */
uint32_t RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency)
{
  /* HCLK clock frequency */
  return __LL_RCC_CALC_HCLK_FREQ(SYSCLK_Frequency, LL_RCC_GetAHBPrescaler());
}

/**
  * @brief  Return PCLK1 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK1 clock frequency (in Hz)
  */
uint32_t RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK1 clock frequency */
  return __LL_RCC_CALC_PCLK1_FREQ(HCLK_Frequency, LL_RCC_GetAPB1Prescaler());
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(RCC) */

/**
  * @}
  */

#endif /* USE_FULL_LL_DRIVER */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

/**
  ******************************************************************************
  * @file    py32t020_ll_adc.h
  * @author  MCU Application Team
  * @brief   Header file of ADC LL module.
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
#ifndef __PY32T020_LL_ADC_H
#define __PY32T020_LL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32t0xx.h"
#include "py32t020_ll_bus.h"

/** @addtogroup PY32T020_LL_Driver
  * @{
  */

#if defined (ADC1)

/** @defgroup ADC_LL ADC
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup ADC_LL_Private_Constants ADC Private Constants
  * @{
  */

/* Internal mask for ADC group regular trigger:                               */
/* To select into literal LL_ADC_REG_TRIG_x the relevant bits for:            */
/* - regular trigger source                                                   */
/* - regular trigger edge                                                     */
#define ADC_REG_TRIG_EXT_EDGE_DEFAULT       (ADC_CFGR1_EXTEN_0)

/* Mask containing trigger source masks for each of possible                  */
/* trigger edge selection duplicated with shifts [0; 4; 8; 12]                */
/* corresponding to {SW start; ext trigger; ext trigger; ext trigger}.        */
#define ADC_REG_TRIG_SOURCE_MASK            (((LL_ADC_REG_TRIG_SOFTWARE & ADC_CFGR1_EXTSEL) << (4U * 0U)) | \
                                             ((                           ADC_CFGR1_EXTSEL) << (4U * 1U)) | \
                                             ((                           ADC_CFGR1_EXTSEL) << (4U * 2U)) | \
                                             ((                           ADC_CFGR1_EXTSEL) << (4U * 3U))  )

/* Mask containing trigger edge masks for each of possible                    */
/* trigger edge selection duplicated with shifts [0; 4; 8; 12]                */
/* corresponding to {SW start; ext trigger; ext trigger; ext trigger}.        */
#define ADC_REG_TRIG_EDGE_MASK              (((LL_ADC_REG_TRIG_SOFTWARE & ADC_CFGR1_EXTEN) << (4U * 0U)) | \
                                             ((ADC_REG_TRIG_EXT_EDGE_DEFAULT)              << (4U * 1U)) | \
                                             ((ADC_REG_TRIG_EXT_EDGE_DEFAULT)              << (4U * 2U)) | \
                                             ((ADC_REG_TRIG_EXT_EDGE_DEFAULT)              << (4U * 3U))  )

/* Definition of ADC group regular trigger bits information.                  */
#define ADC_REG_TRIG_EXTSEL_BITOFFSET_POS  ( 6U) /* Value equivalent to POSITION_VAL(ADC_CFGR1_EXTSEL) */
#define ADC_REG_TRIG_EXTEN_BITOFFSET_POS   (10U) /* Value equivalent to POSITION_VAL(ADC_CFGR1_EXTEN) */



/* Internal mask for ADC channel:                                             */
/* To select into literal LL_ADC_CHANNEL_x the relevant bits for:             */
/* - channel identifier defined by number                                     */
/* - channel identifier defined by bitfield                                   */
/* - channel differentiation between external channels (connected to          */
/*   GPIO pins) and internal channels (connected to internal paths)           */
#define ADC_CHANNEL_ID_NUMBER_MASK         (ADC_CFGR1_AWDCH)
#define ADC_CHANNEL_ID_BITFIELD_MASK       (ADC_CHSELR_CHSEL)
#define ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS (26U)/* Value equivalent to POSITION_VAL(ADC_CHANNEL_ID_NUMBER_MASK) */
#define ADC_CHANNEL_ID_MASK                (ADC_CHANNEL_ID_NUMBER_MASK | ADC_CHANNEL_ID_BITFIELD_MASK | ADC_CHANNEL_ID_INTERNAL_CH_MASK)
/* Equivalent mask of ADC_CHANNEL_NUMBER_MASK aligned on register LSB (bit 0) */
#define ADC_CHANNEL_ID_NUMBER_MASK_POSBIT0 (0x0000001FU) /* Equivalent to shift: (ADC_CHANNEL_NUMBER_MASK >> POSITION_VAL(ADC_CHANNEL_NUMBER_MASK)) */

/* Channel differentiation between external and internal channels */
#define ADC_CHANNEL_ID_INTERNAL_CH         (0x80000000U) /* Marker of internal channel */
#define ADC_CHANNEL_ID_INTERNAL_CH_MASK    (ADC_CHANNEL_ID_INTERNAL_CH)

/* Definition of channels ID number information to be inserted into           */
/* channels literals definition.                                              */
#define ADC_CHANNEL_0_NUMBER               (0x00000000U)
#define ADC_CHANNEL_1_NUMBER               (                                                            ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_2_NUMBER               (                                        ADC_CFGR1_AWDCH_1                    )
#define ADC_CHANNEL_3_NUMBER               (                                        ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_4_NUMBER               (                    ADC_CFGR1_AWDCH_2                                        )
#define ADC_CHANNEL_5_NUMBER               (                    ADC_CFGR1_AWDCH_2                     | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_6_NUMBER               (                    ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_1                    )
#define ADC_CHANNEL_7_NUMBER               (                    ADC_CFGR1_AWDCH_2 | ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_8_NUMBER               (ADC_CFGR1_AWDCH_3                                                            )
#define ADC_CHANNEL_9_NUMBER               (ADC_CFGR1_AWDCH_3                                         | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_10_NUMBER              (ADC_CFGR1_AWDCH_3                     | ADC_CFGR1_AWDCH_1                    )
#define ADC_CHANNEL_11_NUMBER              (ADC_CFGR1_AWDCH_3                     | ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_12_NUMBER              (ADC_CFGR1_AWDCH_3 | ADC_CFGR1_AWDCH_2                                        )
 
/* Definition of channels ID bitfield information to be inserted into         */
/* channels literals definition.                                              */
#define ADC_CHANNEL_0_BITFIELD             (ADC_CHSELR_CHSEL0)
#define ADC_CHANNEL_1_BITFIELD             (ADC_CHSELR_CHSEL1)
#define ADC_CHANNEL_2_BITFIELD             (ADC_CHSELR_CHSEL2)
#define ADC_CHANNEL_3_BITFIELD             (ADC_CHSELR_CHSEL3)
#define ADC_CHANNEL_4_BITFIELD             (ADC_CHSELR_CHSEL4)
#define ADC_CHANNEL_5_BITFIELD             (ADC_CHSELR_CHSEL5)
#define ADC_CHANNEL_6_BITFIELD             (ADC_CHSELR_CHSEL6)
#define ADC_CHANNEL_7_BITFIELD             (ADC_CHSELR_CHSEL7)
#define ADC_CHANNEL_8_BITFIELD             (ADC_CHSELR_CHSEL8)
#define ADC_CHANNEL_9_BITFIELD             (ADC_CHSELR_CHSEL9)
#define ADC_CHANNEL_10_BITFIELD            (ADC_CHSELR_CHSEL10)
#define ADC_CHANNEL_11_BITFIELD            (ADC_CHSELR_CHSEL11)
#define ADC_CHANNEL_12_BITFIELD            (ADC_CHSELR_CHSEL12)
 
/* Internal mask for ADC analog watchdog:                                     */
/* To select into literals LL_ADC_AWD_CHANNELx_xxx the relevant bits for:     */
/* - analog watchdog 1: monitored channel defined by number,                  */
/*   selection of ADC group (ADC group regular).                              */

/* Internal register offset for ADC analog watchdog channel configuration */
#define ADC_AWD_CR1_REGOFFSET              (0x00000000U)

#define ADC_AWD_CRX_REGOFFSET_MASK         (ADC_AWD_CR1_REGOFFSET)

#define ADC_AWD_CR1_CHANNEL_MASK           (ADC_CFGR1_AWDCH | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)
#define ADC_AWD_CR_ALL_CHANNEL_MASK        (ADC_AWD_CR1_CHANNEL_MASK)

/* Internal register offset for ADC analog watchdog threshold configuration */
#define ADC_AWD_TR1_REGOFFSET              (ADC_AWD_CR1_REGOFFSET)
#define ADC_AWD_TRX_REGOFFSET_MASK         (ADC_AWD_TR1_REGOFFSET)


/* ADC registers bits positions */
#define ADC_CFGR1_RES_BITOFFSET_POS        ( 3U) /* Value equivalent to POSITION_VAL(ADC_CFGR1_RESSEL) */
#define ADC_CFGR1_AWDSGL_BITOFFSET_POS     (22U) /* Value equivalent to POSITION_VAL(ADC_CFGR1_AWDSGL) */
#define ADC_TR_HT_BITOFFSET_POS            (16U) /* Value equivalent to POSITION_VAL(ADC_TR_HT) */
#define ADC_CHSELR_CHSEL0_BITOFFSET_POS    ( 0U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL0) */
#define ADC_CHSELR_CHSEL1_BITOFFSET_POS    ( 1U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL1) */
#define ADC_CHSELR_CHSEL2_BITOFFSET_POS    ( 2U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL2) */
#define ADC_CHSELR_CHSEL3_BITOFFSET_POS    ( 3U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL3) */
#define ADC_CHSELR_CHSEL4_BITOFFSET_POS    ( 4U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL4) */
#define ADC_CHSELR_CHSEL5_BITOFFSET_POS    ( 5U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL5) */
#define ADC_CHSELR_CHSEL6_BITOFFSET_POS    ( 6U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL6) */
#define ADC_CHSELR_CHSEL7_BITOFFSET_POS    ( 7U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL7) */
#define ADC_CHSELR_CHSEL8_BITOFFSET_POS    ( 8U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL8) */
#define ADC_CHSELR_CHSEL9_BITOFFSET_POS    ( 9U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL9) */
#define ADC_CHSELR_CHSEL10_BITOFFSET_POS   (10U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL10) */
#define ADC_CHSELR_CHSEL11_BITOFFSET_POS   (11U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL11) */
#define ADC_CHSELR_CHSEL12_BITOFFSET_POS   (12U) /* Value equivalent to POSITION_VAL(ADC_CHSELR_CHSEL12) */
 
/* ADC registers bits groups */
#define ADC_CR_BITS_PROPERTY_RS            (ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN) /* ADC register CR bits with HW property "rs": Software can read as well as set this bit. Writing '0' has no effect on the bit value. */


/* ADC internal channels related definitions */
/* Internal voltage reference VrefInt */
#define VREFINT_CAL_VREF                   ( 1200U)                    /* Analog voltage reference (Vref+) value (tolerance: +-10 mV) (unit: mV). */
/* Temperature sensor */
#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1fff0114))  /* Internal temperature sensor, address of parameter TS_CAL1: temperature sensor ADC raw data acquired at temperature  30 DegC (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV). */
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1fff0118))  /* Internal temperature sensor, address of parameter TS_CAL2: temperature sensor ADC raw data acquired at temperature  85 DegC (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV). */
#define TEMPSENSOR_CAL1_TEMP               (( int32_t)   30)           /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL1_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL2_TEMP               (( int32_t)   85)           /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL2_ADDR (tolerance: +-5 DegC) (unit: DegC). */
#define TEMPSENSOR_CAL_VREFANALOG          ( 3300U)                    /* Analog voltage reference (Vref+) voltage with which temperature sensor has been calibrated in production (+-10 mV) (unit: mV). */

/* Definitions of ADC hardware constraints delays */
/* Note: Only ADC IP HW delays are defined in ADC LL driver driver,           */
/*       not timeout values:                                                  */
/*       Timeout values for ADC operations are dependent to device clock      */
/*       configuration (system clock versus ADC clock),                       */
/*       and therefore must be defined in user application.                   */
/*       Refer to @ref ADC_LL_EC_HW_DELAYS for description of ADC timeout     */
/*       values definition.                                                   */
/* Unit: CPU cycles.                                                          */
#define LL_ADC_CLOCK_RATIO_VS_CPU_HIGHEST          ((uint32_t) 512U * 16U * 4U)
#define LL_ADC_TIMEOUT_DISABLE_CPU_CYCLES          (LL_ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1U)
#define LL_ADC_TIMEOUT_STOP_CONVERSION_CPU_CYCLES  (LL_ADC_CLOCK_RATIO_VS_CPU_HIGHEST * 1U)


/**
  * @}
  */


/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup ADC_LL_ES_INIT ADC Exported Init structure
  * @{
  */

/**
  * @brief  Structure definition of some features of ADC instance.
  * @note   These parameters have an impact on ADC scope: ADC instance.
  *         Refer to corresponding unitary functions into
  *         @ref ADC_LL_EF_Configuration_ADC_Instance .
  * @note   The setting of these parameters by function @ref LL_ADC_Init()
  *         is conditioned to ADC state:
  *         ADC instance must be disabled.
  *         Each feature can be updated afterwards with a unitary function
  *         and potentially with ADC in a different state than disabled,
  *         refer to description of each function for setting
  *         conditioned to ADC state.
  */
typedef struct
{
  uint32_t Clock;                       /*!< Set ADC instance clock source and prescaler.
                                             This parameter can be a value of @ref ADC_LL_EC_CLOCK_SOURCE
                                             This feature can be modified afterwards using unitary function @ref LL_ADC_SetClock().
                                             For more details, refer to description of this function. */

  uint32_t Resolution;                  /*!< Set ADC resolution.
                                             This parameter can be a value of @ref ADC_LL_EC_RESOLUTION
                                             This feature can be modified afterwards using unitary function @ref LL_ADC_SetResolution(). */

  uint32_t DataAlignment;               /*!< Set ADC conversion data alignment.
                                             This parameter can be a value of @ref ADC_LL_EC_DATA_ALIGN
                                             This feature can be modified afterwards using unitary function @ref LL_ADC_SetDataAlignment(). */

  uint32_t LowPowerMode;                /*!< Set ADC low power mode.
                                             This parameter can be a value of @ref ADC_LL_EC_LP_MODE
                                             This feature can be modified afterwards using unitary function @ref LL_ADC_SetLowPowerMode(). */

} LL_ADC_InitTypeDef;

/**
  * @brief  Structure definition of some features of ADC group regular.
  * @note   These parameters have an impact on ADC scope: ADC group regular.
  *         Refer to corresponding unitary functions into
  *         @ref ADC_LL_EF_Configuration_ADC_Group_Regular
  *         (functions with prefix "REG").
  */
typedef struct
{
  uint32_t TriggerSource;               /*!< Set ADC group regular conversion trigger source: internal (SW start) or from external IP (timer event).
                                             This parameter can be a value of @ref ADC_LL_EC_REG_TRIGGER_SOURCE
                                             @note setting trigger source to external trigger also set trigger polarity to rising edge
                                                   In case of need to modify trigger edge, use function @ref LL_ADC_REG_SetTriggerEdge().

                                             This feature can be modified afterwards using unitary function @ref LL_ADC_REG_SetTriggerSource(). */

  uint32_t SequencerDiscont;            /*!< Set ADC group regular sequencer discontinuous mode: sequence subdivided and scan conversions interrupted every selected number of ranks.
                                             This parameter can be a value of @ref ADC_LL_EC_REG_SEQ_DISCONT_MODE
                                             @note This parameter has an effect only if group regular sequencer is enabled
                                                   (several ADC channels enabled in group regular sequencer).

                                             This feature can be modified afterwards using unitary function @ref LL_ADC_REG_SetSequencerDiscont(). */

  uint32_t ContinuousMode;              /*!< Set ADC continuous conversion mode on ADC group regular, whether ADC conversions are performed in single mode (one conversion per trigger) or in continuous mode (after the first trigger, following conversions launched successively automatically).
                                             This parameter can be a value of @ref ADC_LL_EC_REG_CONTINUOUS_MODE
                                             Note: It is not possible to enable both ADC group regular continuous mode and discontinuous mode.

                                             This feature can be modified afterwards using unitary function @ref LL_ADC_REG_SetContinuousMode(). */

  uint32_t Overrun;                     /*!< Set ADC group regular behavior in case of overrun:
                                             data preserved or overwritten.
                                             This parameter can be a value of @ref ADC_LL_EC_REG_OVR_DATA_BEHAVIOR

                                             This feature can be modified afterwards using unitary function @ref LL_ADC_REG_SetOverrun(). */

} LL_ADC_REG_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup ADC_LL_Exported_Constants ADC Exported Constants
  * @{
  */

/** @defgroup ADC_LL_EC_FLAG ADC flags
  * @brief    Flags defines which can be used with LL_ADC_ReadReg function
  * @{ 
  */

#define LL_ADC_FLAG_EOC                    ADC_ISR_EOC        /*!< ADC flag ADC group regular end of unitary conversion */
#define LL_ADC_FLAG_EOS                    ADC_ISR_EOSEQ      /*!< ADC flag ADC group regular end of sequence conversions */
#define LL_ADC_FLAG_OVR                    ADC_ISR_OVR        /*!< ADC flag ADC group regular overrun */
#define LL_ADC_FLAG_EOSMP                  ADC_ISR_EOSMP      /*!< ADC flag ADC group regular end of sampling phase */
#define LL_ADC_FLAG_AWD                    ADC_ISR_AWD        /*!< ADC flag ADC analog watchdog 1 */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_IT ADC interruptions for configuration (interruption enable or disable)
  * @brief    IT defines which can be used with LL_ADC_ReadReg and  LL_ADC_WriteReg functions
  * @{ 
  */

#define LL_ADC_IT_EOC                      ADC_IER_EOCIE      /*!< ADC interruption ADC group regular end of unitary conversion */
#define LL_ADC_IT_EOS                      ADC_IER_EOSEQIE    /*!< ADC interruption ADC group regular end of sequence conversions */
#define LL_ADC_IT_OVR                      ADC_IER_OVRIE      /*!< ADC interruption ADC group regular overrun */
#define LL_ADC_IT_EOSMP                    ADC_IER_EOSMPIE    /*!< ADC interruption ADC group regular end of sampling phase */
#define LL_ADC_IT_AWD                      ADC_IER_AWDIE      /*!< ADC interruption ADC analog watchdog 1 */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_COMMON_PATH_INTERNAL  ADC common - Measurement path to internal channels
  * @{
  */
/* Note: Other measurement paths to internal channels may be available        */
/*       (connections to other peripherals).                                  */
/*       If they are not listed below, they do not require any specific       */
/*       path enable. In this case, Access to measurement path is done        */
/*       only by selecting the corresponding ADC internal channel.            */
#define LL_ADC_PATH_INTERNAL_NONE          (0x00000000U)          /*!< ADC measurement pathes all disabled */
#define LL_ADC_PATH_INTERNAL_VREFINT       (ADC_CCR_VREFEN)       /*!< ADC measurement path to internal channel VrefInt */
#define LL_ADC_PATH_INTERNAL_TEMPSENSOR    (ADC_CCR_TSEN)         /*!< ADC measurement path to internal channel temperature sensor */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_CLOCK_SOURCE  ADC instance - Clock source
  * @{
  */
#define LL_ADC_CLOCK_SYNC_PCLK_DIV1        (0x00000000U)                                                                                        /*!< ADC synchronous clock derived from APB clock divided by a prescaler of 1 */
#define LL_ADC_CLOCK_SYNC_PCLK_DIV2        (RCC_CCIPR_ADC_CKMODE_0                                                                           )  /*!< ADC synchronous clock derived from APB clock divided by a prescaler of 2 */
#define LL_ADC_CLOCK_SYNC_PCLK_DIV4        (                         RCC_CCIPR_ADC_CKMODE_1                                                  )  /*!< ADC synchronous clock derived from APB clock divided by a prescaler of 4 */
#define LL_ADC_CLOCK_SYNC_PCLK_DIV8        (RCC_CCIPR_ADC_CKMODE_0 | RCC_CCIPR_ADC_CKMODE_1                                                  )  /*!< ADC synchronous clock derived from APB clock divided by a prescaler of 8 */
#define LL_ADC_CLOCK_SYNC_PCLK_DIV16       (                                                  RCC_CCIPR_ADC_CKMODE_2                         )  /*!< ADC synchronous clock derived from APB clock divided by a prescaler of 16 */
#define LL_ADC_CLOCK_SYNC_PCLK_DIV32       (RCC_CCIPR_ADC_CKMODE_0                          | RCC_CCIPR_ADC_CKMODE_2                         )  /*!< ADC synchronous clock derived from APB clock divided by a prescaler of 32 */
#define LL_ADC_CLOCK_SYNC_PCLK_DIV64       (                         RCC_CCIPR_ADC_CKMODE_1 | RCC_CCIPR_ADC_CKMODE_2                         )  /*!< ADC synchronous clock derived from APB clock divided by a prescaler of 64 */
#define LL_ADC_CLOCK_ASYNC_HSI_DIV1        (                                                                           RCC_CCIPR_ADC_CKMODE_3)  /*!< ADC asynchronous clock derived from HSI clock divided by a prescaler of 1 */
#define LL_ADC_CLOCK_ASYNC_HSI_DIV2        (RCC_CCIPR_ADC_CKMODE_0                                                   | RCC_CCIPR_ADC_CKMODE_3)  /*!< ADC asynchronous clock derived from HSI clock divided by a prescaler of 2 */
#define LL_ADC_CLOCK_ASYNC_HSI_DIV4        (                         RCC_CCIPR_ADC_CKMODE_1                          | RCC_CCIPR_ADC_CKMODE_3)  /*!< ADC asynchronous clock derived from HSI clock divided by a prescaler of 4 */
#define LL_ADC_CLOCK_ASYNC_HSI_DIV8        (RCC_CCIPR_ADC_CKMODE_0 | RCC_CCIPR_ADC_CKMODE_1                          | RCC_CCIPR_ADC_CKMODE_3)  /*!< ADC asynchronous clock derived from HSI clock divided by a prescaler of 8 */
#define LL_ADC_CLOCK_ASYNC_HSI_DIV16       (                                                  RCC_CCIPR_ADC_CKMODE_2 | RCC_CCIPR_ADC_CKMODE_3)  /*!< ADC asynchronous clock derived from HSI clock divided by a prescaler of 16 */
#define LL_ADC_CLOCK_ASYNC_HSI_DIV32       (RCC_CCIPR_ADC_CKMODE_0 |                          RCC_CCIPR_ADC_CKMODE_2 | RCC_CCIPR_ADC_CKMODE_3)  /*!< ADC asynchronous clock derived from HSI clock divided by a prescaler of 32 */
#define LL_ADC_CLOCK_ASYNC_HSI_DIV64       (                         RCC_CCIPR_ADC_CKMODE_1 | RCC_CCIPR_ADC_CKMODE_2 | RCC_CCIPR_ADC_CKMODE_3)  /*!< ADC asynchronous clock derived from HSI clock divided by a prescaler of 64 */
/**
  * @}
  */
 
/** @defgroup ADC_LL_EC_RESOLUTION  ADC instance - Resolution
  * @{ 
  */
#define LL_ADC_RESOLUTION_12B              (0x00000000U)                             /*!< ADC resolution 12 bits */
#define LL_ADC_RESOLUTION_10B              (ADC_CFGR1_RESSEL_0)                      /*!< ADC resolution 10 bits */
#define LL_ADC_RESOLUTION_8B               (ADC_CFGR1_RESSEL_1)                      /*!< ADC resolution  8 bits */
#define LL_ADC_RESOLUTION_6B               (ADC_CFGR1_RESSEL_1 | ADC_CFGR1_RESSEL_0) /*!< ADC resolution  6 bits */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_DATA_ALIGN  ADC instance - Data alignment
  * @{
  */
#define LL_ADC_DATA_ALIGN_RIGHT            (0x00000000U)                             /*!< ADC conversion data alignment: right aligned (alignment on data register LSB bit 0)*/
#define LL_ADC_DATA_ALIGN_LEFT             (ADC_CFGR1_ALIGN)                         /*!< ADC conversion data alignment: left aligned (aligment on data register MSB bit 15)*/
/**
  * @}
  */

/** @defgroup ADC_LL_EC_LP_MODE  ADC instance - Low power mode
  * @{  
  */
#define LL_ADC_LP_MODE_NONE                (0x00000000U)                             /*!< No ADC low power mode activated */
#define LL_ADC_LP_AUTOWAIT                 (ADC_CFGR1_WAIT)                          /*!< ADC low power mode auto delay: Dynamic low power mode, ADC conversions are performed only when necessary (when previous ADC conversion data is read). See description with function @ref LL_ADC_SetLowPowerMode(). */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_GROUPS  ADC instance - Groups
  * @{
  */
#define LL_ADC_GROUP_REGULAR               (0x00000001U)                             /*!< ADC group regular */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_CHANNEL  ADC instance - Channel number
  * @{ 
  */
#define LL_ADC_CHANNEL_0                   (ADC_CHANNEL_0_NUMBER  | ADC_CHANNEL_0_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN0  */
#define LL_ADC_CHANNEL_1                   (ADC_CHANNEL_1_NUMBER  | ADC_CHANNEL_1_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN1  */
#define LL_ADC_CHANNEL_2                   (ADC_CHANNEL_2_NUMBER  | ADC_CHANNEL_2_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN2  */
#define LL_ADC_CHANNEL_3                   (ADC_CHANNEL_3_NUMBER  | ADC_CHANNEL_3_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN3  */
#define LL_ADC_CHANNEL_4                   (ADC_CHANNEL_4_NUMBER  | ADC_CHANNEL_4_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN4  */
#define LL_ADC_CHANNEL_5                   (ADC_CHANNEL_5_NUMBER  | ADC_CHANNEL_5_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN5  */
#define LL_ADC_CHANNEL_6                   (ADC_CHANNEL_6_NUMBER  | ADC_CHANNEL_6_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN6  */
#define LL_ADC_CHANNEL_7                   (ADC_CHANNEL_7_NUMBER  | ADC_CHANNEL_7_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN7  */
#define LL_ADC_CHANNEL_8                   (ADC_CHANNEL_8_NUMBER  | ADC_CHANNEL_8_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN8  */
#define LL_ADC_CHANNEL_9                   (ADC_CHANNEL_9_NUMBER  | ADC_CHANNEL_9_BITFIELD ) /*!< ADC external channel (channel connected to GPIO pin) ADCx_IN9  */
#define LL_ADC_CHANNEL_10                  (ADC_CHANNEL_10_NUMBER | ADC_CHANNEL_10_BITFIELD) 
#define LL_ADC_CHANNEL_11                  (ADC_CHANNEL_11_NUMBER | ADC_CHANNEL_11_BITFIELD)
#define LL_ADC_CHANNEL_12                  (ADC_CHANNEL_12_NUMBER | ADC_CHANNEL_12_BITFIELD) /*!< ADC channel (channel connected to VCC/3 ) ADCx_IN10            */

#define LL_ADC_CHANNEL_TEMPSENSOR          (LL_ADC_CHANNEL_10 | ADC_CHANNEL_ID_INTERNAL_CH)  /*!< ADC internal channel connected to Temperature sensor. */
#define LL_ADC_CHANNEL_VREFINT             (LL_ADC_CHANNEL_11 | ADC_CHANNEL_ID_INTERNAL_CH)  /*!< ADC internal channel connected to VrefInt: Internal voltage reference. */
#define LL_ADC_CHANNEL_1_3VCCA             (LL_ADC_CHANNEL_12 | ADC_CHANNEL_ID_INTERNAL_CH)  /*!< ADC internal channel connected to 1/3VCCA: Internal voltage reference. */
 
/**
  * @}
  */

/** @defgroup ADC_LL_VREFBUF_VOLTAGE  ADC instance - Vrefbuf Voltage
  * @{ 
  */
#define LL_ADC_VREFBUF_0P6V     (0x00000000U)                     /*!< VREFBUF Output voltage 0.6V.   */
#define LL_ADC_VREFBUF_1P5V     (ADC_CCR_VREFBUF_SEL_0)           /*!< VREFBUF Output voltage 1.5V.   */
#define LL_ADC_VREFBUF_2P048V   (ADC_CCR_VREFBUF_SEL_1)           /*!< VREFBUF Output voltage 2.048V. */
#define LL_ADC_VREFBUF_2P5V     (ADC_CCR_VREFBUF_SEL  )           /*!< VREFBUF Output voltage 2.5V.   */  
/**
  * @}
  */

/** @defgroup ADC_LL_EC_REG_TRIGGER_SOURCE  ADC group regular - Trigger source
  * @{ 
  */
#define LL_ADC_REG_TRIG_SOFTWARE           (0x00000000U)                                                             /*!< ADC group regular conversion trigger internal: SW start. */
#define LL_ADC_REG_TRIG_EXT_TIM1_TRGO      (                                          ADC_REG_TRIG_EXT_EDGE_DEFAULT) /*!< ADC group regular conversion trigger from external IP: TIM1 TRGO. Trigger edge set to rising edge (default setting). */
#define LL_ADC_REG_TRIG_EXT_TIM1_CH4       (                     ADC_CFGR1_EXTSEL_0 | ADC_REG_TRIG_EXT_EDGE_DEFAULT) /*!< ADC group regular conversion trigger from external IP: TIM1 channel 4 event (capture compare: input capture or output capture). Trigger edge set to rising edge (default setting). */
#define LL_ADC_REG_TRIG_EXT_TIM14_CH1      (                     ADC_CFGR1_EXTSEL_1 | ADC_REG_TRIG_EXT_EDGE_DEFAULT) /*!< ADC group regular conversion trigger from external IP: TIM14 channel 1 event (capture compare: input capture or output capture). Trigger edge set to rising edge (default setting). */
/**
  * @}
  */  

/** @defgroup ADC_LL_EC_REG_TRIGGER_EDGE  ADC group regular - Trigger edge
  * @{
  */
#define LL_ADC_REG_TRIG_EXT_RISING         (                    ADC_CFGR1_EXTEN_0) /*!< ADC group regular conversion trigger polarity set to rising edge */
#define LL_ADC_REG_TRIG_EXT_FALLING        (ADC_CFGR1_EXTEN_1                    ) /*!< ADC group regular conversion trigger polarity set to falling edge */
#define LL_ADC_REG_TRIG_EXT_RISINGFALLING  (ADC_CFGR1_EXTEN_1 | ADC_CFGR1_EXTEN_0) /*!< ADC group regular conversion trigger polarity set to both rising and falling edges */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_REG_CONTINUOUS_MODE  ADC group regular - Continuous mode
* @{
*/
#define LL_ADC_REG_CONV_SINGLE             (0x00000000U)                           /*!< ADC conversions are performed in single mode: one conversion per trigger */
#define LL_ADC_REG_CONV_CONTINUOUS         (ADC_CFGR1_CONT)                        /*!< ADC conversions are performed in continuous mode: after the first trigger, following conversions launched successively automatically */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_REG_OVR_DATA_BEHAVIOR  ADC group regular - Overrun behavior on conversion data
* @{
*/
#define LL_ADC_REG_OVR_DATA_PRESERVED      (0x00000000U)                           /*!< ADC group regular behavior in case of overrun: data preserved */
#define LL_ADC_REG_OVR_DATA_OVERWRITTEN    (ADC_CFGR1_OVRMOD)                      /*!< ADC group regular behavior in case of overrun: data overwritten */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_REG_SEQ_SCAN_DIRECTION  ADC group regular - Sequencer scan direction
  * @{
  */
#define LL_ADC_REG_SEQ_SCAN_DIR_FORWARD    (0x00000000U)                           /*!< ADC group regular sequencer scan direction forward: from lowest channel number to highest channel number (scan of all ranks, ADC conversion of ranks with channels enabled in sequencer).*/
#define LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD   (ADC_CFGR1_SCANDIR)                     /*!< ADC group regular sequencer scan direction backward: from highest channel number to lowest channel number (scan of all ranks, ADC conversion of ranks with channels enabled in sequencer) */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_REG_SEQ_DISCONT_MODE  ADC group regular - Sequencer discontinuous mode
  * @{
  */
#define LL_ADC_REG_SEQ_DISCONT_DISABLE     (0x00000000U)                           /*!< ADC group regular sequencer discontinuous mode disable */
#define LL_ADC_REG_SEQ_DISCONT_1RANK       (ADC_CFGR1_DISCEN)                      /*!< ADC group regular sequencer discontinuous mode enable with sequence interruption every rank */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_CHANNEL_SAMPLINGTIME  Channel - Sampling time
  * @{  
  */
#define LL_ADC_SAMPLINGTIME_3CYCLES_5      (0x00000000U)                                         /*!< Sampling time 3.5 ADC clock cycles */
#define LL_ADC_SAMPLINGTIME_5CYCLES_5      (                                  ADC_SMPR_SMP_0)    /*!< Sampling time 5.5 ADC clock cycles */
#define LL_ADC_SAMPLINGTIME_7CYCLES_5      (                 ADC_SMPR_SMP_1                 )    /*!< Sampling time 7.5 ADC clock cycles */
#define LL_ADC_SAMPLINGTIME_13CYCLES_5     (                 ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0)    /*!< Sampling time 13.5 ADC clock cycles */
#define LL_ADC_SAMPLINGTIME_28CYCLES_5     (ADC_SMPR_SMP_2                                  )    /*!< Sampling time 28.5 ADC clock cycles */
#define LL_ADC_SAMPLINGTIME_41CYCLES_5     (ADC_SMPR_SMP_2                  | ADC_SMPR_SMP_0)    /*!< Sampling time 41.5 ADC clock cycles */
#define LL_ADC_SAMPLINGTIME_134CYCLES_5    (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1                 )    /*!< Sampling time 134.5 ADC clock cycles */
#define LL_ADC_SAMPLINGTIME_239CYCLES_5    (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0)    /*!< Sampling time 239.5 ADC clock cycles */
/**
  * @}
  */   

/** @defgroup ADC_LL_EC_AWD_NUMBER Analog watchdog - Analog watchdog number
  * @{
  */
#define LL_ADC_AWD                        (ADC_AWD_CR1_CHANNEL_MASK  | ADC_AWD_CR1_REGOFFSET)    /*!< ADC analog watchdog number 1 */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_AWD_CHANNELS  Analog watchdog - Monitored channels
  * @{   
  */

#define LL_ADC_AWD_DISABLE                 (0x00000000U)                                                                      /*!< ADC analog watchdog monitoring disabled */
#define LL_ADC_AWD_ALL_CHANNELS_REG        (                                            ADC_CFGR1_AWDEN                   )   /*!< ADC analog watchdog monitoring of all channels, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_0_REG           ((LL_ADC_CHANNEL_0  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN1, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_1_REG           ((LL_ADC_CHANNEL_1  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN2, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_2_REG           ((LL_ADC_CHANNEL_2  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN3, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_3_REG           ((LL_ADC_CHANNEL_3  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN4, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_4_REG           ((LL_ADC_CHANNEL_4  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN5, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_5_REG           ((LL_ADC_CHANNEL_5  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN6, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_6_REG           ((LL_ADC_CHANNEL_6  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN7, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_7_REG           ((LL_ADC_CHANNEL_7  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN8, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_8_REG           ((LL_ADC_CHANNEL_8  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN9, converted by group regular only */
#define LL_ADC_AWD_CHANNEL_9_REG           ((LL_ADC_CHANNEL_9  & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC external channel (channel connected to GPIO pin) ADCx_IN11, converted by group regular only */
#define LL_ADC_AWD_CH_TEMPSENSOR_REG       ((LL_ADC_CHANNEL_10 & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC internal channel connected to Temperature sensor, converted by group regular only */
#define LL_ADC_AWD_CH_VREFINT_REG          ((LL_ADC_CHANNEL_11 & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC internal channel connected to VrefInt: Internal voltage reference, converted by group regular only */
#define LL_ADC_AWD_CH_1_3VCCA_REG          ((LL_ADC_CHANNEL_12 & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)   /*!< ADC analog watchdog monitoring of ADC internal channel connected to VrefInt: Internal voltage reference, converted by group regular only */


/**
  * @}
  */   

/** @defgroup ADC_LL_EC_AWD_THRESHOLDS  Analog watchdog - Thresholds
  * @{
  */
#define LL_ADC_AWD_THRESHOLD_HIGH          (ADC_TR_HT            )     /*!< ADC analog watchdog threshold high */
#define LL_ADC_AWD_THRESHOLD_LOW           (            ADC_TR_LT)     /*!< ADC analog watchdog threshold low */
#define LL_ADC_AWD_THRESHOLDS_HIGH_LOW     (ADC_TR_HT | ADC_TR_LT)     /*!< ADC analog watchdog both thresholds high and low concatenated into the same data */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_CAL_SAMPLINGTIME  Calibration - Sampling time
  * @{
  */
#define LL_ADC_CAL_SAMPLINGTIME_1P5CYCLE     (0x00000000U)                              /*!< CAL Sampling time 1.5 ADC clock cycle */
#define LL_ADC_CAL_SAMPLINGTIME_2P5CYCLES    (                    ADC_CCSR_CALSMP_0)    /*!< CAL Sampling time 2.5 ADC clock cycles */
#define LL_ADC_CAL_SAMPLINGTIME_4P5CYCLES    (ADC_CCSR_CALSMP_1                    )    /*!< CAL Sampling time 4.5 ADC clock cycles */
#define LL_ADC_CAL_SAMPLINGTIME_8P5CYCLES    (ADC_CCSR_CALSMP_1 | ADC_CCSR_CALSMP_0)    /*!< CAL Sampling time 8.5 ADC clock cycles */
/**
  * @}
  */   

/** @defgroup ADC_LL_EC_CAL_MODE  Calibration - Mode
  * @{
  */
#define LL_ADC_CAL_MODE_OFFSET                (0x00000000U)                          /*!< ADC calibration mode selection only OFFSET */
#define LL_ADC_CAL_MODE_OFFSET_CAPACITANCE    ADC_CCSR_CALSEL                        /*!< ADC calibration mode selection OFFSET AND CAPACITANCE */
/**
  * @}
  */

/** @defgroup ADC_LL_EC_CAL_STATUS  Calibration - Status
  * @{
  */
#define LL_ADC_CAL_OFFSET_STATUS_FAIL          (0x00000000)          /*!< ADC calibration status OFFSET FAIL */
#define LL_ADC_CAL_OFFSET_STATUS_SUCCESS       (ADC_CCSR_OFFSUC)     /*!< ADC calibration status OFFSET SUCCESS */

#define LL_ADC_CAL_CAPACITANCE_STATUS_FAIL     (0x00000000)          /*!< ADC calibration status CAPACITANCE FAIL */
#define LL_ADC_CAL_CAPACITANCE_STATUS_SUCCESS  (ADC_CCSR_CAPSUC)     /*!< ADC calibration status CAPACITANCE SUCCESS */
/**
  * @}
  */





/** @defgroup ADC_LL_EC_HW_DELAYS  Definitions of ADC hardware constraints delays
  * @note   Only ADC IP HW delays are defined in ADC LL driver driver,
  *         not timeout values.
  *         For details on delays values, refer to descriptions in source code
  *         above each literal definition.
  * @{
  */

/* Delay for internal voltage reference stabilization time.                   */
/* Delay set to maximum value (refer to device datasheet,                     */
/* parameter "tSTART").                                                       */
/* Unit: us                                                                   */
#define LL_ADC_DELAY_VREFINT_STAB_US       (  10U)  /*!< Delay for internal voltage reference stabilization time */

/* Delay for temperature sensor stabilization time.                           */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tSTART").                                                       */
/* Unit: us                                                                   */
#define LL_ADC_DELAY_TEMPSENSOR_STAB_US    (  10U)  /*!< Delay for temperature sensor stabilization time */

/* Delay required between ADC end of calibration and ADC enable.              */
/* Unit: ADC clock cycles.                                                    */
#define LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES ( 4U)  /*!< Delay required between ADC end of calibration and ADC enable */

 /* Delay between ADC end of calibration and ADC enable.                     */
 /* Delay estimation in CPU cycles: Case of ADC enable done                  */
 /* immediately after ADC calibration, ADC clock setting slow                */
 /* (CPU clock / ADC clock) is above 64.                                     */
#define LL_ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 64)

 /* Timeout values for ADC Calibration.   */
#define LL_ADC_CALIBRATION_TIMEOUT_CPU_CYCLES ( 1000U)
/**
  * @}
  */

/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/** @defgroup ADC_LL_Exported_Macros ADC Exported Macros
  * @{
  */

/** @defgroup ADC_LL_EM_WRITE_READ Common write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in ADC register
  * @param  __INSTANCE__ ADC Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_ADC_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in ADC register
  * @param  __INSTANCE__ ADC Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_ADC_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup ADC_LL_EM_HELPER_MACRO ADC helper macro
  * @{
  */

/**
  * @brief  Helper macro to get ADC channel number in decimal format
  *         from literals LL_ADC_CHANNEL_x.
  * @note   Example:
  *           __LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_CHANNEL_4)
  *           will return decimal number "4".
  * @note   The input can be a value from functions where a channel
  *         number is returned, either defined with number
  *         or with bitfield (only one bit must be set).
  * @note   Depending on devices and packages, some channels may not be available.
  *         Refer to device datasheet for channels availability.
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  *         @arg @ref LL_ADC_CHANNEL_VREFINT
  *         @arg @ref LL_ADC_CHANNEL_TEMPSENSOR
  *         @arg @ref LL_ADC_CHANNEL_1_3VCCA
  * @retval Value between Min_Data=0 and Max_Data=12 
  */
#define __LL_ADC_CHANNEL_TO_DECIMAL_NB(__CHANNEL__)                                                               \
  ((((__CHANNEL__) & ADC_CHANNEL_ID_BITFIELD_MASK) == 0U)                                                         \
    ? (                                                                                                           \
       ((__CHANNEL__) & ADC_CHANNEL_ID_NUMBER_MASK) >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS                        \
      )                                                                                                           \
      :                                                                                                           \
      (                                                                                                           \
       (((__CHANNEL__) & ADC_CHSELR_CHSEL0) == ADC_CHSELR_CHSEL0) ? (0U) :                                        \
        (                                                                                                         \
         (((__CHANNEL__) & ADC_CHSELR_CHSEL1) == ADC_CHSELR_CHSEL1) ? (1U) :                                      \
          (                                                                                                       \
           (((__CHANNEL__) & ADC_CHSELR_CHSEL2) == ADC_CHSELR_CHSEL2) ? (2U) :                                    \
            (                                                                                                     \
             (((__CHANNEL__) & ADC_CHSELR_CHSEL3) == ADC_CHSELR_CHSEL3) ? (3U) :                                  \
              (                                                                                                   \
               (((__CHANNEL__) & ADC_CHSELR_CHSEL4) == ADC_CHSELR_CHSEL4) ? (4U) :                                \
                (                                                                                                 \
                 (((__CHANNEL__) & ADC_CHSELR_CHSEL5) == ADC_CHSELR_CHSEL5) ? (5U) :                              \
                  (                                                                                               \
                   (((__CHANNEL__) & ADC_CHSELR_CHSEL6) == ADC_CHSELR_CHSEL6) ? (6U) :                            \
                    (                                                                                             \
                     (((__CHANNEL__) & ADC_CHSELR_CHSEL7) == ADC_CHSELR_CHSEL7) ? (7U) :                          \
                      (                                                                                           \
                       (((__CHANNEL__) & ADC_CHSELR_CHSEL8) == ADC_CHSELR_CHSEL8) ? (8U) :                        \
                        (                                                                                         \
                         (((__CHANNEL__) & ADC_CHSELR_CHSEL9) == ADC_CHSELR_CHSEL9) ? (9U) :                      \
                          (                                                                                       \
                           (((__CHANNEL__) & ADC_CHSELR_CHSEL10) == ADC_CHSELR_CHSEL10) ? (10U) :                 \
                            (                                                                                     \
                             (((__CHANNEL__) & ADC_CHSELR_CHSEL11) == ADC_CHSELR_CHSEL11) ? (11U) :               \
                              (                                                                                   \
                               (((__CHANNEL__) & ADC_CHSELR_CHSEL12) == ADC_CHSELR_CHSEL12) ? (12U) :             \
                                     (0U)                                                                         \
                              )                                                                                   \
                            )                                                                                     \
                          )                                                                                       \
                        )                                                                                         \
                      )                                                                                           \
                    )                                                                                             \
                  )                                                                                               \
                )                                                                                                 \
              )                                                                                                   \
            )                                                                                                     \
          )                                                                                                       \
        )                                                                                                         \
      )                                                                                                           \
  )


 
/**
  * @brief  Helper macro to get ADC channel in literal format LL_ADC_CHANNEL_x
  *         from number in decimal format.
  * @note   Example:
  *           __LL_ADC_DECIMAL_NB_TO_CHANNEL(4)
  *           will return a data equivalent to "LL_ADC_CHANNEL_4".
  * @note   Depending on devices and packages, some channels may not be available.
  *         Refer to device datasheet for channels availability.
  * @param  __DECIMAL_NB__ Value between Min_Data=0 and Max_Data=10
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  *         @arg @ref LL_ADC_CHANNEL_VREFINT       (1)
  *         @arg @ref LL_ADC_CHANNEL_TEMPSENSOR    (1)
  *         @arg @ref LL_ADC_CHANNEL_1_3VCCA       (1)
  *         (1) For ADC channel read back from ADC register,
  *             comparison with internal channel parameter to be done
  *             using helper macro @ref __LL_ADC_CHANNEL_INTERNAL_TO_EXTERNAL().
  */
#define __LL_ADC_DECIMAL_NB_TO_CHANNEL(__DECIMAL_NB__)                         \
  (                                                                            \
   ((__DECIMAL_NB__) << ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS) |                 \
   (ADC_CHSELR_CHSEL0 << (__DECIMAL_NB__))                                     \
  )

/**
  * @brief  Helper macro to determine whether the selected channel
  *         corresponds to literal definitions of driver.
  * @note   The different literal definitions of ADC channels are:
  *         - ADC internal channel:
  *           LL_ADC_CHANNEL_VREFINT, LL_ADC_CHANNEL_TEMPSENSOR, ...
  *         - ADC external channel (channel connected to a GPIO pin):
  *           LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_2, ...
  * @note   Depending on devices and packages, some channels may not be available.
  *         Refer to device datasheet for channels availability.
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  *         @arg @ref LL_ADC_CHANNEL_VREFINT
  *         @arg @ref LL_ADC_CHANNEL_TEMPSENSOR
  *         @arg @ref LL_ADC_CHANNEL_1_3VCCA
  * @retval Value "0" if the channel corresponds to a parameter definition of a ADC external channel (channel connected to a GPIO pin).
  *         Value "1" if the channel corresponds to a parameter definition of a ADC internal channel.
  */
#define __LL_ADC_IS_CHANNEL_INTERNAL(__CHANNEL__)                              \
  (((__CHANNEL__) & ADC_CHANNEL_ID_INTERNAL_CH_MASK) != 0U)

/**
  * @brief  Helper macro to convert a channel defined from parameter
  *         definition of a ADC internal channel (LL_ADC_CHANNEL_VREFINT,
  *         LL_ADC_CHANNEL_TEMPSENSOR, ...),
  *         to its equivalent parameter definition of a ADC external channel
  *         (LL_ADC_CHANNEL_1, LL_ADC_CHANNEL_2, ...).
  * @note   Depending on devices and packages, some channels may not be available.
  *         Refer to device datasheet for channels availability.
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CHANNEL_VREFINT
  *         @arg @ref LL_ADC_CHANNEL_TEMPSENSOR
  *         @arg @ref LL_ADC_CHANNEL_1_3VCCA
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  */
#define __LL_ADC_CHANNEL_INTERNAL_TO_EXTERNAL(__CHANNEL__)                     \
  ((__CHANNEL__) & ~ADC_CHANNEL_ID_INTERNAL_CH_MASK)

/**
  * @brief  Helper macro to determine whether the internal channel
  *         selected is available on the ADC instance selected.
  * @param  __ADC_INSTANCE__ ADC instance
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CHANNEL_VREFINT
  *         @arg @ref LL_ADC_CHANNEL_TEMPSENSOR
  *         @arg @ref LL_ADC_CHANNEL_1_3VCCA
  * @retval Value "0" if the internal channel selected is not available on the ADC instance selected.
  *         Value "1" if the internal channel selected is available on the ADC instance selected.  
  */
#define __LL_ADC_IS_CHANNEL_INTERNAL_AVAILABLE(__ADC_INSTANCE__, __CHANNEL__)  \
  (                                                                            \
    ((__CHANNEL__) == LL_ADC_CHANNEL_VREFINT)    ||                            \
    ((__CHANNEL__) == LL_ADC_CHANNEL_TEMPSENSOR) ||                            \
    ((__CHANNEL__) == LL_ADC_CHANNEL_1_3VCCA)                                  \
  )
 
/**
  * @brief  Helper macro to define ADC analog watchdog parameter:
  *         define a single channel to monitor with analog watchdog
  *         from sequencer channel and groups definition.
  * @note   To be used with function @ref LL_ADC_SetAnalogWDMonitChannels().
  *         Example:
  *           LL_ADC_SetAnalogWDMonitChannels(
  *             ADC1, LL_ADC_AWD,
  *             __LL_ADC_ANALOGWD_CHANNEL_GROUP(LL_ADC_CHANNEL4, LL_ADC_GROUP_REGULAR))
  * @note   Depending on devices and packages, some channels may not be available.
  *         Refer to device datasheet for channels availability.
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  *         @arg @ref LL_ADC_CHANNEL_VREFINT       (1)
  *         @arg @ref LL_ADC_CHANNEL_TEMPSENSOR    (1)
  *         @arg @ref LL_ADC_CHANNEL_1_3VCCA       (1)
  *         (1) For ADC channel read back from ADC register,
  *             comparison with internal channel parameter to be done
  *             using helper macro @ref __LL_ADC_CHANNEL_INTERNAL_TO_EXTERNAL().
  * @param  __GROUP__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_GROUP_REGULAR
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_AWD_DISABLE
  *         @arg @ref LL_ADC_AWD_ALL_CHANNELS_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_0_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_1_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_2_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_3_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_4_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_5_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_6_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_7_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_8_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_9_REG
  *         @arg @ref LL_ADC_AWD_CH_VREFINT_REG
  *         @arg @ref LL_ADC_AWD_CH_TEMPSENSOR_REG
  *         @arg @ref LL_ADC_AWD_CH_1_3VCCA_REG
  */
#define __LL_ADC_ANALOGWD_CHANNEL_GROUP(__CHANNEL__, __GROUP__)\
  (((__CHANNEL__) & ADC_CHANNEL_ID_MASK) | ADC_CFGR1_AWDEN | ADC_CFGR1_AWDSGL)

/**
  * @brief  Helper macro to set the value of ADC analog watchdog threshold high
  *         or low in function of ADC resolution, when ADC resolution is
  *         different of 12 bits.
  * @note   To be used with function @ref LL_ADC_ConfigAnalogWDThresholds()
  *         or @ref LL_ADC_SetAnalogWDThresholds().
  *         Example, with a ADC resolution of 8 bits, to set the value of
  *         analog watchdog threshold high (on 8 bits):
  *           LL_ADC_SetAnalogWDThresholds
  *            (< ADCx param >,
  *             __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(LL_ADC_RESOLUTION_8B, <threshold_value_8_bits>)
  *            );
  * @param  __ADC_RESOLUTION__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @param  __AWD_THRESHOLD__ Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval Value between Min_Data=0x000 and Max_Data=0xFFF
  */
#define __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION(__ADC_RESOLUTION__, __AWD_THRESHOLD__) \
  ((__AWD_THRESHOLD__) << ((__ADC_RESOLUTION__) >> (ADC_CFGR1_RES_BITOFFSET_POS - 1U )))

/**
  * @brief  Helper macro to get the value of ADC analog watchdog threshold high
  *         or low in function of ADC resolution, when ADC resolution is
  *         different of 12 bits.
  * @note   To be used with function @ref LL_ADC_GetAnalogWDThresholds().
  *         Example, with a ADC resolution of 8 bits, to get the value of
  *         analog watchdog threshold high (on 8 bits):
  *           < threshold_value_6_bits > = __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION
  *            (LL_ADC_RESOLUTION_8B,
  *             LL_ADC_GetAnalogWDThresholds(<ADCx param>, LL_ADC_AWD_THRESHOLD_HIGH)
  *            );
  * @param  __ADC_RESOLUTION__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @param  __AWD_THRESHOLD_12_BITS__ Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval Value between Min_Data=0x000 and Max_Data=0xFFF
  */
#define __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION(__ADC_RESOLUTION__, __AWD_THRESHOLD_12_BITS__) \
  ((__AWD_THRESHOLD_12_BITS__) >> ((__ADC_RESOLUTION__) >> (ADC_CFGR1_RES_BITOFFSET_POS - 1U )))

/**
  * @brief  Helper macro to get the ADC analog watchdog threshold high
  *         or low from raw value containing both thresholds concatenated.
  * @note   To be used with function @ref LL_ADC_GetAnalogWDThresholds().
  *         Example, to get analog watchdog threshold high from the register raw value:
  *           __LL_ADC_ANALOGWD_THRESHOLDS_HIGH_LOW(LL_ADC_AWD_THRESHOLD_HIGH, <raw_value_with_both_thresholds>);
  * @param  __AWD_THRESHOLD_TYPE__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_AWD_THRESHOLD_HIGH
  *         @arg @ref LL_ADC_AWD_THRESHOLD_LOW
  * @param  __AWD_THRESHOLDS__ Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  * @retval Value between Min_Data=0x000 and Max_Data=0xFFF
  */
#define __LL_ADC_ANALOGWD_THRESHOLDS_HIGH_LOW(__AWD_THRESHOLD_TYPE__, __AWD_THRESHOLDS__) \
  (((__AWD_THRESHOLD_TYPE__) == LL_ADC_AWD_THRESHOLD_LOW)                                 \
    ? (                                                                                   \
       (__AWD_THRESHOLDS__) & LL_ADC_AWD_THRESHOLD_LOW                                    \
      )                                                                                   \
      :                                                                                   \
      (                                                                                   \
       ((__AWD_THRESHOLDS__) >> ADC_TR_HT_BITOFFSET_POS) & LL_ADC_AWD_THRESHOLD_LOW       \
      )                                                                                   \
  )

/**
  * @brief  Helper macro to select the ADC common instance
  *         to which is belonging the selected ADC instance.
  * @note   ADC common register instance can be used for:
  *         - Set parameters common to several ADC instances
  *         - Multimode (for devices with several ADC instances)
  *         Refer to functions having argument "ADCxy_COMMON" as parameter.
  * @param  __ADCx__ ADC instance
  * @retval ADC common register instance
  */
#define __LL_ADC_COMMON_INSTANCE(__ADCx__)                                     \
  (ADC1_COMMON)

/**
  * @brief  Helper macro to check if all ADC instances sharing the same
  *         ADC common instance are disabled.
  * @note   This check is required by functions with setting conditioned to
  *         ADC state:
  *         All ADC instances of the ADC common group must be disabled.
  *         Refer to functions having argument "ADCxy_COMMON" as parameter.
  * @note   On devices with only 1 ADC common instance, parameter of this macro
  *         is useless and can be ignored (parameter kept for compatibility
  *         with devices featuring several ADC common instances).
  * @param  __ADCXY_COMMON__ ADC common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __LL_ADC_COMMON_INSTANCE() )
  * @retval Value "0" if all ADC instances sharing the same ADC common instance
  *         are disabled.
  *         Value "1" if at least one ADC instance sharing the same ADC common instance
  *         is enabled.
  */
#define __LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(__ADCXY_COMMON__)              \
  LL_ADC_IsEnabled(ADC1)

/**
  * @brief  Helper macro to define the ADC conversion data full-scale digital
  *         value corresponding to the selected ADC resolution.
  * @note   ADC conversion data full-scale corresponds to voltage range
  *         determined by analog voltage references Vref+ and Vref-
  *         (refer to reference manual).
  * @param  __ADC_RESOLUTION__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @retval ADC conversion data equivalent voltage value (unit: mVolt)
  */
#define __LL_ADC_DIGITAL_SCALE(__ADC_RESOLUTION__)                             \
  (0xFFFU >> ((__ADC_RESOLUTION__) >> (ADC_CFGR1_RES_BITOFFSET_POS - 1U)))

/**
  * @brief  Helper macro to convert the ADC conversion data from
  *         a resolution to another resolution.
  * @param  __DATA__ ADC conversion data to be converted
  * @param  __ADC_RESOLUTION_CURRENT__ Resolution of to the data to be converted
  *         This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @param  __ADC_RESOLUTION_TARGET__ Resolution of the data after conversion
  *         This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @retval ADC conversion data to the requested resolution
  */
#define __LL_ADC_CONVERT_DATA_RESOLUTION(__DATA__, __ADC_RESOLUTION_CURRENT__, __ADC_RESOLUTION_TARGET__) \
  (((__DATA__)                                                                 \
    << ((__ADC_RESOLUTION_CURRENT__) >> (ADC_CFGR1_RES_BITOFFSET_POS - 1U)))   \
   >> ((__ADC_RESOLUTION_TARGET__) >> (ADC_CFGR1_RES_BITOFFSET_POS - 1U))      \
  )

/**
  * @brief  Helper macro to calculate the voltage (unit: mVolt)
  *         corresponding to a ADC conversion data (unit: digital value).
  * @note   Analog reference voltage (Vref+) must be either known from
  *         user board environment or can be calculated using ADC measurement
  *         and ADC helper macro @ref __LL_ADC_CALC_VREFANALOG_VOLTAGE().
  * @param  __VREFANALOG_VOLTAGE__ Analog reference voltage (unit: mV)
  * @param  __ADC_DATA__ ADC conversion data (resolution 12 bits)
  *                       (unit: digital value).
  * @param  __ADC_RESOLUTION__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @retval ADC conversion data equivalent voltage value (unit: mVolt)
  */
#define __LL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,__ADC_DATA__,__ADC_RESOLUTION__)                      \
  ((__ADC_DATA__) * (__VREFANALOG_VOLTAGE__)/ __LL_ADC_DIGITAL_SCALE(__ADC_RESOLUTION__))

/**
  * @brief  Helper macro to calculate analog reference voltage (Vref+)
  *         (unit: mVolt) from ADC conversion data of internal voltage
  *         reference VrefInt.
  * @note   Computation is using VrefInt calibration value
  *         stored in system memory for each device during production.
  * @note   This voltage depends on user board environment: voltage level
  *         connected to pin Vref+.
  *         On devices with small package, the pin Vref+ is not present
  *         and internally bonded to pin Vdda.
  * @param  __VREFINT_ADC_DATA__ ADC conversion data (resolution 12 bits)
  *         of internal voltage reference VrefInt (unit: digital value).
  * @param  __ADC_RESOLUTION__ This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @retval Analog reference voltage (unit: mV)       
  */
#define __LL_ADC_CALC_VREFANALOG_VOLTAGE(__VREFINT_ADC_DATA__,__ADC_RESOLUTION__)                   \
  (4095*( VREFINT_CAL_VREF)/ __LL_ADC_CONVERT_DATA_RESOLUTION((__VREFINT_ADC_DATA__),(__ADC_RESOLUTION__),LL_ADC_RESOLUTION_12B))

/**
  * @brief  Helper macro to calculate the temperature (unit: degree Celsius)
  *         from ADC conversion data of internal temperature sensor.
  * @note   Computation is using temperature sensor calibration values
  *         stored in system memory for each device during production.
  * @note   Calculation formula:
  *           Temperature = ((TS_ADC_DATA - TS_CAL1)
  *                           * (TS_CAL2_TEMP - TS_CAL1_TEMP))
  *                         / (TS_CAL2 - TS_CAL1) + TS_CAL1_TEMP
  *           with TS_ADC_DATA = temperature sensor raw data measured by ADC
  *                Avg_Slope = (TS_CAL2 - TS_CAL1)
  *                            / (TS_CAL2_TEMP - TS_CAL1_TEMP)
  *                TS_CAL1   = equivalent TS_ADC_DATA at temperature
  *                            TEMP_DEGC_CAL1 (calibrated in factory)
  *                TS_CAL2   = equivalent TS_ADC_DATA at temperature
  *                            TEMP_DEGC_CAL2 (calibrated in factory)
  *         Caution: Calculation relevancy under reserve that calibration
  *                  parameters are correct (address and data).
  *                  To calculate temperature using temperature sensor
  *                  datasheet typical values (generic values less, therefore
  *                  less accurate than calibrated values),
  *                  use helper macro @ref __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS().
  * @note   As calculation input, the analog reference voltage (Vref+) must be
  *         defined as it impacts the ADC LSB equivalent voltage.
  * @note   Analog reference voltage (Vref+) must be either known from
  *         user board environment or can be calculated using ADC measurement
  *         and ADC helper macro @ref __LL_ADC_CALC_VREFANALOG_VOLTAGE().
  * @param  __VREFANALOG_VOLTAGE__  Analog reference voltage (unit: mV)
  * @param  __TEMPSENSOR_ADC_DATA__ ADC conversion data of internal
  *                                 temperature sensor (unit: digital value).
  * @param  __ADC_RESOLUTION__      ADC resolution at which internal temperature
  *                                 sensor voltage has been measured.
  *         This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @retval Temperature (unit: degree Celsius)   
  */
#define __LL_ADC_CALC_TEMPERATURE(__VREFANALOG_VOLTAGE__,\
                                  __TEMPSENSOR_ADC_DATA__,\
                                  __ADC_RESOLUTION__)                              \
  (((( ((int32_t)((__LL_ADC_CONVERT_DATA_RESOLUTION((__TEMPSENSOR_ADC_DATA__),     \
                                                    (__ADC_RESOLUTION__),          \
                                                    LL_ADC_RESOLUTION_12B)         \
                   * (__VREFANALOG_VOLTAGE__))                                     \
                  / TEMPSENSOR_CAL_VREFANALOG)                                     \
        - (int32_t) *TEMPSENSOR_CAL1_ADDR)                                         \
     ) * (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)                    \
    ) / (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) \
   ) + TEMPSENSOR_CAL1_TEMP                                                        \
  )

/**
  * @brief  Helper macro to calculate the temperature (unit: degree Celsius)
  *         from ADC conversion data of internal temperature sensor.
  * @note   Computation is using temperature sensor typical values
  *         (refer to device datasheet).
  * @note   Calculation formula:
  *           Temperature = (TS_ADC_DATA * Conversion_uV-TS_TYP_CALx_VOLT(uV) )
  *                         / Avg_Slope + CALx_TEMP
  *           with TS_ADC_DATA      = temperature sensor raw data measured by ADC
  *                                   (unit: digital value)
  *                Avg_Slope        = temperature sensor slope
  *                                   (unit: uV/Degree Celsius)
  *                TS_TYP_CALx_VOLT = temperature sensor digital value at
  *                                   temperature CALx_TEMP (unit: mV)
  *         Caution: Calculation relevancy under reserve the temperature sensor
  *                  of the current device has characteristics in line with
  *                  datasheet typical values.
  *                  If temperature sensor calibration values are available on
  *                  on this device (presence of macro __LL_ADC_CALC_TEMPERATURE()),
  *                  temperature calculation will be more accurate using
  *                  helper macro @ref __LL_ADC_CALC_TEMPERATURE().
  * @note   As calculation input, the analog reference voltage (Vref+) must be
  *         defined as it impacts the ADC LSB equivalent voltage.
  * @note   Analog reference voltage (Vref+) must be either known from
  *         user board environment or can be calculated using ADC measurement
  *         and ADC helper macro @ref __LL_ADC_CALC_VREFANALOG_VOLTAGE().
  * @note   ADC measurement data must correspond to a resolution of 12bits
  *         (full scale digital value 4095). If not the case, the data must be
  *         preliminarily rescaled to an equivalent resolution of 12 bits.
  * @param  __TEMPSENSOR_TYP_AVGSLOPE__   Device datasheet data: Temperature sensor slope typical value (unit: uV/DegCelsius).
  *                                       refer to device datasheet parameter "Avg_Slope".
  * @param  __TEMPSENSOR_TYP_CALX_V__     Device datasheet data: Temperature sensor voltage typical value (at temperature and Vref+ defined in parameters below) (unit: mV).
  *                                       refer to device datasheet parameter "V30" (corresponding to TS_CAL1).
  * @param  __TEMPSENSOR_CALX_TEMP__      Device datasheet data: Temperature at which temperature sensor voltage (see parameter above) is corresponding (unit: mV)
  * @param  __VREFANALOG_VOLTAGE__        Analog voltage reference (Vref+) voltage (unit: mV)
  * @param  __TEMPSENSOR_ADC_DATA__       ADC conversion data of internal temperature sensor (unit: digital value).
  * @param  __ADC_RESOLUTION__            ADC resolution at which internal temperature sensor voltage has been measured.
  *         This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @retval Temperature (unit: degree Celsius)
  */

#define __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(__TEMPSENSOR_TYP_AVGSLOPE__,\
                                             __TEMPSENSOR_TYP_CALX_V__,\
                                             __TEMPSENSOR_CALX_TEMP__,\
                                             __VREFANALOG_VOLTAGE__,\
                                             __TEMPSENSOR_ADC_DATA__,\
                                             __ADC_RESOLUTION__)               \
  (((( (int32_t)((((__TEMPSENSOR_ADC_DATA__) * (__VREFANALOG_VOLTAGE__))       \
                  / __LL_ADC_DIGITAL_SCALE(__ADC_RESOLUTION__))                \
                 * 1000)- (int32_t)(((__TEMPSENSOR_TYP_CALX_V__))              \
                 * 1000))) / (__TEMPSENSOR_TYP_AVGSLOPE__)                     \
   ) + (__TEMPSENSOR_CALX_TEMP__)                                              \
  )

/**
  * @}
  */

/** @defgroup ADC_LL_EF_Configuration_ADC_Common Configuration of ADC hierarchical scope: common to several ADC instances
  * @{
  */

/**
  * @brief  Set parameter common to several ADC: measurement path to internal
  *         channels (VrefInt, temperature sensor, ...).
  * @note   One or several values can be selected.
  *         Example: (LL_ADC_PATH_INTERNAL_VREFINT |
  *                   LL_ADC_PATH_INTERNAL_TEMPSENSOR)
  * @note   Stabilization time of measurement path to internal channel:
  *         After enabling internal paths, before starting ADC conversion,
  *         a delay is required for internal voltage reference and
  *         temperature sensor stabilization time.
  *         Refer to device datasheet.
  *         Refer to literal @ref LL_ADC_DELAY_VREFINT_STAB_US.
  *         Refer to literal @ref LL_ADC_DELAY_TEMPSENSOR_STAB_US.
  * @note   ADC internal channel sampling time constraint:
  *         For ADC conversion of internal channels,
  *         a sampling time minimum value is required.
  *         Refer to device datasheet.
  * @note   Setting of this feature is conditioned to ADC state:
  *         All ADC instances of the ADC common group must be disabled.
  *         This check can be done with function @ref LL_ADC_IsEnabled() for each
  *         ADC instance or by using helper macro helper macro
  *         @ref __LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE().
  * @rmtoll CCR      VREFEN         LL_ADC_SetCommonPathInternalCh\n
  *         CCR      TSEN           LL_ADC_SetCommonPathInternalCh
  * @param  ADCxy_COMMON ADC common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __LL_ADC_COMMON_INSTANCE() )
  * @param  PathInternal This parameter can be a combination of the following values:
  *         @arg @ref LL_ADC_PATH_INTERNAL_NONE
  *         @arg @ref LL_ADC_PATH_INTERNAL_VREFINT
  *         @arg @ref LL_ADC_PATH_INTERNAL_TEMPSENSOR   
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  MODIFY_REG(ADCxy_COMMON->CCR, ADC_CCR_VREFEN | ADC_CCR_TSEN, PathInternal);
}

/**
  * @brief  Get parameter common to several ADC: measurement path to internal
  *         channels (VrefInt, temperature sensor, ...).
  * @note   One or several values can be selected.
  *         Example: (LL_ADC_PATH_INTERNAL_VREFINT |
  *                   LL_ADC_PATH_INTERNAL_TEMPSENSOR)
  * @rmtoll CCR      VREFEN         LL_ADC_GetCommonPathInternalCh\n
  *         CCR      TSEN           LL_ADC_GetCommonPathInternalCh
  * @param  ADCxy_COMMON ADC common instance
  *         (can be set directly from CMSIS definition or by using helper macro @ref __LL_ADC_COMMON_INSTANCE() )
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_ADC_PATH_INTERNAL_NONE
  *         @arg @ref LL_ADC_PATH_INTERNAL_VREFINT
  *         @arg @ref LL_ADC_PATH_INTERNAL_TEMPSENSOR
  */
__STATIC_INLINE uint32_t LL_ADC_GetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(READ_BIT(ADCxy_COMMON->CCR, ADC_CCR_VREFEN | ADC_CCR_TSEN));
}

/**
  * @}
  */

/** @defgroup ADC_LL_EF_Configuration_ADC_Instance Configuration of ADC hierarchical scope: ADC instance
  * @{
  */

/**
  * @brief  Set ADC instance clock source and prescaler.
  * @note   Setting of this feature is conditioned to ADC state:
  *         ADC must be disabled.
  * @rmtoll RCC_CCIPR    ADC_CKMODE      LL_ADC_SetClock
  * @param  ADCx ADC instance
  * @param  ClockSource This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV1
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV2
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV4
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV8
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV16
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV32
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV64
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV1
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV2
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV4
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV8
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV16
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV32
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV64
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetClock(ADC_TypeDef *ADCx, uint32_t ClockSource)
{
  (void)ADCx;
  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADC_CKMODE, ClockSource);
}
 
/**
  * @brief  Get ADC instance clock source and prescaler.
  * @rmtoll RCC_CCIPR    CKMODE         LL_ADC_GetClock
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV1
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV2
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV4
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV8
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV16
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV32
  *         @arg @ref LL_ADC_CLOCK_SYNC_PCLK_DIV64
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV1
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV2
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV4
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV8
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV16
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV32
  *         @arg @ref LL_ADC_CLOCK_ASYNC_HSI_DIV64
  */
__STATIC_INLINE uint32_t LL_ADC_GetClock(ADC_TypeDef *ADCx)
{
  (void)ADCx;
  return (uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_ADC_CKMODE));
}
 
/**
  * @brief  Set ADC resolution.
  *         Refer to reference manual for alignments formats
  *         dependencies to ADC resolutions.
  * @note   Setting of this feature is conditioned to ADC state:
  *         ADC must be disabled or enabled without conversion on going
  *         on group regular.
  * @rmtoll CFGR1    RES            LL_ADC_SetResolution
  * @param  ADCx ADC instance
  * @param  Resolution This parameter can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetResolution(ADC_TypeDef *ADCx, uint32_t Resolution)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_RESSEL, Resolution);
}

/**
  * @brief  Get ADC resolution.
  *         Refer to reference manual for alignments formats
  *         dependencies to ADC resolutions.
  * @rmtoll CFGR1    RES            LL_ADC_GetResolution
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_RESOLUTION_12B
  *         @arg @ref LL_ADC_RESOLUTION_10B
  *         @arg @ref LL_ADC_RESOLUTION_8B
  *         @arg @ref LL_ADC_RESOLUTION_6B   
  */
__STATIC_INLINE uint32_t LL_ADC_GetResolution(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, ADC_CFGR1_RESSEL));
}

/**
  * @brief  Set ADC conversion data alignment.
  * @note   Refer to reference manual for alignments formats
  *         dependencies to ADC resolutions.
  * @note   Setting of this feature is conditioned to ADC state:
  *         ADC must be disabled or enabled without conversion on going
  *         on group regular.
  * @rmtoll CFGR1    ALIGN          LL_ADC_SetDataAlignment
  * @param  ADCx ADC instance
  * @param  DataAlignment This parameter can be one of the following values:
  *         @arg @ref LL_ADC_DATA_ALIGN_RIGHT
  *         @arg @ref LL_ADC_DATA_ALIGN_LEFT
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetDataAlignment(ADC_TypeDef *ADCx, uint32_t DataAlignment)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_ALIGN, DataAlignment);
}

/**
  * @brief  Get ADC conversion data alignment.
  * @note   Refer to reference manual for alignments formats
  *         dependencies to ADC resolutions.
  * @rmtoll CFGR1    ALIGN          LL_ADC_GetDataAlignment
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_DATA_ALIGN_RIGHT
  *         @arg @ref LL_ADC_DATA_ALIGN_LEFT
  */
__STATIC_INLINE uint32_t LL_ADC_GetDataAlignment(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, ADC_CFGR1_ALIGN));
}

/**
  * @brief  Set ADC low power mode.
  * @note   With ADC low power mode "auto wait", the ADC conversion data read
  *         is corresponding to previous ADC conversion start, independently
  *         of delay during which ADC was idle.
  *         Therefore, the ADC conversion data may be outdated: does not
  *         correspond to the current voltage level on the selected
  *         ADC channel.
  * @note   Setting of this feature is conditioned to ADC state:
  *         ADC must be disabled or enabled without conversion on going
  *         on group regular.
  * @rmtoll CFGR1    WAIT           LL_ADC_SetLowPowerMode
  * @param  ADCx ADC instance
  * @param  LowPowerMode This parameter can be one of the following values:
  *         @arg @ref LL_ADC_LP_MODE_NONE
  *         @arg @ref LL_ADC_LP_AUTOWAIT           
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetLowPowerMode(ADC_TypeDef *ADCx, uint32_t LowPowerMode)
{
  MODIFY_REG(ADCx->CFGR1, (ADC_CFGR1_WAIT), LowPowerMode);
}

/**
  * @brief  Get ADC low power mode:
  * @note   With ADC low power mode "auto wait", the ADC conversion data read
  *         is corresponding to previous ADC conversion start, independently
  *         of delay during which ADC was idle.
  *         Therefore, the ADC conversion data may be outdated: does not
  *         correspond to the current voltage level on the selected
  *         ADC channel.
  * @rmtoll CFGR1    WAIT           LL_ADC_GetLowPowerMode
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_LP_MODE_NONE
  *         @arg @ref LL_ADC_LP_AUTOWAIT         
  */
__STATIC_INLINE uint32_t LL_ADC_GetLowPowerMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, (ADC_CFGR1_WAIT )));
}


/**
  * @brief  Set sampling time common to a group of channels.
  * @note   Unit: ADC clock cycles.
  * @note   In case of ADC conversion of internal channel (VrefInt,
  *         temperature sensor, ...), a sampling time minimum value
  *         is required.
  *         Refer to device datasheet.
  * @rmtoll SMPR     SMP            LL_ADC_SetSamplingTimeCommonChannels
  * @param  ADCx ADC instance
  * @param  SamplingTime This parameter can be one of the following values:
  *         @arg @ref LL_ADC_SAMPLINGTIME_3CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_5CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_7CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_13CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_28CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_41CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_134CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_239CYCLES_5
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetSamplingTimeCommonChannels(ADC_TypeDef *ADCx, uint32_t SamplingTime)
{
  MODIFY_REG(ADCx->SMPR, ADC_SMPR_SMP, SamplingTime);
}

/**
  * @brief  Get sampling time common to a group of channels.
  * @note   Unit: ADC clock cycles.
  * @rmtoll SMPR     SMP            LL_ADC_GetSamplingTimeCommonChannels
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_SAMPLINGTIME_3CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_5CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_7CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_13CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_28CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_41CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_134CYCLES_5
  *         @arg @ref LL_ADC_SAMPLINGTIME_239CYCLES_5
  */
__STATIC_INLINE uint32_t LL_ADC_GetSamplingTimeCommonChannels(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->SMPR, ADC_SMPR_SMP));
}

/**
  * @}
  */

/** @defgroup ADC_LL_EF_Configuration_ADC_Group_Regular Configuration of ADC hierarchical scope: group regular
  * @{
  */

/**
  * @brief  Set ADC group regular conversion trigger source:
  *         internal (SW start) or from external IP (timer event,
  *         external interrupt line).
  * @rmtoll CFGR1    EXTSEL         LL_ADC_REG_SetTriggerSource\n
  *         CFGR1    EXTEN          LL_ADC_REG_SetTriggerSource
  * @param  ADCx ADC instance
  * @param  TriggerSource This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REG_TRIG_SOFTWARE
  *         @arg @ref LL_ADC_REG_TRIG_EXT_TIM1_TRGO
  *         @arg @ref LL_ADC_REG_TRIG_EXT_TIM1_CH4
  *         @arg @ref LL_ADC_REG_TRIG_EXT_TIM14_CH1
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_EXTEN | ADC_CFGR1_EXTSEL, TriggerSource);
}

/**
  * @brief  Get ADC group regular conversion trigger source:
  *         internal (SW start) or from external IP (timer event,
  *         external interrupt line).
  * @note   To determine whether group regular trigger source is
  *         internal (SW start) or external, without detail
  *         of which peripheral is selected as external trigger,
  *         (equivalent to
  *         "if(LL_ADC_REG_GetTriggerSource(ADC1) == LL_ADC_REG_TRIG_SOFTWARE)")
  *         use function @ref LL_ADC_REG_IsTriggerSourceSWStart.
  * @note   Availability of parameters of trigger sources from timer
  *         depends on timers availability on the selected device.
  * @rmtoll CFGR1    EXTSEL         LL_ADC_REG_GetTriggerSource\n
  *         CFGR1    EXTEN          LL_ADC_REG_GetTriggerSource
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REG_TRIG_SOFTWARE
  *         @arg @ref LL_ADC_REG_TRIG_EXT_TIM1_TRGO
  *         @arg @ref LL_ADC_REG_TRIG_EXT_TIM1_CH4
  *         @arg @ref LL_ADC_REG_TRIG_EXT_TIM14_CH1
  */
__STATIC_INLINE uint32_t LL_ADC_REG_GetTriggerSource(ADC_TypeDef *ADCx)
{
  register uint32_t TriggerSource = READ_BIT(ADCx->CFGR1, ADC_CFGR1_EXTSEL | ADC_CFGR1_EXTEN);

  /* Value for shift of {0; 4; 8; 12} depending on value of bitfield          */
  /* corresponding to ADC_CFGR1_EXTEN {0; 1; 2; 3}.                           */
  register uint32_t ShiftExten = ((TriggerSource & ADC_CFGR1_EXTEN) >> (ADC_REG_TRIG_EXTEN_BITOFFSET_POS - 2U));

  /* Set bitfield corresponding to ADC_CFGR1_EXTEN and ADC_CFGR1_EXTSEL       */
  /* to match with triggers literals definition.                              */
  return ((TriggerSource
           & (ADC_REG_TRIG_SOURCE_MASK >> ShiftExten) & ADC_CFGR1_EXTSEL)
          | ((ADC_REG_TRIG_EDGE_MASK >> ShiftExten) & ADC_CFGR1_EXTEN)
         );
}

/**
  * @brief  Get ADC group regular conversion trigger source internal (SW start)
  *         or external.
  * @note   In case of group regular trigger source set to external trigger,
  *         to determine which peripheral is selected as external trigger,
  *         use function @ref LL_ADC_REG_GetTriggerSource().
  * @rmtoll CFGR1    EXTEN          LL_ADC_REG_IsTriggerSourceSWStart
  * @param  ADCx ADC instance
  * @retval Value "0" if trigger source external trigger
  *         Value "1" if trigger source SW start.
  */
__STATIC_INLINE uint32_t LL_ADC_REG_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->CFGR1, ADC_CFGR1_EXTEN) == (LL_ADC_REG_TRIG_SOFTWARE & ADC_CFGR1_EXTEN));
}

/**
  * @brief  Set ADC group regular conversion trigger polarity.
  * @note   Applicable only for trigger source set to external trigger.
  * @rmtoll CFGR1    EXTEN          LL_ADC_REG_SetTriggerEdge
  * @param  ADCx ADC instance
  * @param  ExternalTriggerEdge This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REG_TRIG_EXT_RISING
  *         @arg @ref LL_ADC_REG_TRIG_EXT_FALLING
  *         @arg @ref LL_ADC_REG_TRIG_EXT_RISINGFALLING
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_EXTEN, ExternalTriggerEdge);
}

/**
  * @brief  Get ADC group regular conversion trigger polarity.
  * @note   Applicable only for trigger source set to external trigger.
  * @rmtoll CFGR1    EXTEN          LL_ADC_REG_GetTriggerEdge
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REG_TRIG_EXT_RISING
  *         @arg @ref LL_ADC_REG_TRIG_EXT_FALLING
  *         @arg @ref LL_ADC_REG_TRIG_EXT_RISINGFALLING
  */
__STATIC_INLINE uint32_t LL_ADC_REG_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, ADC_CFGR1_EXTEN));
}


/**
  * @brief  Set ADC group regular sequencer scan direction.
  * @rmtoll CFGR1    SCANDIR        LL_ADC_REG_SetSequencerScanDirection
  * @param  ADCx ADC instance
  * @param  ScanDirection This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REG_SEQ_SCAN_DIR_FORWARD
  *         @arg @ref LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetSequencerScanDirection(ADC_TypeDef *ADCx, uint32_t ScanDirection)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_SCANDIR, ScanDirection);
}

/**
  * @brief  Get ADC group regular sequencer scan direction.
  * @rmtoll CFGR1    SCANDIR        LL_ADC_REG_GetSequencerScanDirection
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REG_SEQ_SCAN_DIR_FORWARD
  *         @arg @ref LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD
  */
__STATIC_INLINE uint32_t LL_ADC_REG_GetSequencerScanDirection(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, ADC_CFGR1_SCANDIR));
}

/**
  * @brief  Set ADC group regular sequencer discontinuous mode:
  *         sequence subdivided and scan conversions interrupted every selected
  *         number of ranks.
  * @rmtoll CFGR1    DISCEN         LL_ADC_REG_SetSequencerDiscont\n
  * @param  ADCx ADC instance
  * @param  SeqDiscont This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REG_SEQ_DISCONT_DISABLE
  *         @arg @ref LL_ADC_REG_SEQ_DISCONT_1RANK
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_DISCEN, SeqDiscont);
}

/**
  * @brief  Get ADC group regular sequencer discontinuous mode:
  *         sequence subdivided and scan conversions interrupted every selected
  *         number of ranks.
  * @rmtoll CFGR1    DISCEN         LL_ADC_REG_GetSequencerDiscont\n
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REG_SEQ_DISCONT_DISABLE
  *         @arg @ref LL_ADC_REG_SEQ_DISCONT_1RANK
  */
__STATIC_INLINE uint32_t LL_ADC_REG_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, ADC_CFGR1_DISCEN));
}

/**
  * @brief  Set ADC group regular sequence: channel on rank corresponding to
  *         channel number.
  * @note   One or several values can be selected.
  *         Example: (LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_8 | ...)
  * @rmtoll CHSELR   CHSEL0         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL1         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL2         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL3         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL4         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL5         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL6         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL7         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL8         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL9         LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL10        LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL11        LL_ADC_REG_SetSequencerChannels\n
  *         CHSELR   CHSEL12        LL_ADC_REG_SetSequencerChannels
  * @param  ADCx ADC instance
  * @param  Channel This parameter can be a combination of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetSequencerChannels(ADC_TypeDef *ADCx, uint32_t Channel)
{
  /* Parameter "Channel" is used with masks because containing                */
  /* other bits reserved for other purpose.                                   */
  WRITE_REG(ADCx->CHSELR, (Channel & ADC_CHANNEL_ID_BITFIELD_MASK));
}

/**
  * @brief  Add channel to ADC group regular sequence: channel on rank corresponding to
  *         channel number.
  * @note   This function performs:
  *         - Channels ordering into each rank of scan sequence:
  *           rank of each channel is fixed by channel HW number
  *           (channel 0 fixed on rank 0, channel 1 fixed on rank1, ...).
  *         - Set channels selected by adding them to the current sequencer
  *           configuration.
  * @note   One or several values can be selected.
  *         Example: (LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_8 | ...)
  * @rmtoll CHSELR   CHSEL0         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL1         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL2         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL3         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL4         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL5         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL6         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL7         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL8         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL9         LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL10        LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL11        LL_ADC_REG_SetSequencerChAdd\n
  *         CHSELR   CHSEL12        LL_ADC_REG_SetSequencerChAdd
  * @param  ADCx ADC instance
  * @param  Channel This parameter can be a combination of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetSequencerChAdd(ADC_TypeDef *ADCx, uint32_t Channel)
{
  /* Parameter "Channel" is used with masks because containing                */
  /* other bits reserved for other purpose.                                   */
  SET_BIT(ADCx->CHSELR, (Channel & ADC_CHANNEL_ID_BITFIELD_MASK));
}

/**
  * @brief  Remove channel to ADC group regular sequence: channel on rank corresponding to
  *         channel number.
  * @note   One or several values can be selected.
  *         Example: (LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_8 | ...)
  * @rmtoll CHSELR   CHSEL0         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL1         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL2         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL3         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL4         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL5         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL6         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL7         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL8         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL9         LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL10        LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL11        LL_ADC_REG_SetSequencerChRem\n
  *         CHSELR   CHSEL12        LL_ADC_REG_SetSequencerChRem
  * @param  ADCx ADC instance
  * @param  Channel This parameter can be a combination of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetSequencerChRem(ADC_TypeDef *ADCx, uint32_t Channel)
{
  /* Parameter "Channel" is used with masks because containing                */
  /* other bits reserved for other purpose.                                   */
  CLEAR_BIT(ADCx->CHSELR, (Channel & ADC_CHANNEL_ID_BITFIELD_MASK));
}

/**
  * @brief  Get ADC group regular sequence: channel on rank corresponding to
  *         channel number.
  * @note   One or several values can be retrieved.
  *         Example: (LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_8 | ...)
  * @rmtoll CHSELR   CHSEL0         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL1         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL2         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL3         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL4         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL5         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL6         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL7         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL8         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL9         LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL10        LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL11        LL_ADC_REG_GetSequencerChannels\n
  *         CHSELR   CHSEL12        LL_ADC_REG_GetSequencerChannels
  * @param  ADCx ADC instance
  * @retval Returned value can be a combination of the following values:
  *         @arg @ref LL_ADC_CHANNEL_0
  *         @arg @ref LL_ADC_CHANNEL_1
  *         @arg @ref LL_ADC_CHANNEL_2
  *         @arg @ref LL_ADC_CHANNEL_3
  *         @arg @ref LL_ADC_CHANNEL_4
  *         @arg @ref LL_ADC_CHANNEL_5
  *         @arg @ref LL_ADC_CHANNEL_6
  *         @arg @ref LL_ADC_CHANNEL_7
  *         @arg @ref LL_ADC_CHANNEL_8
  *         @arg @ref LL_ADC_CHANNEL_9
  *         @arg @ref LL_ADC_CHANNEL_10
  *         @arg @ref LL_ADC_CHANNEL_11
  *         @arg @ref LL_ADC_CHANNEL_12      
  */
__STATIC_INLINE uint32_t LL_ADC_REG_GetSequencerChannels(ADC_TypeDef *ADCx)
{
  register uint32_t ChannelsBitfield = READ_BIT(ADCx->CHSELR, ADC_CHSELR_CHSEL);

  return (   (((ChannelsBitfield & ADC_CHSELR_CHSEL0) >> ADC_CHSELR_CHSEL0_BITOFFSET_POS) * LL_ADC_CHANNEL_0)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL1) >> ADC_CHSELR_CHSEL1_BITOFFSET_POS) * LL_ADC_CHANNEL_1)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL2) >> ADC_CHSELR_CHSEL2_BITOFFSET_POS) * LL_ADC_CHANNEL_2)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL3) >> ADC_CHSELR_CHSEL3_BITOFFSET_POS) * LL_ADC_CHANNEL_3)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL4) >> ADC_CHSELR_CHSEL4_BITOFFSET_POS) * LL_ADC_CHANNEL_4)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL5) >> ADC_CHSELR_CHSEL5_BITOFFSET_POS) * LL_ADC_CHANNEL_5)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL6) >> ADC_CHSELR_CHSEL6_BITOFFSET_POS) * LL_ADC_CHANNEL_6)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL7) >> ADC_CHSELR_CHSEL7_BITOFFSET_POS) * LL_ADC_CHANNEL_7)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL8) >> ADC_CHSELR_CHSEL8_BITOFFSET_POS) * LL_ADC_CHANNEL_8)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL9) >> ADC_CHSELR_CHSEL9_BITOFFSET_POS) * LL_ADC_CHANNEL_9)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL10) >> ADC_CHSELR_CHSEL10_BITOFFSET_POS) * LL_ADC_CHANNEL_10)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL11) >> ADC_CHSELR_CHSEL11_BITOFFSET_POS) * LL_ADC_CHANNEL_11)
             | (((ChannelsBitfield & ADC_CHSELR_CHSEL12) >> ADC_CHSELR_CHSEL12_BITOFFSET_POS) * LL_ADC_CHANNEL_12)
         );
}
 



/**
  * @brief  Set ADC continuous conversion mode on ADC group regular.
  * @note   Description of ADC continuous conversion mode:
  *         - single mode: one conversion per trigger
  *         - continuous mode: after the first trigger, following
  *           conversions launched successively automatically.
  * @rmtoll CFGR1    CONT           LL_ADC_REG_SetContinuousMode
  * @param  ADCx ADC instance
  * @param  Continuous This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REG_CONV_SINGLE
  *         @arg @ref LL_ADC_REG_CONV_CONTINUOUS
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetContinuousMode(ADC_TypeDef *ADCx, uint32_t Continuous)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_CONT, Continuous);
}

/**
  * @brief  Get ADC continuous conversion mode on ADC group regular.
  * @note   Description of ADC continuous conversion mode:
  *         - single mode: one conversion per trigger
  *         - continuous mode: after the first trigger, following
  *           conversions launched successively automatically.
  * @rmtoll CFGR1    CONT           LL_ADC_REG_GetContinuousMode
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REG_CONV_SINGLE
  *         @arg @ref LL_ADC_REG_CONV_CONTINUOUS
  */
__STATIC_INLINE uint32_t LL_ADC_REG_GetContinuousMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, ADC_CFGR1_CONT));
}

/**
  * @brief  Set ADC group regular behavior in case of overrun:
  *         data preserved or overwritten.
  * @rmtoll CFGR1    OVRMOD         LL_ADC_REG_SetOverrun
  * @param  ADCx ADC instance
  * @param  Overrun This parameter can be one of the following values:
  *         @arg @ref LL_ADC_REG_OVR_DATA_PRESERVED
  *         @arg @ref LL_ADC_REG_OVR_DATA_OVERWRITTEN
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_SetOverrun(ADC_TypeDef *ADCx, uint32_t Overrun)
{
  MODIFY_REG(ADCx->CFGR1, ADC_CFGR1_OVRMOD, Overrun);
}

/**
  * @brief  Get ADC group regular behavior in case of overrun:
  *         data preserved or overwritten.
  * @rmtoll CFGR1    OVRMOD         LL_ADC_REG_GetOverrun
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_REG_OVR_DATA_PRESERVED
  *         @arg @ref LL_ADC_REG_OVR_DATA_OVERWRITTEN
  */
__STATIC_INLINE uint32_t LL_ADC_REG_GetOverrun(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CFGR1, ADC_CFGR1_OVRMOD));
}

/**
  * @}
  */

/** @defgroup ADC_LL_EF_Configuration_ADC_AnalogWatchdog Configuration of ADC transversal scope: analog watchdog
  * @{
  */

/**
  * @brief  Set ADC analog watchdog monitored channels:
  *         a single channel or all channels,
  *         on ADC group regular.
  * @note   Once monitored channels are selected, analog watchdog
  *         is enabled.
  * @note   Depending on devices and packages, some channels may not be available.
  *         Refer to device datasheet for channels availability.
  * @rmtoll CFGR1    AWDCH          LL_ADC_SetAnalogWDMonitChannels\n
  *         CFGR1    AWDSGL         LL_ADC_SetAnalogWDMonitChannels\n
  *         CFGR1    AWDEN          LL_ADC_SetAnalogWDMonitChannels
  * @param  ADCx ADC instance
  * @param  AWDChannelGroup This parameter can be one of the following values:
  *         @arg @ref LL_ADC_AWD_DISABLE
  *         @arg @ref LL_ADC_AWD_ALL_CHANNELS_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_0_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_1_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_2_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_3_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_4_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_5_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_6_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_7_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_8_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_9_REG
  *         @arg @ref LL_ADC_AWD_CH_VREFINT_REG
  *         @arg @ref LL_ADC_AWD_CH_TEMPSENSOR_REG
  *         @arg @ref LL_ADC_AWD_CH_1_3VCCA_REG
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDChannelGroup)
{
  MODIFY_REG(ADCx->CFGR1,
             (ADC_CFGR1_AWDCH | ADC_CFGR1_AWDSGL | ADC_CFGR1_AWDEN),
             (AWDChannelGroup & ADC_AWD_CR_ALL_CHANNEL_MASK));
}

/**
  * @brief  Get ADC analog watchdog monitored channel.
  * @note   Usage of the returned channel number:
  *         - To reinject this channel into another function LL_ADC_xxx:
  *           the returned channel number is only partly formatted on definition
  *           of literals LL_ADC_CHANNEL_x. Therefore, it has to be compared
  *           with parts of literals LL_ADC_CHANNEL_x or using
  *           helper macro @ref __LL_ADC_CHANNEL_TO_DECIMAL_NB().
  *           Then the selected literal LL_ADC_CHANNEL_x can be used
  *           as parameter for another function.
  *         - To get the channel number in decimal format:
  *           process the returned value with the helper macro
  *           @ref __LL_ADC_CHANNEL_TO_DECIMAL_NB().
  *           Applicable only when the analog watchdog is set to monitor
  *           one channel.
  * @note   Depending on devices and packages, some channels may not be available.
  *         Refer to device datasheet for channels availability.
  * @rmtoll CFGR1    AWDCH          LL_ADC_GetAnalogWDMonitChannels\n
  *         CFGR1    AWDSGL         LL_ADC_GetAnalogWDMonitChannels\n
  *         CFGR1    AWDEN          LL_ADC_GetAnalogWDMonitChannels
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_AWD_DISABLE
  *         @arg @ref LL_ADC_AWD_ALL_CHANNELS_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_0_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_1_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_2_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_3_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_4_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_5_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_6_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_7_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_8_REG
  *         @arg @ref LL_ADC_AWD_CHANNEL_9_REG
  *         @arg @ref LL_ADC_AWD_CH_VREFINT_REG
  *         @arg @ref LL_ADC_AWD_CH_TEMPSENSOR_REG
  *         @arg @ref LL_ADC_AWD_CH_1_3VCCA_REG
  */
__STATIC_INLINE uint32_t LL_ADC_GetAnalogWDMonitChannels(ADC_TypeDef *ADCx)
{
  register uint32_t AWDChannelGroup = READ_BIT(ADCx->CFGR1, (ADC_CFGR1_AWDCH | ADC_CFGR1_AWDSGL | ADC_CFGR1_AWDEN));

  /* Note: Set variable according to channel definition including channel ID  */
  /*       with bitfield.                                                     */
  register uint32_t AWDChannelSingle = ((AWDChannelGroup & ADC_CFGR1_AWDSGL) >> ADC_CFGR1_AWDSGL_BITOFFSET_POS);
  register uint32_t AWDChannelBitField = (ADC_CHANNEL_0_BITFIELD << ((AWDChannelGroup & ADC_CHANNEL_ID_NUMBER_MASK) >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS));

  return (AWDChannelGroup | (AWDChannelBitField * AWDChannelSingle));
}

/**
  * @brief  Set ADC analog watchdog thresholds value of both thresholds
  *         high and low.
  * @note   If value of only one threshold high or low must be set,
  *         use function @ref LL_ADC_SetAnalogWDThresholds().
  * @note   In case of ADC resolution different of 12 bits,
  *         analog watchdog thresholds data require a specific shift.
  *         Use helper macro @ref __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION().
  * @rmtoll TR       HT             LL_ADC_ConfigAnalogWDThresholds\n
  *         TR       LT             LL_ADC_ConfigAnalogWDThresholds
  * @param  ADCx ADC instance
  * @param  AWDThresholdHighValue Value between Min_Data=0x000 and Max_Data=0xFFF
  * @param  AWDThresholdLowValue Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval None
  */
__STATIC_INLINE void LL_ADC_ConfigAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDThresholdHighValue, uint32_t AWDThresholdLowValue)
{
  MODIFY_REG(ADCx->TR,ADC_TR_HT | ADC_TR_LT,(AWDThresholdHighValue << ADC_TR_HT_BITOFFSET_POS) | AWDThresholdLowValue);
}

/**
  * @brief  Set ADC analog watchdog threshold value of threshold
  *         high or low.
  * @note   If values of both thresholds high or low must be set,
  *         use function @ref LL_ADC_ConfigAnalogWDThresholds().
  * @note   In case of ADC resolution different of 12 bits,
  *         analog watchdog thresholds data require a specific shift.
  *         Use helper macro @ref __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION().
  * @rmtoll TR       HT             LL_ADC_SetAnalogWDThresholds\n
  *         TR       LT             LL_ADC_SetAnalogWDThresholds
  * @param  ADCx ADC instance
  * @param  AWDThresholdsHighLow This parameter can be one of the following values:
  *         @arg @ref LL_ADC_AWD_THRESHOLD_HIGH
  *         @arg @ref LL_ADC_AWD_THRESHOLD_LOW
  * @param  AWDThresholdValue Value between Min_Data=0x000 and Max_Data=0xFFF
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDThresholdsHighLow, uint32_t AWDThresholdValue)
{
  /* Parameter "AWDThresholdsHighLow" is used with mask "0x00000010"          */
  /* to be equivalent to "POSITION_VAL(AWDThresholdsHighLow)": if threshold   */
  /* high is selected, then data is shifted to LSB. Else(threshold low),      */
  /* data is not shifted.                                                     */
  MODIFY_REG(ADCx->TR, AWDThresholdsHighLow,AWDThresholdValue << ((AWDThresholdsHighLow >> ADC_TR_HT_BITOFFSET_POS) & 0x00000010U));
}

/**
  * @brief  Get ADC analog watchdog threshold value of threshold high,
  *         threshold low or raw data with ADC thresholds high and low
  *         concatenated.
  * @note   If raw data with ADC thresholds high and low is retrieved,
  *         the data of each threshold high or low can be isolated
  *         using helper macro:
  *         @ref __LL_ADC_ANALOGWD_THRESHOLDS_HIGH_LOW().
  * @note   In case of ADC resolution different of 12 bits,
  *         analog watchdog thresholds data require a specific shift.
  *         Use helper macro @ref __LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION().
  * @rmtoll TR      HT            LL_ADC_GetAnalogWDThresholds\n
  *         TR      LT            LL_ADC_GetAnalogWDThresholds
  * @param  ADCx ADC instance
  * @param  AWDThresholdsHighLow This parameter can be one of the following values:
  *         @arg @ref LL_ADC_AWD_THRESHOLD_HIGH
  *         @arg @ref LL_ADC_AWD_THRESHOLD_LOW
  *         @arg @ref LL_ADC_AWD_THRESHOLDS_HIGH_LOW
  * @retval Value between Min_Data=0x000 and Max_Data=0xFFF
*/
__STATIC_INLINE uint32_t LL_ADC_GetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDThresholdsHighLow)
{
  /* Parameter "AWDThresholdsHighLow" is used with mask "0x00000010"          */
  /* to be equivalent to "POSITION_VAL(AWDThresholdsHighLow)": if threshold   */
  /* high is selected, then data is shifted to LSB. Else(threshold low or     */
  /* both thresholds), data is not shifted.                                   */
  return (uint32_t)(READ_BIT(ADCx->TR,(AWDThresholdsHighLow | ADC_TR_LT))>> ((~AWDThresholdsHighLow) & 0x00000010U));
}


/** @defgroup ADC_LL_EF_Operation_ADC_Instance Operation on ADC hierarchical scope: ADC instance
  * @{
  */

/**
  * @brief  Enable the selected ADC instance.
  * @rmtoll CR       ADEN           LL_ADC_Enable
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_Enable(ADC_TypeDef *ADCx)
{
  /* Note: Write register with some additional bits forced to state reset     */
  /*       instead of modifying only the selected bit for this function,      */
  /*       to not interfere with bits with HW property "rs".                  */
  MODIFY_REG(ADCx->CR,ADC_CR_BITS_PROPERTY_RS,ADC_CR_ADEN);
}

/**
  * @brief  Disable the selected ADC instance.
  * @rmtoll CR       ADDIS          LL_ADC_Disable
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_Disable(ADC_TypeDef *ADCx)
{
  /* Note: Write register with some additional bits forced to state reset     */
  /*       instead of modifying only the selected bit for this function,      */
  /*       to not interfere with bits with HW property "rs".                  */
  MODIFY_REG(ADCx->CR, ADC_CR_BITS_PROPERTY_RS, ADC_CR_ADDIS);
}


/**
  * @brief  Reset the selected ADC instance.
  * @param  ADCx ADC instance                 
  * @retval None
  */
__STATIC_INLINE void LL_ADC_Reset(ADC_TypeDef *ADCx)
{
  /* Note: Write register with some additional bits forced to state reset     */
  /*       instead of modifying only the selected bit for this function       */

  /* Force reset of ADC clock (core clock) */
  LL_APB1_GRP2_ForceReset(LL_APB1_GRP2_PERIPH_ADC1);

  /* Release reset of ADC clock (core clock) */
  LL_APB1_GRP2_ReleaseReset(LL_APB1_GRP2_PERIPH_ADC1);
}

/**
  * @brief  Get the selected ADC instance enable state.
  *         (not only core clock: this ADC has a dual clock domain)
  * @rmtoll CR       ADEN           LL_ADC_IsEnabled
  * @param  ADCx ADC instance
  * @retval 0: ADC is disabled, 1: ADC is enabled.
  */
__STATIC_INLINE uint32_t LL_ADC_IsEnabled(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->CR, ADC_CR_ADEN) == (ADC_CR_ADEN));
}

/**
  * @brief  Get the selected ADC instance disable state.
  * @rmtoll CR       ADDIS          LL_ADC_IsDisableOngoing
  * @param  ADCx ADC instance
  * @retval 0: no ADC disable command on going.
  */
__STATIC_INLINE uint32_t LL_ADC_IsDisableOngoing(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->CR, ADC_CR_ADDIS) == (ADC_CR_ADDIS));
}

/**
  * @brief  Start ADC calibration in the mode single-ended
  *         or differential (for devices with differential mode available).
  *         ADC must be ADC disabled.
  * @rmtoll CR       ADCAL          LL_ADC_StartCalibration
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_StartCalibration(ADC_TypeDef *ADCx)
{
  /* Note: Write register with some additional bits forced to state reset     */
  /*       instead of modifying only the selected bit for this function,      */
  /*       to not interfere with bits with HW property "rs".                  */
  MODIFY_REG(ADCx->CR,ADC_CR_BITS_PROPERTY_RS,ADC_CR_ADCAL);
}

/**
  * @brief  Get ADC calibration state.
  * @rmtoll CR       ADCAL          LL_ADC_IsCalibrationOnGoing
  * @param  ADCx ADC instance
  * @retval 0: calibration complete, 1: calibration in progress.
  */
__STATIC_INLINE uint32_t LL_ADC_IsCalibrationOnGoing(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->CR, ADC_CR_ADCAL) == (ADC_CR_ADCAL));
}
/**
  * @}
  */
/**
  * @}
  */

/** @defgroup ADC_LL_EF_Operation_ADC_Group_Regular Operation on ADC hierarchical scope: group regular
  * @{
  */

/**
  * @brief  Start ADC group regular conversion.
  * @rmtoll CR       ADSTART        LL_ADC_REG_StartConversion
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_StartConversion(ADC_TypeDef *ADCx)
{
  /* Note: Write register with some additional bits forced to state reset     */
  /*       instead of modifying only the selected bit for this function,      */
  /*       to not interfere with bits with HW property "rs".                  */
  MODIFY_REG(ADCx->CR, ADC_CR_BITS_PROPERTY_RS,ADC_CR_ADSTART);
}

/**
  * @brief  Stop ADC group regular conversion.
  * @rmtoll CR       ADSTP          LL_ADC_REG_StopConversion
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_REG_StopConversion(ADC_TypeDef *ADCx)
{
  /* Note: Write register with some additional bits forced to state reset     */
  /*       instead of modifying only the selected bit for this function,      */
  /*       to not interfere with bits with HW property "rs".                  */
  MODIFY_REG(ADCx->CR,ADC_CR_BITS_PROPERTY_RS,ADC_CR_ADSTP);
}

/**
  * @brief  Get ADC group regular conversion state.
  * @rmtoll CR       ADSTART        LL_ADC_REG_IsConversionOngoing
  * @param  ADCx ADC instance
  * @retval 0: no conversion is on going on ADC group regular.
  */
__STATIC_INLINE uint32_t LL_ADC_REG_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->CR, ADC_CR_ADSTART) == (ADC_CR_ADSTART));
}

/**
  * @brief  Get ADC group regular command of conversion stop state
  * @rmtoll CR       ADSTP          LL_ADC_REG_IsStopConversionOngoing
  * @param  ADCx ADC instance
  * @retval 0: no command of conversion stop is on going on ADC group regular.
  */
__STATIC_INLINE uint32_t LL_ADC_REG_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->CR, ADC_CR_ADSTP) == (ADC_CR_ADSTP));
}

/**
  * @brief  Get ADC group regular conversion data, range fit for
  *         all ADC configurations: all ADC resolutions and
  *         all oversampling increased data width (for devices
  *         with feature oversampling).
  * @rmtoll DR       DATA           LL_ADC_REG_ReadConversionData32
  * @param  ADCx ADC instance
  * @retval Value between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t LL_ADC_REG_ReadConversionData32(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->DR, ADC_DR_DATA));
}

/**
  * @brief  Get ADC group regular conversion data, range fit for
  *         ADC resolution 12 bits.
  * @note   For devices with feature oversampling: Oversampling
  *         can increase data width, function for extended range
  *         may be needed: @ref LL_ADC_REG_ReadConversionData32.
  * @rmtoll DR       DATA           LL_ADC_REG_ReadConversionData12
  * @param  ADCx ADC instance
  * @retval Value between Min_Data=0x000 and Max_Data=0xFFF
  */
__STATIC_INLINE uint16_t LL_ADC_REG_ReadConversionData12(ADC_TypeDef *ADCx)
{
  return (uint16_t)(READ_BIT(ADCx->DR, ADC_DR_DATA));
}

/**
  * @brief  Get ADC group regular conversion data, range fit for
  *         ADC resolution 10 bits.
  * @note   For devices with feature oversampling: Oversampling
  *         can increase data width, function for extended range
  *         may be needed: @ref LL_ADC_REG_ReadConversionData32.
  * @rmtoll DR       DATA           LL_ADC_REG_ReadConversionData10
  * @param  ADCx ADC instance
  * @retval Value between Min_Data=0x000 and Max_Data=0x3FF
  */
__STATIC_INLINE uint16_t LL_ADC_REG_ReadConversionData10(ADC_TypeDef *ADCx)
{
  return (uint16_t)(READ_BIT(ADCx->DR, ADC_DR_DATA));
}

/**
  * @brief  Get ADC group regular conversion data, range fit for
  *         ADC resolution 8 bits.
  * @note   For devices with feature oversampling: Oversampling
  *         can increase data width, function for extended range
  *         may be needed: @ref LL_ADC_REG_ReadConversionData32.
  * @rmtoll DR       DATA           LL_ADC_REG_ReadConversionData8
  * @param  ADCx ADC instance
  * @retval Value between Min_Data=0x00 and Max_Data=0xFF
  */
__STATIC_INLINE uint8_t LL_ADC_REG_ReadConversionData8(ADC_TypeDef *ADCx)
{
  return (uint8_t)(READ_BIT(ADCx->DR, ADC_DR_DATA));
}

/**
  * @brief  Get ADC group regular conversion data, range fit for
  *         ADC resolution 6 bits.
  * @note   For devices with feature oversampling: Oversampling
  *         can increase data width, function for extended range
  *         may be needed: @ref LL_ADC_REG_ReadConversionData32.
  * @rmtoll DR       DATA           LL_ADC_REG_ReadConversionData6
  * @param  ADCx ADC instance
  * @retval Value between Min_Data=0x00 and Max_Data=0x3F
  */
__STATIC_INLINE uint8_t LL_ADC_REG_ReadConversionData6(ADC_TypeDef *ADCx)
{
  return (uint8_t)(READ_BIT(ADCx->DR, ADC_DR_DATA));
}

/**
  * @}
  */

/** @defgroup ADC_LL_EF_FLAG_Management ADC flag management
  * @{
  */

/**
  * @brief  Get flag ADC group regular end of unitary conversion.
  * @rmtoll ISR      EOC            LL_ADC_IsActiveFlag_EOC
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsActiveFlag_EOC(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->ISR, ADC_ISR_EOC) == (ADC_ISR_EOC));
}

/**
  * @brief  Get flag ADC group regular end of sequence conversions.
  * @rmtoll ISR      EOSEQ          LL_ADC_IsActiveFlag_EOS
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsActiveFlag_EOS(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->ISR, LL_ADC_FLAG_EOS) == (LL_ADC_FLAG_EOS));
}

/**
  * @brief  Get flag ADC group regular overrun.
  * @rmtoll ISR      OVR            LL_ADC_IsActiveFlag_OVR
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsActiveFlag_OVR(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->ISR, LL_ADC_FLAG_OVR) == (LL_ADC_FLAG_OVR));
}

/**
  * @brief  Get flag ADC group regular end of sampling phase.
  * @rmtoll ISR      EOSMP          LL_ADC_IsActiveFlag_EOSMP
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsActiveFlag_EOSMP(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->ISR, LL_ADC_FLAG_EOSMP) == (LL_ADC_FLAG_EOSMP));
}

/**
  * @brief  Get flag ADC analog watchdog 1 flag
  * @rmtoll ISR      AWD            LL_ADC_IsActiveFlag_AWD
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsActiveFlag_AWD(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->ISR, LL_ADC_FLAG_AWD) == (LL_ADC_FLAG_AWD));
}

/**
  * @brief  Clear flag ADC group regular end of unitary conversion.
  * @rmtoll ISR      EOC            LL_ADC_ClearFlag_EOC
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_ClearFlag_EOC(ADC_TypeDef *ADCx)
{
  WRITE_REG(ADCx->ISR, LL_ADC_FLAG_EOC);
}

/**
  * @brief  Clear flag ADC group regular end of sequence conversions.
  * @rmtoll ISR      EOSEQ          LL_ADC_ClearFlag_EOS
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_ClearFlag_EOS(ADC_TypeDef *ADCx)
{
  WRITE_REG(ADCx->ISR, LL_ADC_FLAG_EOS);
}

/**
  * @brief  Clear flag ADC group regular overrun.
  * @rmtoll ISR      OVR            LL_ADC_ClearFlag_OVR
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_ClearFlag_OVR(ADC_TypeDef *ADCx)
{
  WRITE_REG(ADCx->ISR, LL_ADC_FLAG_OVR);
}

/**
  * @brief  Clear flag ADC group regular end of sampling phase.
  * @rmtoll ISR      EOSMP          LL_ADC_ClearFlag_EOSMP
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_ClearFlag_EOSMP(ADC_TypeDef *ADCx)
{
  WRITE_REG(ADCx->ISR, LL_ADC_FLAG_EOSMP);
}

/**
  * @brief  Clear flag ADC analog watchdog 1.
  * @rmtoll ISR      AWD            LL_ADC_ClearFlag_AWD
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_ClearFlag_AWD(ADC_TypeDef *ADCx)
{
  WRITE_REG(ADCx->ISR, LL_ADC_FLAG_AWD);
}

/**
  * @}
  */

/** @defgroup ADC_LL_EF_IT_Management ADC IT management
  * @{
  */
/**
  * @brief  Enable interruption ADC group regular end of unitary conversion.
  * @rmtoll IER      EOCIE          LL_ADC_EnableIT_EOC
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_EnableIT_EOC(ADC_TypeDef *ADCx)
{
  SET_BIT(ADCx->IER, LL_ADC_IT_EOC);
}

/**
  * @brief  Enable interruption ADC group regular end of sequence conversions.
  * @rmtoll IER      EOSEQIE        LL_ADC_EnableIT_EOS
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_EnableIT_EOS(ADC_TypeDef *ADCx)
{
  SET_BIT(ADCx->IER, LL_ADC_IT_EOS);
}

/**
  * @brief  Enable ADC group regular interruption overrun.
  * @rmtoll IER      OVRIE          LL_ADC_EnableIT_OVR
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_EnableIT_OVR(ADC_TypeDef *ADCx)
{
  SET_BIT(ADCx->IER, LL_ADC_IT_OVR);
}

/**
  * @brief  Enable interruption ADC group regular end of sampling.
  * @rmtoll IER      EOSMPIE        LL_ADC_EnableIT_EOSMP
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_EnableIT_EOSMP(ADC_TypeDef *ADCx)
{
  SET_BIT(ADCx->IER, LL_ADC_IT_EOSMP);
}

/**
  * @brief  Enable interruption ADC analog watchdog 1.
  * @rmtoll IER      AWDIE          LL_ADC_EnableIT_AWD
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_EnableIT_AWD(ADC_TypeDef *ADCx)
{
  SET_BIT(ADCx->IER, LL_ADC_IT_AWD);
}

/**
  * @brief  Disable interruption ADC group regular end of unitary conversion.
  * @rmtoll IER      EOCIE          LL_ADC_DisableIT_EOC
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_DisableIT_EOC(ADC_TypeDef *ADCx)
{
  CLEAR_BIT(ADCx->IER, LL_ADC_IT_EOC);
}

/**
  * @brief  Disable interruption ADC group regular end of sequence conversions.
  * @rmtoll IER      EOSEQIE        LL_ADC_DisableIT_EOS
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_DisableIT_EOS(ADC_TypeDef *ADCx)
{
  CLEAR_BIT(ADCx->IER, LL_ADC_IT_EOS);
}

/**
  * @brief  Disable interruption ADC group regular overrun.
  * @rmtoll IER      OVRIE          LL_ADC_DisableIT_OVR
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_DisableIT_OVR(ADC_TypeDef *ADCx)
{
  CLEAR_BIT(ADCx->IER, LL_ADC_IT_OVR);
}

/**
  * @brief  Disable interruption ADC group regular end of sampling.
  * @rmtoll IER      EOSMPIE        LL_ADC_DisableIT_EOSMP
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_DisableIT_EOSMP(ADC_TypeDef *ADCx)
{
  CLEAR_BIT(ADCx->IER, LL_ADC_IT_EOSMP);
}

/**
  * @brief  Disable interruption ADC analog watchdog 1.
  * @rmtoll IER      AWDIE          LL_ADC_DisableIT_AWD
  * @param  ADCx ADC instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_DisableIT_AWD(ADC_TypeDef *ADCx)
{
  CLEAR_BIT(ADCx->IER, LL_ADC_IT_AWD);
}

/**
  * @brief  Get state of interruption ADC group regular end of unitary conversion
  *         (0: interrupt disabled, 1: interrupt enabled).
  * @rmtoll IER      EOCIE          LL_ADC_IsEnabledIT_EOC
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsEnabledIT_EOC(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->IER, LL_ADC_IT_EOC) == (LL_ADC_IT_EOC));
}

/**
  * @brief  Get state of interruption ADC group regular end of sequence conversions
  *         (0: interrupt disabled, 1: interrupt enabled).
  * @rmtoll IER      EOSEQIE        LL_ADC_IsEnabledIT_EOS
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsEnabledIT_EOS(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->IER, LL_ADC_IT_EOS) == (LL_ADC_IT_EOS));
}

/**
  * @brief  Get state of interruption ADC group regular overrun
  *         (0: interrupt disabled, 1: interrupt enabled).
  * @rmtoll IER      OVRIE          LL_ADC_IsEnabledIT_OVR
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsEnabledIT_OVR(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->IER, LL_ADC_IT_OVR) == (LL_ADC_IT_OVR));
}

/**
  * @brief  Get state of interruption ADC group regular end of sampling
  *         (0: interrupt disabled, 1: interrupt enabled).
  * @rmtoll IER      EOSMPIE        LL_ADC_IsEnabledIT_EOSMP
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsEnabledIT_EOSMP(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->IER, LL_ADC_IT_EOSMP) == (LL_ADC_IT_EOSMP));
}

/**
  * @brief  Get state of interruption ADC analog watchdog 1
  *         (0: interrupt disabled, 1: interrupt enabled).
  * @rmtoll IER      AWDIE          LL_ADC_IsEnabledIT_AWD
  * @param  ADCx ADC instance
  * @retval State of bit (1 or 0).   
  */
__STATIC_INLINE uint32_t LL_ADC_IsEnabledIT_AWD(ADC_TypeDef *ADCx)
{
  return (READ_BIT(ADCx->IER, LL_ADC_IT_AWD) == (LL_ADC_IT_AWD));
}
/**
  * @}
  */

/** @defgroup ADC_LL_EF_Calibration_Management ADC Calibration management
  * @{
  */
/**
  * @brief  Set calibration sample time
  * @note   Unit: ADC clock cycles.
  * @rmtoll CCSR     CALSMP            LL_ADC_SetCalibrationSamplingTime
  * @param  ADCx ADC instance
  * @param  CalibrationSamplingTime This parameter can be one of the following values:
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_2P5CYCLES
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_4P5CYCLES
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_8P5CYCLES
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_1P5CYCLE
  * @retval None   
  */
__STATIC_INLINE void LL_ADC_SetCalibrationSamplingTime(ADC_TypeDef *ADCx, uint32_t CalibrationSamplingTime)
{
  MODIFY_REG(ADCx->CCSR, ADC_CCSR_CALSMP, CalibrationSamplingTime);
}

/**
  * @brief  Get sampling time common to a group of channels.
  * @note   Unit: ADC clock cycles.
  * @rmtoll CCSR     CALSMP            LL_ADC_GetCalibrationSamplingTime
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_2P5CYCLES
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_4P5CYCLES
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_8P5CYCLES
  *         @arg @ref LL_ADC_CAL_SAMPLINGTIME_1P5CYCLE   
  */
__STATIC_INLINE uint32_t LL_ADC_GetCalibrationSamplingTime(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CCSR, ADC_CCSR_CALSMP));
}

/**
  * @brief  Get the calibration offset status
  * @rmtoll CCSR     OFFSUC         LL_ADC_GetCalibrationOffsetStatus
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CAL_OFFSET_STATUS_SUCCESS
  *         @arg @ref LL_ADC_CAL_OFFSET_STATUS_FAIL 
  */
__STATIC_INLINE uint32_t LL_ADC_GetCalibrationOffsetStatus(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CCSR, ADC_CCSR_OFFSUC));
}

/**
  * @brief  Get the calibration capacitance status
  * @rmtoll CCSR     CAPSUC        LL_ADC_GetCalibrationCapacitanceStatus
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CAL_CAPACITANCE_STATUS_SUCCESS
  *         @arg @ref LL_ADC_CAL_CAPACITANCE_STATUS_FAIL 
  */
__STATIC_INLINE uint32_t LL_ADC_GetCalibrationCapacitanceStatus(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CCSR, ADC_CCSR_CAPSUC));
}

/**
  * @brief  Set the calibration mode
  * @rmtoll CCSR     CALSEL            LL_ADC_SetCalibrationMode
  * @param  ADCx ADC instance
  * @param  CalibrationMode This parameter can be one of the following values:
            @arg @ref LL_ADC_CAL_MODE_OFFSET
  *         @arg @ref LL_ADC_CAL_MODE_OFFSET_CAPACITANCE
  * @retval None     
  */
__STATIC_INLINE void LL_ADC_SetCalibrationMode(ADC_TypeDef *ADCx,uint32_t CalibrationMode)
{
  MODIFY_REG(ADCx->CCSR,ADC_CCSR_CALSEL, CalibrationMode);
}

/**
  * @brief  Get the calibration mode
  * @rmtoll CCSR     CALSEL            LL_ADC_GetCalibrationMode
  * @param  ADCx ADC instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_CAL_MODE_OFFSET
  *         @arg @ref LL_ADC_CAL_MODE_OFFSET_CAPACITANCE
  */
__STATIC_INLINE uint32_t LL_ADC_GetCalibrationMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(READ_BIT(ADCx->CCSR, ADC_CCSR_CALSEL));
}
/**
  * @}
  */


/** @defgroup ADC_LL_EF_VrefBuf ADC Vrefbuf Management
  * @{
  */
  
/**
  * @brief  Enable VrefBuffer Output.
  * @rmtoll CCR      VREFBUF_EN          LL_ADC_EnableVrefBuffVoltage
  * @param  ADCxy_COMMON ADC common instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_EnableVrefBufferVoltage(ADC_Common_TypeDef *ADCxy_COMMON)
{
  SET_BIT(ADCxy_COMMON->CCR, ADC_CCR_VREFBUF_EN);
}
 
/**
  * @brief  Disable VrefBuffer Output.
  * @rmtoll CCR      VREFBUF_EN          LL_ADC_DisableVrefBuffVoltage
  * @param  ADCxy_COMMON ADC common instance
  * @retval None
  */
__STATIC_INLINE void LL_ADC_DisableVrefBufferVoltage(ADC_Common_TypeDef *ADCxy_COMMON)
{
  CLEAR_BIT(ADCxy_COMMON->CCR, ADC_CCR_VREFBUF_EN);
}
 
/**
  * @brief  Get state of VrefBuff Voltage output
  *         (0: output disabled, 1: output enabled).
  * @rmtoll CCR      VREFBUF_EN          LL_ADC_IsEnabled_VrefBuffVoltage
  * @param  ADCxy_COMMON ADC common instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_ADC_IsEnabled_VrefBufferVoltage(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (READ_BIT(ADCxy_COMMON->CCR, ADC_CCR_VREFBUF_EN) == (ADC_CCR_VREFBUF_EN));
}
 
/**
  * @brief  Set ADC VrefBuffer Voltage.
  * @rmtoll CR       VREFBUFF_SEL   LL_ADC_SetVrefBufferVoltage
  * @param  ADCxy_COMMON ADC common instance
  * @param  VrefBufferVoltage This parameter can be one of the following values:
  *         @arg @ref LL_ADC_VREFBUF_0P6V
  *         @arg @ref LL_ADC_VREFBUF_1P5V
  *         @arg @ref LL_ADC_VREFBUF_2P048V 
  *         @arg @ref LL_ADC_VREFBUF_2P5V   
  * @retval None
  */
__STATIC_INLINE void LL_ADC_SetVrefBufferVoltage(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t VrefBufferVoltage)
{
  MODIFY_REG(ADCxy_COMMON->CCR, ADC_CCR_VREFBUF_SEL, VrefBufferVoltage);
}
 
/**
  * @brief  Get ADC VrefBuffer Voltage.
  * @rmtoll CCR       VREFBUFF_SEL      LL_ADC_GetVrefBufferVoltage
  * @param  ADCxy_COMMON ADC common instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_ADC_VREFBUF_0P6V
  *         @arg @ref LL_ADC_VREFBUF_1P5V
  *         @arg @ref LL_ADC_VREFBUF_2P048V 
  *         @arg @ref LL_ADC_VREFBUF_2P5V   
  */
__STATIC_INLINE uint32_t LL_ADC_GetVrefBufferVoltage(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(READ_BIT(ADCxy_COMMON->CCR, ADC_CCR_VREFBUF_SEL));
}
/**
  * @}
  */
 
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup ADC_LL_EF_Init Initialization and de-initialization functions
  * @{
  */

/* Initialization of some features of ADC common parameters and multimode */
ErrorStatus LL_ADC_CommonDeInit(ADC_Common_TypeDef *ADCxy_COMMON);

/* De-initialization of ADC instance */
ErrorStatus LL_ADC_DeInit(ADC_TypeDef *ADCx);

/* Initialization of some features of ADC instance */
ErrorStatus LL_ADC_Init(ADC_TypeDef *ADCx, LL_ADC_InitTypeDef *ADC_InitStruct);
void        LL_ADC_StructInit(LL_ADC_InitTypeDef *ADC_InitStruct);

/* Initialization of some features of ADC instance and ADC group regular */
ErrorStatus LL_ADC_REG_Init(ADC_TypeDef *ADCx, LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct);
void        LL_ADC_REG_StructInit(LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct);

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

#endif /* ADC1 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __PY32T020_LL_ADC_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
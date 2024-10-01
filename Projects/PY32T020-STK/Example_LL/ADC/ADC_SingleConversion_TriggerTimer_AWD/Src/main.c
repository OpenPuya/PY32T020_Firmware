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
#include <cstdint>
#include <stdint.h>

/* Private define ------------------------------------------------------------*/
#define ADC_CALIBRATION_TIMEOUT_MS ((uint32_t)1)

/* Private variables ---------------------------------------------------------*/
uint16_t ADCCoversionValue[3] = {0};
uint8_t ADCCoversionFlag = 0;
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_AdcEnable(void);
static void APP_AdcCalibrate(void);
static void APP_AdcConfig(void);
static void APP_TimerInit(void);

/**
 * @brief  Main program.
 * @param  None
 * @retval int
 */
int main(void) {
  /* Enable SYSCFG clock and PWR clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure Systemclock */
  APP_SystemClockConfig();

  /* Initialize LED */
  BSP_LED_Init(LED_TK1);

  /* Configure ADC parameters */
  APP_AdcConfig();

  /* ADC automatic self-calibration */
  APP_AdcCalibrate();

  /* Enable ADC */
  APP_AdcEnable();

  while (1) {
    /* Start ADC conversion (if it is software triggered then start conversion
     * directly) */
    LL_ADC_REG_StartConversion(ADC1);
    while (ADCCoversionFlag == 0) {
    }

    
  }
}

/**
 * @brief  Configure ADC parameters
 * @param  None
 * @retval None
 */
static void APP_AdcConfig(void) {
  /* Enable ADC1 clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  /* Set ADC clock to pclk/4 */
  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV4);

  /* Set ADC resolution to 12 bit */
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);

  /* ADC conversion data alignment: right aligned */
  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

  /* No ADC low power mode activated */
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);

  /* Sampling time 239.5 ADC clock cycles */
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

  /* ADC regular group conversion trigger from external IP: TIM1 TRGO. */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

  /* Set ADC conversion mode to single mode: one conversion per trigger */
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

  /* ADC regular group behavior in case of overrun: data overwritten */
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

  /* Disable ADC regular group sequencer discontinuous mode  */
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);

  /* Set channel 5 as conversion channel */
  LL_ADC_REG_SetSequencerChannels(
      ADC1, LL_ADC_CHANNEL_0 | LL_ADC_CHANNEL_5 |
                LL_ADC_CHANNEL_VREFINT); // 这里修改为你的实际通道

  /* Dose not enable internal conversion channel */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_VREFINT);

  /* Set channel VREFINT as ADC analog watchdog channel */
  LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_CH_VREFINT_REG);

  // 保护电压
  const uint32_t VDDA_APPLI = 3300; // mv

  // 内部1.2V参考电压，因此阈值设置为0-1500
  LL_ADC_ConfigAnalogWDThresholds(ADC1, 1500, 0);

  NVIC_SetPriority(ADC_COMP_IRQn, 0);
  NVIC_EnableIRQ(ADC_COMP_IRQn);

  // 开启数据EOC和EOS中断
  LL_ADC_EnableIT_EOC(ADC1);
  LL_ADC_EnableIT_EOS(ADC1);

  /* Enable ADC analog watchdog IT */
  LL_ADC_EnableIT_AWD(ADC1);
}

/**
 * @brief  ADC calibration program.
 * @param  None
 * @retval None
 */
static void APP_AdcCalibrate(void) {
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0;
#endif

  if (LL_ADC_IsEnabled(ADC1) == 0) {

    /* Enable ADC calibration */
    LL_ADC_StartCalibration(ADC1);

#if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
#endif

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {
#if (USE_TIMEOUT == 1)
      /* Detects if the calibration has timed out */
      if (LL_SYSTICK_IsActiveCounterFlag()) {
        if (Timeout-- == 0) {
          APP_ErrorHandler();
        }
      }
#endif
    }

    /* The delay between the end of ADC calibration and ADC enablement is at
     * least 4 ADC clocks */
    LL_mDelay(1);
  }
}

/**
 * @brief  Enable ADC.
 * @param  None
 * @retval None
 */
static void APP_AdcEnable(void) {
  /* Enable ADC */
  LL_ADC_Enable(ADC1);

  /* The delay between ADC enablement and ADC stabilization is at least 8 ADC
   * clocks */
  LL_mDelay(1);
}

/**
 * @brief  ADC interruption handler callback program.
 * @param  None
 * @retval None
 */
void APP_AdcAWDCallback() { BSP_LED_On(LED3); }

/**
 * @brief  Configure systemclock
 * @param  None
 * @retval None
 */
static void APP_SystemClockConfig(void) {
  /* Enable HSI */
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1) {
  }

  /* Set AHB prescaler: HCLK = SYSCLK */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Select HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS) {
  }

  /* Set APB prescaler: PCLK = HCLK */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);

  /* Update the SystemCoreClock global variable(which can be updated also
   * through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
}

/**
 * @brief  Error handling function
 * @param  None
 * @retval None
 */
void APP_ErrorHandler(void) {
  /* Infinite loop */
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file:Pointer to the source file name
 * @param  line:assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add His own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* Infinite loop */
  while (1) {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

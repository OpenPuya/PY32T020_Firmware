/**
  ******************************************************************************
  * @file    py32t020x5.h
  * @brief   CMSIS Cortex-M0+ Device Peripheral Access Layer Header File.
  *          This file contains all the peripheral register's definitions, bits
  *          definitions and memory mapping for PY32T0xx devices.
  * @version v1.0.0
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


/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup py32t020x5
  * @{
  */

#ifndef __PY32T020X5_H
#define __PY32T020X5_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M0+ Processor and Core Peripherals
   */
#define __CM0PLUS_REV             0 /*!< Core Revision r0p0                            */
#define __MPU_PRESENT             0 /*!< PY32T0xx do not provide MPU                  */
#define __VTOR_PRESENT            1 /*!< Vector  Table  Register supported             */
#define __NVIC_PRIO_BITS          2 /*!< PY32T0xx uses 2 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0 /*!< Set to 1 if different SysTick Config is used  */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief PY32T0xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */

/*!< Interrupt Number Definition */
typedef enum
{
  /******  Cortex-M0+ Processor Exceptions Numbers *************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M Hard Fault Interrupt                                   */
  SVC_IRQn                    = -5,     /*!< 11 Cortex-M SV Call Interrupt                                     */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M Pend SV Interrupt                                     */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M System Tick Interrupt                                 */
  /******  PY32T0 specific Interrupt Numbers *******************************************************************/
  RTC_IRQn                    = 2,      /*!< RTC interrupt through the EXTI line 19                            */
  FLASH_IRQn                  = 3,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 4,      /*!< RCC global Interrupt                                              */
  EXTI0_1_IRQn                = 5,      /*!< EXTI 0 and 1 Interrupts                                           */
  EXTI2_3_IRQn                = 6,      /*!< EXTI Line 2 and 3 Interrupts                                      */
  EXTI4_15_IRQn               = 7,      /*!< EXTI Line 4 to 15 Interrupts                                      */
  TK_IRQn                     = 8,      /*!< TK Interrupt                                                      */
  ADC_COMP_IRQn               = 12,     /*!< ADC COMP Interrupts                                               */
  TIM1_BRK_UP_TRG_COM_IRQn    = 13,     /*!< TIM1 Break, Update, Trigger and Commutation Interrupts            */
  TIM1_CC_IRQn                = 14,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM14_IRQn                  = 19,     /*!< TIM14 global Interrupt                                            */
  I2C1_IRQn                   = 23,     /*!< I2C1 Interrupt  (combined with EXTI 23)                           */
  SPI1_IRQn                   = 25,     /*!< SPI1 Interrupt                                                    */
  UART3_IRQn                  = 29,     /*!< UART3 Interrupt                                                   */
  UART2_IRQn                  = 30,     /*!< UART2 Interrupt                                                   */
  UART1_IRQn                  = 31,     /*!< UART1 Interrupt                                                   */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm0plus.h"               /* Cortex-M0+ processor and core peripherals */
#include "system_py32t0xx.h"            /* PY32T0xx System Header */
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */
typedef struct
{
  __IO uint32_t ISR;          /*!< ADC interrupt and status register,             Address offset: 0x00 */
  __IO uint32_t IER;          /*!< ADC interrupt enable register,                 Address offset: 0x04 */
  __IO uint32_t CR;           /*!< ADC control register,                          Address offset: 0x08 */
  __IO uint32_t CFGR1;        /*!< ADC configuration register 1,                  Address offset: 0x0C */
  __IO uint32_t RESERVED0;    /*!< Reserved,                                      Address offset: 0x10 */
  __IO uint32_t SMPR;         /*!< ADC sampling time register,                    Address offset: 0x14 */
  __IO uint32_t RESERVED1[2]; /*!< Reserved,                                                 0x18-0x1C */
  __IO uint32_t TR;           /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
  __IO uint32_t RESERVED2;    /*!< Reserved,                                                      0x24 */
  __IO uint32_t CHSELR;       /*!< ADC group regular sequencer register,          Address offset: 0x28 */
  __IO uint32_t RESERVED3[5]; /*!< Reserved,                                                      0x2C */
  __IO uint32_t DR;           /*!< ADC group regular data register,               Address offset: 0x40 */
  __IO uint32_t CCSR;         /*!< ADC calibration configuration&status register  Address offset: 0x44 */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t CCR;          /*!< ADC common configuration register,             Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

/**
  * @brief Comparator
  */
typedef struct
{
  __IO uint32_t CSR;           /*!< COMP control and status register,           Address offset: 0x00 */
  __IO uint32_t FR;            /*!< COMP filter register,                       Address offset: 0x04 */
} COMP_TypeDef;

typedef struct
{
  __IO uint32_t CSR_ODD;    /*!< COMP control and status register located in register of comparator instance odd, used for bits common to several COMP instances, Address offset: 0x00 */
  __IO uint32_t FR_ODD;
  uint32_t RESERVED[2];     /*Reserved*/
  __IO uint32_t CSR_EVEN;   /*!< COMP control and status register located in register of comparator instance even, used for bits common to several COMP instances, Address offset: 0x04 */
  __IO uint32_t FR_EVEN;
} COMP_Common_TypeDef;

/**
  * @brief CRC calculation unit
  */
typedef struct
{
  __IO uint32_t DR;             /*!< CRC Data register,                         Address offset: 0x00 */
  __IO uint32_t IDR;            /*!< CRC Independent data register,             Address offset: 0x04 */
  __IO uint32_t CR;             /*!< CRC Control register,                      Address offset: 0x08 */
} CRC_TypeDef;

/**
  * @brief Debug MCU
  */
typedef struct
{
  __IO uint32_t IDCODE;      /*!< MCU device ID code,              Address offset: 0x00 */
  __IO uint32_t CR;          /*!< Debug configuration register,    Address offset: 0x04 */
  __IO uint32_t APBFZ1;      /*!< Debug APB freeze register 1,     Address offset: 0x08 */
  __IO uint32_t APBFZ2;      /*!< Debug APB freeze register 2,     Address offset: 0x0C */
} DBGMCU_TypeDef;


/**
  * @brief Asynch Interrupt/Event Controller (EXTI)
  */
typedef struct
{
  __IO uint32_t RTSR;          /*!< EXTI Rising Trigger Selection Register 1,        Address offset:   0x00 */
  __IO uint32_t FTSR;          /*!< EXTI Falling Trigger Selection Register 1,       Address offset:   0x04 */
  __IO uint32_t SWIER;         /*!< EXTI Software Interrupt event Register 1,        Address offset:   0x08 */
  __IO uint32_t PR;            /*!< EXTI Pending Register 1                          Address offset:   0x0C */
  __IO uint32_t RESERVED0[20]; /*!< Reserved 1,                                                0x10 -- 0x5F */
  __IO uint32_t EXTICR[2];     /*!< EXTI External Interrupt Configuration Register,            0x60 -- 0x67 */
  __IO uint32_t RESERVED1[6];  /*!< Reserved 2,                                                0x68 -- 0x7F */
  __IO uint32_t IMR;           /*!< EXTI Interrupt Mask Register ,                   Address offset:   0x80 */
  __IO uint32_t EMR;           /*!< EXTI Event Mask Register ,                       Address offset:   0x84 */
} EXTI_TypeDef;

/**
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t ACR;          /*!< FLASH Access Control register,                     Address offset: 0x00 */
  __IO uint32_t RESERVED1;    /*!< Reserved1,                                         Address offset: 0x04 */
  __IO uint32_t KEYR;         /*!< FLASH Key register,                                Address offset: 0x08 */
  __IO uint32_t OPTKEYR;      /*!< FLASH Option Key register,                         Address offset: 0x0C */
  __IO uint32_t SR;           /*!< FLASH Status register,                             Address offset: 0x10 */
  __IO uint32_t CR;           /*!< FLASH Control register,                            Address offset: 0x14 */
  __IO uint32_t RESERVED2[2]; /*!< Reserved2,                                         Address offset: 0x18-0x1C */
  __IO uint32_t OPTR;         /*!< FLASH Option register,                             Address offset: 0x20 */
  __IO uint32_t SDKR;         /*!< FLASH SDK address register,                        Address offset: 0x24 */
  __IO uint32_t BTCR;         /*!< FLASH boot control                                 Address offset: 0x28 */
  __IO uint32_t WRPR;         /*!< FLASH WRP address register,                        Address offset: 0x2C */
  __IO uint32_t RESERVED3[(0x90 - 0x2C) / 4 - 1];
  __IO uint32_t STCR;         /*!< FLASH sleep time config register,                  Address offset: 0x90 */
  __IO uint32_t RESERVED4[(0x100 - 0x90) / 4 - 1];
  __IO uint32_t TS0;          /*!< FLASH TS0 register,                                Address offset: 0x100 */
  __IO uint32_t TS1;          /*!< FLASH TS1 register,                                Address offset: 0x104 */
  __IO uint32_t TS2P;         /*!< FLASH TS2P register,                               Address offset: 0x108 */
  __IO uint32_t TPS3;         /*!< FLASH TPS3 register,                               Address offset: 0x10C */
  __IO uint32_t TS3;          /*!< FLASH TS3 register,                                Address offset: 0x110 */
  __IO uint32_t PERTPE;       /*!< FLASH PERTPE register,                             Address offset: 0x114 */
  __IO uint32_t SMERTPE;      /*!< FLASH SMERTPE register,                            Address offset: 0x118 */
  __IO uint32_t PRGTPE;       /*!< FLASH PRGTPE register,                             Address offset: 0x11C */
  __IO uint32_t PRETPE;       /*!< FLASH PRETPE register,                             Address offset: 0x120 */
} FLASH_TypeDef;

/**
  * @brief Option Bytes
  */
typedef struct
{
  __IO uint8_t RDP;          /*!< Reserved,                                       Address offset: 0x00 */
  __IO uint8_t USER;         /*!< FLASH option byte user options,                 Address offset: 0x01 */
  __IO uint8_t nRDP;         /*!< Reserved,                                       Address offset: 0x02 */
  __IO uint8_t nUSER;        /*!< Complemented FLASH option byte user options,    Address offset: 0x03 */
  __IO uint8_t SDK_STRT;     /*!< SDK area start address(stored in SDK[4:0]),     Address offset: 0x04 */
  __IO uint8_t SDK_END;      /*!< SDK area end address(stored in SDK[12:8]),      Address offset: 0x05 */
  __IO uint8_t nSDK_STRT;    /*!< Complemented SDK area start address,            Address offset: 0x06 */
  __IO uint8_t nSDK_END;     /*!< Complemented SDK area end address,              Address offset: 0x07 */
  __IO uint8_t BOOTSIZE;     /*!< BOOT size(stored in BOOTSIZZE[2:0]),            Address offset: 0x08 */
  __IO uint8_t SWDMODE;      /*!< SWD mode select(stored in SWDMODE[1:0]),        Address offset: 0x09 */
  __IO uint8_t nBOOTSIZE;    /*!< Complemented BOOT size,                         Address offset: 0x0A */
  __IO uint8_t nSWDMODE;     /*!< Complemented SDK SWD mode,                      Address offset: 0x0B */
  __IO uint16_t WRP;         /*!< FLASH option byte write protection,             Address offset: 0x0C */
  __IO uint16_t nWRP;        /*!< Complemented FLASH option byte write protection,Address offset: 0x0E */
} OB_TypeDef;

/**
  * @brief General Purpose I/O
  */
typedef struct
{
  __IO uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  __IO uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
} GPIO_TypeDef;

/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct
{
  __IO uint32_t CR1;         /*I2C Control register1,                    Address offset: 0x00      */
  __IO uint32_t CR2;         /*I2C Control register2,                    Address offset: 0x04      */
  __IO uint32_t OAR1;        /*I2C Own address register1,                Address offset: 0x08      */
       uint32_t RESERVED1;   /*!< RESERVED1,                             Address offset: 0x0C      */
  __IO uint32_t DR;          /*I2C Data register ,                       Address offset: 0x10      */
  __IO uint32_t SR1;         /*I2C Status register1 ,                    Address offset: 0x14      */
  __IO uint32_t SR2;         /*I2C Status register2 ,                    Address offset: 0x18      */
  __IO uint32_t CCR;         /*I2C Clock control register ,              Address offset: 0x1C      */
  __IO uint32_t TRISE;       /*I2C TRISE register ,                      Address offset: 0x20      */
  __IO uint32_t WT;          /*I2C Wakeup Time register ,                Address offset: 0x24      */  
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */
typedef struct
{
  __IO uint32_t KR;          /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;          /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;         /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;          /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;

/**
  * @brief Power Control
  */
typedef struct
{
  __IO uint32_t CR1;          /*!< PWR Power Control Register 1,                     Address offset: 0x00 */
  __IO uint32_t RESERVED1[7];
} PWR_TypeDef;

/**
  * @brief Touch Key 
  */
typedef struct
{
  __IO uint32_t CR1;          /*!< TK Control Register 1,                                Address offset: 0x00 */
  __IO uint32_t CR2;          /*!< TK Control Register 2,                                Address offset: 0x04 */
  __IO uint32_t IDACR;        /*!< TK IDAC Control Register,                             Address offset: 0x08 */
  __IO uint32_t KR;           /*!< TK Key enable Control Register,                       Address offset: 0x0C */
  __IO uint32_t CSR;          /*!< TK Channel Select Register,                           Address offset: 0x10 */
  __IO uint32_t UCR;          /*!< TK Unsel Control Register,                            Address offset: 0x14 */
  __IO uint32_t STR1;         /*!< TK Stable Time Register 1,                            Address offset: 0x18 */
  __IO uint32_t STR2;         /*!< TK Stable Time Register 2,                            Address offset: 0x1C */
  __IO uint32_t WR;           /*!< TK Window Register,                                   Address offset: 0x20 */
  __IO uint32_t SDR;          /*!< TK Switch Divide Register,                            Address offset: 0x24 */
  __IO uint32_t PRSR;         /*!< TK Pseudo Register,                                   Address offset: 0x28 */
  __IO uint32_t DR;           /*!< TK Data Register,                                     Address offset: 0x2C */
  __IO uint32_t SR;           /*!< TK Status Register,                                   Address offset: 0x30 */
  __IO uint32_t IER;          /*!< TK Interrupt Enable Register,                         Address offset: 0x34 */
  __IO uint32_t ICR;          /*!< TK Interrupt Clear Register,                          Address offset: 0x38 */
  __IO uint32_t CDR;          /*!< TK Compare Data Register,                             Address offset: 0x3C */
  __IO uint32_t LPDR;         /*!< TK Lowpower Delta Register,                           Address offset: 0x40 */
  __IO uint32_t LPTR;         /*!< TK Lowpower Time Register,                            Address offset: 0x44 */
  __IO uint32_t OPR;          /*!< TK OP Register,                                       Address offset: 0x48 */
} TK_TypeDef;

/**
  * @brief Reset and Clock Control
  */
typedef struct
{
  __IO uint32_t CR;          /*!< RCC Clock Sources Control Register,                                     Address offset: 0x00 */
  __IO uint32_t ICSCR;       /*!< RCC Internal Clock Sources Calibration Register,                        Address offset: 0x04 */
  __IO uint32_t CFGR;        /*!< RCC Regulated Domain Clocks Configuration Register,                     Address offset: 0x08 */
  __IO uint32_t RESERVED0;   /*!< Reserved,                                                               Address offset: 0x0C */
  __IO uint32_t ECSCR;       /*!< RCC External clock source control register,                             Address offset: 0x10 */
  __IO uint32_t RESERVED1;   /*!< Reserved,                                                               Address offset: 0x14 */
  __IO uint32_t CIER;        /*!< RCC Clock Interrupt Enable Register,                                    Address offset: 0x18 */
  __IO uint32_t CIFR;        /*!< RCC Clock Interrupt Flag Register,                                      Address offset: 0x1C */
  __IO uint32_t CICR;        /*!< RCC Clock Interrupt Clear Register,                                     Address offset: 0x20 */
  __IO uint32_t IOPRSTR;     /*!< RCC IO port reset register,                                             Address offset: 0x24 */
  __IO uint32_t AHBRSTR;     /*!< RCC AHB peripherals reset register,                                     Address offset: 0x28 */
  __IO uint32_t APBRSTR1;    /*!< RCC APB peripherals reset register 1,                                   Address offset: 0x2C */
  __IO uint32_t APBRSTR2;    /*!< RCC APB peripherals reset register 2,                                   Address offset: 0x30 */
  __IO uint32_t IOPENR;      /*!< RCC IO port enable register,                                            Address offset: 0x34 */
  __IO uint32_t AHBENR;      /*!< RCC AHB peripherals clock enable register,                              Address offset: 0x38 */
  __IO uint32_t APBENR1;     /*!< RCC APB peripherals clock enable register1,                             Address offset: 0x3C */
  __IO uint32_t APBENR2;     /*!< RCC APB peripherals clock enable register2,                             Address offset: 0x40 */
  __IO uint32_t RESERVED2[4];/*!< Reserved,                                                               Address offset: 0x44-0x50 */
  __IO uint32_t CCIPR;       /*!< RCC Peripherals Independent Clocks Configuration Register,              Address offset: 0x54 */
  __IO uint32_t RESERVED3;   /*!< Reserved,                                                               Address offset: 0x58 */
  __IO uint32_t BDCR;        /*!< RCC Backup Domain Control Register,                                     Address offset: 0x5C */
  __IO uint32_t CSR;         /*!< RCC Unregulated Domain Clock Control and Status Register,               Address offset: 0x60 */
} RCC_TypeDef;

/**
* @brief RTC Registers
*/
typedef struct
{
  __IO uint32_t CRH;         /*!< RTC desc CRH,  Address offset: 0x00 */
  __IO uint32_t CRL;         /*!< RTC desc CRL,  Address offset: 0x04 */
  __IO uint32_t PRLH;        /*!< RTC desc PRLH, Address offset: 0x08 */
  __IO uint32_t PRLL;        /*!< RTC desc PRLL, Address offset: 0x0C */
  __IO uint32_t DIVH;        /*!< RTC desc DIVH, Address offset: 0x10 */
  __IO uint32_t DIVL;        /*!< RTC desc DIVL, Address offset: 0x14 */
  __IO uint32_t CNTH;        /*!< RTC desc CNTH, Address offset: 0x18 */
  __IO uint32_t CNTL;        /*!< RTC desc CNTL, Address offset: 0x1C */
  __IO uint32_t ALRH;        /*!< RTC desc ALRH, Address offset: 0x20 */
  __IO uint32_t ALRL;        /*!< RTC desc ALRL, Address offset: 0x24 */
  uint32_t RESERVED1;
  __IO uint32_t BKP_RTCCR;   /*!< RTC desc BKP_RTCCR, Address offset: 0x2C */
} RTC_TypeDef;


/**
  * @brief Serial Peripheral Interface
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< SPI Control register 1,                              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< SPI Control register 2,                              Address offset: 0x04 */
  __IO uint32_t SR;          /*!< SPI Status register,                                 Address offset: 0x08 */
  __IO uint32_t DR;          /*!< SPI data register,                                   Address offset: 0x0C */
} SPI_TypeDef;

/**
  * @brief System configuration controller
  */
typedef struct
{
  __IO uint32_t CFGR1;         /*!< SYSCFG configuration register 1,                   Address offset: 0x00 */
  __IO uint32_t CFGR2;         /*!< SYSCFG configuration register 2,                   Address offset: 0x04 */
  __IO uint32_t RESERVED1[2];  /*!< Reserved,                                          Address offset: 0x08-0xC */
  __IO uint32_t PAENS;         /*!< Reserved,                                          Address offset: 0x10 */
  __IO uint32_t PBENS;         /*!< GPIO Filter Enable,                                Address offset: 0x14 */
  __IO uint32_t PFENS;         /*!< GPIO Filter Enable,                                Address offset: 0x18 */
  __IO uint32_t IOCFG;         /*!< Reserved,                                          Address offset: 0x1C */
  __IO uint32_t PAANA2EN;      /*!< GPIO Filter Enable,                                Address offset: 0x20 */
  __IO uint32_t PBANA2EN;      /*!< GPIO Filter Enable,                                Address offset: 0x24 */
  __IO uint32_t PFANA2EN;      /*!< GPIO Filter Enable,                                Address offset: 0x28 */
  __IO uint32_t RESERVED2;     /*!< Reserved,                                          Address offset: 0x2C */
  __IO uint32_t PAIORP;        /*!< GPIOA pull up register configuration,              Address offset: 0x30 */
  __IO uint32_t PBIORP;        /*!< GPIOB pull up register configuration,              Address offset: 0x34 */
  __IO uint32_t PFIORP;        /*!< GPIOF pull up register configuration,              Address offset: 0x38 */
} SYSCFG_TypeDef;

/**
  * @brief TIM
  */
typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM interrupt enable register,            Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler register,                   Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  __IO uint32_t RESERVED[2]; /*!< Reserved,                                 Address offset: 0x48 - 0x4F */
  __IO uint32_t OR;          /*!< TIM option register,                      Address offset: 0x50 */
} TIM_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct
{
  __IO uint32_t DR;          /*!< UART DR Register,           Address offset: 0x00  */
  __IO uint32_t BRR;         /*!< UART BRR register,          Address offset: 0x04  */
  __IO uint32_t Reserved[2]; /*!< Reserved,                   Address offset: 0x08 ~ 0x0C */
  __IO uint32_t SR;          /*!< UART SR  register 2,        Address offset: 0x10  */
  __IO uint32_t CR1;         /*!< UART Control register 1,    Address offset: 0x14  */
  __IO uint32_t CR2;         /*!< UART Control register 2,    Address offset: 0x18  */
  __IO uint32_t CR3;         /*!< UART Control register 3,    Address offset: 0x1C  */
  __IO uint32_t RAR;         /*!< UART RAR register,          Address offset: 0x20  */
  __IO uint32_t TAR;         /*!< UART TAR register,          Address offset: 0x24  */
} UART_TypeDef;

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            (0x08000000UL)  /*!< FLASH base address */
#define FLASH_END             (0x08004FFFUL)  /*!< FLASH end address */
#define FLASH_SIZE            (FLASH_END - FLASH_BASE + 1)
#define FLASH_PAGE_SIZE       0x00000080U     /*!< FLASH Page Size, 128 Bytes */
#define FLASH_PAGE_NB         (FLASH_SIZE / FLASH_PAGE_SIZE)
#define FLASH_SECTOR_SIZE     0x00001000U     /*!< FLASH Sector Size, 4096 Bytes */
#define FLASH_SECTOR_NB       (FLASH_SIZE / FLASH_SECTOR_SIZE)
#define SRAM_BASE             (0x20000000UL)  /*!< SRAM base address */
#define SRAM_END              (0x200007FFUL)  /*!< SRAM end address */
#define PERIPH_BASE           (0x40000000UL)  /*!< Peripheral base address */
#define IOPORT_BASE           (0x50000000UL)  /*!< IOPORT base address */
#define FLASH_USERDATA_BASE   (0x1FFF0280UL)  /*!< Base address of : (up to 128 Bytes) embedded FLASH Userdata Area  */
#define FLASH_USERDATA_END    (0x1FFF02FFUL)  /*!< End address of : (up to 128 Bytes) embedded FLASH Userdata Area   */

/*!< Peripheral memory map */
#define APBPERIPH_BASE        (PERIPH_BASE)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)

/*!< APB peripherals */
#define TIM14_BASE            (APBPERIPH_BASE + 0x00002000UL)
#define RTC_BASE              (APBPERIPH_BASE + 0x00002800UL)
#define IWDG_BASE             (APBPERIPH_BASE + 0x00003000UL)
#define TK_BASE               (APBPERIPH_BASE + 0x00003800UL)
#define UART1_BASE            (APBPERIPH_BASE + 0x00004400UL)
#define I2C_BASE              (APBPERIPH_BASE + 0x00005400UL)
#define PWR_BASE              (APBPERIPH_BASE + 0x00007000UL)
#define SYSCFG_BASE           (APBPERIPH_BASE + 0x00010000UL)
#define COMP1_BASE            (APBPERIPH_BASE + 0x00010200UL)
#define COMP2_BASE            (APBPERIPH_BASE + 0x00010210UL)
#define ADC1_BASE             (APBPERIPH_BASE + 0x00012400UL)
#define ADC_BASE              (APBPERIPH_BASE + 0x00012708UL)
#define TIM1_BASE             (APBPERIPH_BASE + 0x00012C00UL)
#define SPI1_BASE             (APBPERIPH_BASE + 0x00013000UL)
#define UART3_BASE            (APBPERIPH_BASE + 0x00013800UL)
#define UART2_BASE            (APBPERIPH_BASE + 0x00014400UL)
#define DBGMCU_BASE           (APBPERIPH_BASE + 0x00015800UL)

/*!< AHB peripherals */
#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000UL)
#define EXTI_BASE             (AHBPERIPH_BASE + 0x00001800UL)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000UL) /*!< FLASH registers base address */
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000UL)
#define OB_BASE               (0x1FFF0080UL)                  /*!< FLASH Option Bytes base address */
#define FLASHSIZE_BASE        (0x1FFF01FCUL)                  /*!< FLASH Size register base address */
#define UID_BASE              (0x1FFF0000UL)                  /*!< Unique device ID register base address */
#define OTP_BASE              (0x1FFF0280UL)

/*!< IOPORT */
#define GPIOA_BASE            (IOPORT_BASE + 0x00000000UL)
#define GPIOB_BASE            (IOPORT_BASE + 0x00000400UL)
#define GPIOF_BASE            (IOPORT_BASE + 0x00001400UL)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define TK                  ((TK_TypeDef *) TK_BASE)
#define UART1               ((UART_TypeDef *) UART1_BASE)
#define I2C1                ((I2C_TypeDef *) I2C_BASE)
#define I2C                 ((I2C_TypeDef *) I2C_BASE)        /* Kept for legacy purpose */
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define COMP1               ((COMP_TypeDef *) COMP1_BASE)
#define COMP2               ((COMP_TypeDef *) COMP2_BASE)
#define COMP12_COMMON       ((COMP_Common_TypeDef *) COMP1_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC_BASE)
#define ADC                 ((ADC_Common_TypeDef *) ADC_BASE) /* Kept for legacy purpose */
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define UART2               ((UART_TypeDef *) UART2_BASE)
#define UART3               ((UART_TypeDef *) UART3_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/** @addtogroup Peripheral_Registers_Bits_Definition
* @{
*/

/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                      Analog to Digital Converter (ADC)                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for ADC_ISR register  ******************/
#define ADC_ISR_EOSMP_Pos         (1U)
#define ADC_ISR_EOSMP_Msk         (0x1UL << ADC_ISR_EOSMP_Pos)                  /*!< 0x00000002 */
#define ADC_ISR_EOSMP              ADC_ISR_EOSMP_Msk                            /*!< ADC group regular end of sampling flag */

#define ADC_ISR_EOC_Pos           (2U)
#define ADC_ISR_EOC_Msk           (0x1UL << ADC_ISR_EOC_Pos)                    /*!< 0x00000004 */
#define ADC_ISR_EOC                ADC_ISR_EOC_Msk                              /*!< ADC group regular end of unitary conversion flag */

#define ADC_ISR_EOSEQ_Pos         (3U)
#define ADC_ISR_EOSEQ_Msk         (0x1UL << ADC_ISR_EOSEQ_Pos)                  /*!< 0x00000008 */
#define ADC_ISR_EOSEQ              ADC_ISR_EOSEQ_Msk                            /*!< ADC group regular end of sequence conversions flag */

#define ADC_ISR_OVR_Pos           (4U)
#define ADC_ISR_OVR_Msk           (0x1UL << ADC_ISR_OVR_Pos)                    /*!< 0x00000010 */
#define ADC_ISR_OVR                ADC_ISR_OVR_Msk                              /*!< ADC group regular overrun flag */

#define ADC_ISR_AWD_Pos           (7U)
#define ADC_ISR_AWD_Msk           (0x1UL << ADC_ISR_AWD_Pos)                    /*!< 0x00000080 */
#define ADC_ISR_AWD                ADC_ISR_AWD_Msk                              /*!< ADC analog watchdog 1 flag */

/********************  Bits definition for ADC_IER register  ******************/
#define ADC_IER_EOSMPIE_Pos       (1U)
#define ADC_IER_EOSMPIE_Msk       (0x1UL << ADC_IER_EOSMPIE_Pos)                /*!< 0x00000002 */
#define ADC_IER_EOSMPIE            ADC_IER_EOSMPIE_Msk                          /*!< ADC group regular end of sampling interrupt */

#define ADC_IER_EOCIE_Pos         (2U)
#define ADC_IER_EOCIE_Msk         (0x1UL << ADC_IER_EOCIE_Pos)                  /*!< 0x00000004 */
#define ADC_IER_EOCIE              ADC_IER_EOCIE_Msk                            /*!< ADC group regular end of unitary conversion interrupt */

#define ADC_IER_EOSEQIE_Pos       (3U)
#define ADC_IER_EOSEQIE_Msk       (0x1UL << ADC_IER_EOSEQIE_Pos)                /*!< 0x00000008 */
#define ADC_IER_EOSEQIE            ADC_IER_EOSEQIE_Msk                          /*!< ADC group regular end of sequence conversions interrupt */

#define ADC_IER_OVRIE_Pos         (4U)
#define ADC_IER_OVRIE_Msk         (0x1UL << ADC_IER_OVRIE_Pos)                  /*!< 0x00000010 */
#define ADC_IER_OVRIE              ADC_IER_OVRIE_Msk                            /*!< ADC group regular overrun interrupt */

#define ADC_IER_AWDIE_Pos         (7U)
#define ADC_IER_AWDIE_Msk         (0x1UL << ADC_IER_AWDIE_Pos)                  /*!< 0x00000080 */
#define ADC_IER_AWDIE              ADC_IER_AWDIE_Msk                            /*!< ADC analog watchdog 1 interrupt */

/********************  Bits definition for ADC_CR register  *******************/
#define ADC_CR_ADEN_Pos           (0U)
#define ADC_CR_ADEN_Msk           (0x1UL << ADC_CR_ADEN_Pos)                    /*!< 0x00000001 */
#define ADC_CR_ADEN                ADC_CR_ADEN_Msk                              /*!< ADC enable */

#define ADC_CR_ADDIS_Pos          (1U)
#define ADC_CR_ADDIS_Msk          (0x1UL << ADC_CR_ADDIS_Pos)                   /*!< 0x00000002 */
#define ADC_CR_ADDIS              ADC_CR_ADDIS_Msk                              /*!< ADC disable */

#define ADC_CR_ADSTART_Pos        (2U)
#define ADC_CR_ADSTART_Msk        (0x1UL << ADC_CR_ADSTART_Pos)                 /*!< 0x00000004 */
#define ADC_CR_ADSTART             ADC_CR_ADSTART_Msk                           /*!< ADC group regular conversion start */

#define ADC_CR_ADSTP_Pos          (4U)
#define ADC_CR_ADSTP_Msk          (0x1UL << ADC_CR_ADSTP_Pos)                   /*!< 0x00000010 */
#define ADC_CR_ADSTP               ADC_CR_ADSTP_Msk                             /*!< ADC group regular conversion stop */

#define ADC_CR_ADCAL_Pos          (31U)
#define ADC_CR_ADCAL_Msk          (0x1UL << ADC_CR_ADCAL_Pos)                   /*!< 0x80000000 */
#define ADC_CR_ADCAL               ADC_CR_ADCAL_Msk                             /*!< ADC calibration */

/*******************  Bits definition for ADC_CFGR1 register  *****************/
#define ADC_CFGR1_SCANDIR_Pos     (2U)
#define ADC_CFGR1_SCANDIR_Msk     (0x1UL << ADC_CFGR1_SCANDIR_Pos)              /*!< 0x00000004 */
#define ADC_CFGR1_SCANDIR          ADC_CFGR1_SCANDIR_Msk                        /*!< ADC group regular sequencer scan direction */

#define ADC_CFGR1_RESSEL_Pos      (3U)
#define ADC_CFGR1_RESSEL_Msk      (0x3UL << ADC_CFGR1_RESSEL_Pos)               /*!< 0x00000018 */
#define ADC_CFGR1_RESSEL           ADC_CFGR1_RESSEL_Msk                         /*!< ADC data resolution */
#define ADC_CFGR1_RESSEL_0        (0x1UL << ADC_CFGR1_RESSEL_Pos)               /*!< 0x00000008 */
#define ADC_CFGR1_RESSEL_1        (0x2UL << ADC_CFGR1_RESSEL_Pos)               /*!< 0x00000010 */

#define ADC_CFGR1_ALIGN_Pos       (5U)
#define ADC_CFGR1_ALIGN_Msk       (0x1UL << ADC_CFGR1_ALIGN_Pos)                /*!< 0x00000020 */
#define ADC_CFGR1_ALIGN            ADC_CFGR1_ALIGN_Msk                          /*!< ADC data alignement */

#define ADC_CFGR1_EXTSEL_Pos      (6U)
#define ADC_CFGR1_EXTSEL_Msk      (0x7UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x000001C0 */
#define ADC_CFGR1_EXTSEL           ADC_CFGR1_EXTSEL_Msk                         /*!< ADC group regular external trigger source */
#define ADC_CFGR1_EXTSEL_0        (0x1UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000040 */
#define ADC_CFGR1_EXTSEL_1        (0x2UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000080 */
#define ADC_CFGR1_EXTSEL_2        (0x4UL << ADC_CFGR1_EXTSEL_Pos)               /*!< 0x00000100 */

#define ADC_CFGR1_EXTEN_Pos       (10U)
#define ADC_CFGR1_EXTEN_Msk       (0x3UL << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000C00 */
#define ADC_CFGR1_EXTEN            ADC_CFGR1_EXTEN_Msk                          /*!< ADC group regular external trigger polarity */
#define ADC_CFGR1_EXTEN_0         (0x1UL << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000400 */
#define ADC_CFGR1_EXTEN_1         (0x2UL << ADC_CFGR1_EXTEN_Pos)                /*!< 0x00000800 */

#define ADC_CFGR1_OVRMOD_Pos      (12U)
#define ADC_CFGR1_OVRMOD_Msk      (0x1UL << ADC_CFGR1_OVRMOD_Pos)               /*!< 0x00001000 */
#define ADC_CFGR1_OVRMOD           ADC_CFGR1_OVRMOD_Msk                         /*!< ADC group regular overrun configuration */

#define ADC_CFGR1_CONT_Pos        (13U)
#define ADC_CFGR1_CONT_Msk        (0x1UL << ADC_CFGR1_CONT_Pos)                 /*!< 0x00002000 */
#define ADC_CFGR1_CONT             ADC_CFGR1_CONT_Msk                           /*!< ADC group regular continuous conversion mode */

#define ADC_CFGR1_WAIT_Pos        (14U)
#define ADC_CFGR1_WAIT_Msk        (0x1UL << ADC_CFGR1_WAIT_Pos)                 /*!< 0x00004000 */
#define ADC_CFGR1_WAIT            ADC_CFGR1_WAIT_Msk                           /*!< ADC low power auto wait */

#define ADC_CFGR1_DISCEN_Pos      (16U)
#define ADC_CFGR1_DISCEN_Msk      (0x1UL << ADC_CFGR1_DISCEN_Pos)               /*!< 0x00010000 */
#define ADC_CFGR1_DISCEN           ADC_CFGR1_DISCEN_Msk                         /*!< ADC group regular sequencer discontinuous mode */

#define ADC_CFGR1_AWDSGL_Pos      (22U)
#define ADC_CFGR1_AWDSGL_Msk      (0x1UL << ADC_CFGR1_AWDSGL_Pos)               /*!< 0x00400000 */
#define ADC_CFGR1_AWDSGL          ADC_CFGR1_AWDSGL_Msk                         /*!< ADC analog watchdog 1 monitoring a single channel or all channels */

#define ADC_CFGR1_AWDEN_Pos       (23U)
#define ADC_CFGR1_AWDEN_Msk       (0x1UL << ADC_CFGR1_AWDEN_Pos)                /*!< 0x00800000 */
#define ADC_CFGR1_AWDEN            ADC_CFGR1_AWDEN_Msk                          /*!< ADC analog watchdog 1 enable on scope ADC group regular */

#define ADC_CFGR1_AWDCH_Pos       (26U)
#define ADC_CFGR1_AWDCH_Msk       (0xFUL << ADC_CFGR1_AWDCH_Pos)                /*!< 0x3C000000 */
#define ADC_CFGR1_AWDCH           ADC_CFGR1_AWDCH_Msk                          /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CFGR1_AWDCH_0         (0x01UL << ADC_CFGR1_AWDCH_Pos)               /*!< 0x04000000 */
#define ADC_CFGR1_AWDCH_1         (0x02UL << ADC_CFGR1_AWDCH_Pos)               /*!< 0x08000000 */
#define ADC_CFGR1_AWDCH_2         (0x04UL << ADC_CFGR1_AWDCH_Pos)               /*!< 0x10000000 */
#define ADC_CFGR1_AWDCH_3         (0x08UL << ADC_CFGR1_AWDCH_Pos)               /*!< 0x20000000 */

/******************  Bit definition for ADC_SMPR register  ********************/
#define ADC_SMPR_SMP_Pos          (0U)
#define ADC_SMPR_SMP_Msk          (0x7UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000007 */
#define ADC_SMPR_SMP               ADC_SMPR_SMP_Msk                             /*!< ADC group of channels sampling time 2 */
#define ADC_SMPR_SMP_0            (0x1UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000001 */
#define ADC_SMPR_SMP_1            (0x2UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000002 */
#define ADC_SMPR_SMP_2            (0x4UL << ADC_SMPR_SMP_Pos)                   /*!< 0x00000004 */

/*******************  Bit definition for ADC_TR register  ********************/
#define ADC_TR_LT_Pos           (0U)
#define ADC_TR_LT_Msk           (0xFFFUL << ADC_TR_LT_Pos)                  /*!< 0x00000FFF */
#define ADC_TR_LT               ADC_TR_LT_Msk                               /*!< ADC analog watchdog 1 threshold low */
#define ADC_TR_LT_0             (0x001UL << ADC_TR_LT_Pos)                  /*!< 0x00000001 */
#define ADC_TR_LT_1             (0x002UL << ADC_TR_LT_Pos)                  /*!< 0x00000002 */
#define ADC_TR_LT_2             (0x004UL << ADC_TR_LT_Pos)                  /*!< 0x00000004 */
#define ADC_TR_LT_3             (0x008UL << ADC_TR_LT_Pos)                  /*!< 0x00000008 */
#define ADC_TR_LT_4             (0x010UL << ADC_TR_LT_Pos)                  /*!< 0x00000010 */
#define ADC_TR_LT_5             (0x020UL << ADC_TR_LT_Pos)                  /*!< 0x00000020 */
#define ADC_TR_LT_6             (0x040UL << ADC_TR_LT_Pos)                  /*!< 0x00000040 */
#define ADC_TR_LT_7             (0x080UL << ADC_TR_LT_Pos)                  /*!< 0x00000080 */
#define ADC_TR_LT_8             (0x100UL << ADC_TR_LT_Pos)                  /*!< 0x00000100 */
#define ADC_TR_LT_9             (0x200UL << ADC_TR_LT_Pos)                  /*!< 0x00000200 */
#define ADC_TR_LT_10            (0x400UL << ADC_TR_LT_Pos)                  /*!< 0x00000400 */
#define ADC_TR_LT_11            (0x800UL << ADC_TR_LT_Pos)                  /*!< 0x00000800 */

#define ADC_TR_HT_Pos           (16U)
#define ADC_TR_HT_Msk           (0xFFFUL << ADC_TR_HT_Pos)                  /*!< 0x0FFF0000 */
#define ADC_TR_HT               ADC_TR_HT_Msk                               /*!< ADC Analog watchdog 1 threshold high */
#define ADC_TR_HT_0             (0x001UL << ADC_TR_HT_Pos)                  /*!< 0x00010000 */
#define ADC_TR_HT_1             (0x002UL << ADC_TR_HT_Pos)                  /*!< 0x00020000 */
#define ADC_TR_HT_2             (0x004UL << ADC_TR_HT_Pos)                  /*!< 0x00040000 */
#define ADC_TR_HT_3             (0x008UL << ADC_TR_HT_Pos)                  /*!< 0x00080000 */
#define ADC_TR_HT_4             (0x010UL << ADC_TR_HT_Pos)                  /*!< 0x00100000 */
#define ADC_TR_HT_5             (0x020UL << ADC_TR_HT_Pos)                  /*!< 0x00200000 */
#define ADC_TR_HT_6             (0x040UL << ADC_TR_HT_Pos)                  /*!< 0x00400000 */
#define ADC_TR_HT_7             (0x080UL << ADC_TR_HT_Pos)                  /*!< 0x00800000 */
#define ADC_TR_HT_8             (0x100UL << ADC_TR_HT_Pos)                  /*!< 0x01000000 */
#define ADC_TR_HT_9             (0x200UL << ADC_TR_HT_Pos)                  /*!< 0x02000000 */
#define ADC_TR_HT_10            (0x400UL << ADC_TR_HT_Pos)                  /*!< 0x04000000 */
#define ADC_TR_HT_11            (0x800UL << ADC_TR_HT_Pos)                  /*!< 0x08000000 */

/******************  Bit definition for ADC_CHSELR register  ******************/
#define ADC_CHSELR_CHSEL_Pos      (0U)
#define ADC_CHSELR_CHSEL_Msk      (0x1FFFUL << ADC_CHSELR_CHSEL_Pos)             /*!< 0x000007FF */
#define ADC_CHSELR_CHSEL           ADC_CHSELR_CHSEL_Msk                         /*!< ADC group regular sequencer channels, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL12_Pos    (12U)
#define ADC_CHSELR_CHSEL12_Msk    (0x1UL << ADC_CHSELR_CHSEL12_Pos)             /*!< 0x00001000 */
#define ADC_CHSELR_CHSEL12         ADC_CHSELR_CHSEL12_Msk                       /*!< ADC group regular sequencer channel 12, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL11_Pos    (11U)
#define ADC_CHSELR_CHSEL11_Msk    (0x1UL << ADC_CHSELR_CHSEL11_Pos)             /*!< 0x00000800 */
#define ADC_CHSELR_CHSEL11         ADC_CHSELR_CHSEL11_Msk                       /*!< ADC group regular sequencer channel 11, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL10_Pos    (10U)
#define ADC_CHSELR_CHSEL10_Msk    (0x1UL << ADC_CHSELR_CHSEL10_Pos)             /*!< 0x00000400 */
#define ADC_CHSELR_CHSEL10         ADC_CHSELR_CHSEL10_Msk                       /*!< ADC group regular sequencer channel 10, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL9_Pos     (9U)
#define ADC_CHSELR_CHSEL9_Msk     (0x1UL << ADC_CHSELR_CHSEL9_Pos)              /*!< 0x00000200 */
#define ADC_CHSELR_CHSEL9          ADC_CHSELR_CHSEL9_Msk                        /*!< ADC group regular sequencer channel 9, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL8_Pos     (8U)
#define ADC_CHSELR_CHSEL8_Msk     (0x1UL << ADC_CHSELR_CHSEL8_Pos)              /*!< 0x00000100 */
#define ADC_CHSELR_CHSEL8          ADC_CHSELR_CHSEL8_Msk                        /*!< ADC group regular sequencer channel 8, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL7_Pos     (7U)
#define ADC_CHSELR_CHSEL7_Msk     (0x1UL << ADC_CHSELR_CHSEL7_Pos)              /*!< 0x00000080 */
#define ADC_CHSELR_CHSEL7          ADC_CHSELR_CHSEL7_Msk                        /*!< ADC group regular sequencer channel 7, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL6_Pos     (6U)
#define ADC_CHSELR_CHSEL6_Msk     (0x1UL << ADC_CHSELR_CHSEL6_Pos)              /*!< 0x00000040 */
#define ADC_CHSELR_CHSEL6          ADC_CHSELR_CHSEL6_Msk                        /*!< ADC group regular sequencer channel 6, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL5_Pos     (5U)
#define ADC_CHSELR_CHSEL5_Msk     (0x1UL << ADC_CHSELR_CHSEL5_Pos)              /*!< 0x00000020 */
#define ADC_CHSELR_CHSEL5          ADC_CHSELR_CHSEL5_Msk                        /*!< ADC group regular sequencer channel 5, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL4_Pos     (4U)
#define ADC_CHSELR_CHSEL4_Msk     (0x1UL << ADC_CHSELR_CHSEL4_Pos)              /*!< 0x00000010 */
#define ADC_CHSELR_CHSEL4          ADC_CHSELR_CHSEL4_Msk                        /*!< ADC group regular sequencer channel 4, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL3_Pos     (3U)
#define ADC_CHSELR_CHSEL3_Msk     (0x1UL << ADC_CHSELR_CHSEL3_Pos)              /*!< 0x00000008 */
#define ADC_CHSELR_CHSEL3          ADC_CHSELR_CHSEL3_Msk                        /*!< ADC group regular sequencer channel 3, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL2_Pos     (2U)
#define ADC_CHSELR_CHSEL2_Msk     (0x1UL << ADC_CHSELR_CHSEL2_Pos)              /*!< 0x00000004 */
#define ADC_CHSELR_CHSEL2          ADC_CHSELR_CHSEL2_Msk                        /*!< ADC group regular sequencer channel 2, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL1_Pos     (1U)
#define ADC_CHSELR_CHSEL1_Msk     (0x1UL << ADC_CHSELR_CHSEL1_Pos)              /*!< 0x00000002 */
#define ADC_CHSELR_CHSEL1          ADC_CHSELR_CHSEL1_Msk                        /*!< ADC group regular sequencer channel 1, available when ADC_CFGR1_CHSELRMOD is reset */

#define ADC_CHSELR_CHSEL0_Pos     (0U)
#define ADC_CHSELR_CHSEL0_Msk     (0x1UL << ADC_CHSELR_CHSEL0_Pos)              /*!< 0x00000001 */
#define ADC_CHSELR_CHSEL0          ADC_CHSELR_CHSEL0_Msk                        /*!< ADC group regular sequencer channel 0, available when ADC_CFGR1_CHSELRMOD is reset */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos           (0U)
#define ADC_DR_DATA_Msk           (0xFFFFUL << ADC_DR_DATA_Pos)                 /*!< 0x0000FFFF */
#define ADC_DR_DATA                ADC_DR_DATA_Msk                              /*!< ADC group regular conversion data */
#define ADC_DR_DATA_0             (0x0001UL << ADC_DR_DATA_Pos)                 /*!< 0x00000001 */
#define ADC_DR_DATA_1             (0x0002UL << ADC_DR_DATA_Pos)                 /*!< 0x00000002 */
#define ADC_DR_DATA_2             (0x0004UL << ADC_DR_DATA_Pos)                 /*!< 0x00000004 */
#define ADC_DR_DATA_3             (0x0008UL << ADC_DR_DATA_Pos)                 /*!< 0x00000008 */
#define ADC_DR_DATA_4             (0x0010UL << ADC_DR_DATA_Pos)                 /*!< 0x00000010 */
#define ADC_DR_DATA_5             (0x0020UL << ADC_DR_DATA_Pos)                 /*!< 0x00000020 */
#define ADC_DR_DATA_6             (0x0040UL << ADC_DR_DATA_Pos)                 /*!< 0x00000040 */
#define ADC_DR_DATA_7             (0x0080UL << ADC_DR_DATA_Pos)                 /*!< 0x00000080 */
#define ADC_DR_DATA_8             (0x0100UL << ADC_DR_DATA_Pos)                 /*!< 0x00000100 */
#define ADC_DR_DATA_9             (0x0200UL << ADC_DR_DATA_Pos)                 /*!< 0x00000200 */
#define ADC_DR_DATA_10            (0x0400UL << ADC_DR_DATA_Pos)                 /*!< 0x00000400 */
#define ADC_DR_DATA_11            (0x0800UL << ADC_DR_DATA_Pos)                 /*!< 0x00000800 */
#define ADC_DR_DATA_12            (0x1000UL << ADC_DR_DATA_Pos)                 /*!< 0x00001000 */
#define ADC_DR_DATA_13            (0x2000UL << ADC_DR_DATA_Pos)                 /*!< 0x00002000 */
#define ADC_DR_DATA_14            (0x4000UL << ADC_DR_DATA_Pos)                 /*!< 0x00004000 */
#define ADC_DR_DATA_15            (0x8000UL << ADC_DR_DATA_Pos)                 /*!< 0x00008000 */

/********************  Bit definition for ADC_CCSR register  ********************/
#define ADC_CCSR_CALSEL_Pos       (11U)
#define ADC_CCSR_CALSEL_Msk       (0x1UL << ADC_CCSR_CALSEL_Pos)                /*!< 0x00000800 */
#define ADC_CCSR_CALSEL           ADC_CCSR_CALSEL_Msk                           /*!< ADC calibration context selection */

#define ADC_CCSR_CALSMP_Pos       (12U)
#define ADC_CCSR_CALSMP_Msk       (0x3UL << ADC_CCSR_CALSMP_Pos)                /*!< 0x00003000 */
#define ADC_CCSR_CALSMP           ADC_CCSR_CALSMP_Msk                           /*!< ADC calibration sample time selection */
#define ADC_CCSR_CALSMP_0         (0x1UL << ADC_CCSR_CALSMP_Pos)                /*!< 0x00001000 */
#define ADC_CCSR_CALSMP_1         (0x2UL << ADC_CCSR_CALSMP_Pos)                /*!< 0x00002000 */

#define ADC_CCSR_CALSET_Pos       (15U)
#define ADC_CCSR_CALSET_Msk       (0x1UL << ADC_CCSR_CALSET_Pos)                /*!< 0x00008000 */
#define ADC_CCSR_CALSET           ADC_CCSR_CALSET_Msk                           /*!< ADC Calibration factor selection */

#define ADC_CCSR_OFFSUC_Pos       (29U)
#define ADC_CCSR_OFFSUC_Msk       (0x1UL << ADC_CCSR_OFFSUC_Pos)                /*!< 0x20000000 */
#define ADC_CCSR_OFFSUC           ADC_CCSR_OFFSUC_Msk                           /*!< Offset Indicates the calibration status bit */

#define ADC_CCSR_CAPSUC_Pos       (30U)
#define ADC_CCSR_CAPSUC_Msk       (0x1UL << ADC_CCSR_CAPSUC_Pos)                /*!< 0x40000000 */
#define ADC_CCSR_CAPSUC           ADC_CCSR_CAPSUC_Msk                           /*!< ADC capacitance calibration flag bit */

#define ADC_CCSR_CALON_Pos        (31U)
#define ADC_CCSR_CALON_Msk        (0x1UL << ADC_CCSR_CALON_Pos)                 /*!< 0x80000000 */
#define ADC_CCSR_CALON            ADC_CCSR_CALON_Msk                            /*!< ADC calibration flag */

/*************************  ADC Common registers  *****************************/
/*******************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_VREFBUF_EN_Pos    (5U)
#define ADC_CCR_VREFBUF_EN_Msk    (0x1UL << ADC_CCR_VREFBUF_EN_Pos)             /*!< 0x00000020 */
#define ADC_CCR_VREFBUF_EN        ADC_CCR_VREFBUF_EN_Msk                        /*!< ADC vrefbuffer enable */

#define ADC_CCR_VREFBUF_SEL_Pos   (6U)
#define ADC_CCR_VREFBUF_SEL_Msk   (0x3UL << ADC_CCR_VREFBUF_SEL_Pos)            /*!< 0x000000C0 */
#define ADC_CCR_VREFBUF_SEL       ADC_CCR_VREFBUF_SEL_Msk                       /*!< ADC vrefbuff output voltage select */
#define ADC_CCR_VREFBUF_SEL_0     (0x1UL << ADC_CCR_VREFBUF_SEL_Pos)            /*!< 0x00000040 */
#define ADC_CCR_VREFBUF_SEL_1     (0x2UL << ADC_CCR_VREFBUF_SEL_Pos)            /*!< 0x00000080 */

#define ADC_CCR_VREFEN_Pos        (22U)
#define ADC_CCR_VREFEN_Msk        (0x1UL << ADC_CCR_VREFEN_Pos)                 /*!< 0x00400000 */
#define ADC_CCR_VREFEN            ADC_CCR_VREFEN_Msk                            /*!< ADC internal path to VrefInt enable */

#define ADC_CCR_TSEN_Pos          (23U)
#define ADC_CCR_TSEN_Msk          (0x1UL << ADC_CCR_TSEN_Pos)                   /*!< 0x00800000 */
#define ADC_CCR_TSEN              ADC_CCR_TSEN_Msk                              /*!< ADC internal path to temperature sensor enable */

/****************************************************************************/
/*                                                                          */
/*                      Analog Comparators (COMP)                           */
/*                                                                          */
/****************************************************************************/
/**********************  Bit definition for COMP_CSR register  **************/
#define COMP_CSR_EN_Pos            (0U)
#define COMP_CSR_EN_Msk            (0x1UL << COMP_CSR_EN_Pos)                  /*!< 0x00000001 */
#define COMP_CSR_EN                COMP_CSR_EN_Msk                             /*!< Comparator enable */
#define COMP_CSR_COMP1_EN          COMP_CSR_EN
#define COMP_CSR_COMP2_EN          COMP_CSR_EN

#define COMP_CSR_INMSEL_Pos        (6U)
#define COMP_CSR_INMSEL_Msk        (0x3UL << COMP_CSR_INMSEL_Pos)              /*!< 0x000000C0 */
#define COMP_CSR_INMSEL            COMP_CSR_INMSEL_Msk                         /*!< desc INMSEL */
#define COMP_CSR_INMSEL_0          (0x1UL << COMP_CSR_INMSEL_Pos)              /*!< 0x00000040 */
#define COMP_CSR_INMSEL_1          (0x2UL << COMP_CSR_INMSEL_Pos)              /*!< 0x00000080 */

#define COMP_CSR_INPSEL_Pos        (8U)
#define COMP_CSR_INPSEL_Msk        (0x3UL << COMP_CSR_INPSEL_Pos)              /*!< 0x00000300 */
#define COMP_CSR_INPSEL            COMP_CSR_INPSEL_Msk                         /*!< desc INPSEL */
#define COMP_CSR_INPSEL_0          (0x1UL << COMP_CSR_INPSEL_Pos)              /*!< 0x00000010 */
#define COMP_CSR_INPSEL_1          (0x2UL << COMP_CSR_INPSEL_Pos)              /*!< 0x00000020 */

#define COMP_CSR_WINMODE_Pos       (11U)
#define COMP_CSR_WINMODE_Msk       (0x1UL << COMP_CSR_WINMODE_Pos)             /*!< 0x00000800 */
#define COMP_CSR_WINMODE           COMP_CSR_WINMODE_Msk                        /*!< Pair of comparators window mode. Bit intended to be used with COMP common instance (COMP_Common_TypeDef) */

#define COMP_CSR_POLARITY_Pos      (15U)
#define COMP_CSR_POLARITY_Msk      (0x1UL << COMP_CSR_POLARITY_Pos)            /*!< 0x00008000 */
#define COMP_CSR_POLARITY          COMP_CSR_POLARITY_Msk                       /*!< Comparator output polarity */

#define COMP_CSR_PWRMODE_Pos       (18U)
#define COMP_CSR_PWRMODE_Msk       (0x1UL << COMP_CSR_PWRMODE_Pos)             /*!< 0x00008000 */
#define COMP_CSR_PWRMODE           COMP_CSR_PWRMODE_Msk                        /*!< Comparator output polarity */

#define COMP_CSR_COMP_VCDIV_Pos     (20U)
#define COMP_CSR_COMP_VCDIV_Msk     (0x3FUL << COMP_CSR_COMP_VCDIV_Pos)          /*!< 0x03F00000 */
#define COMP_CSR_COMP_VCDIV         COMP_CSR_COMP_VCDIV_Msk                      /*!< Comparator voltage divider */
#define COMP_CSR_COMP_VCDIV_0       (0x1UL << COMP_CSR_COMP_VCDIV_Pos)           /*!< 0x00100000 */
#define COMP_CSR_COMP_VCDIV_1       (0x2UL << COMP_CSR_COMP_VCDIV_Pos)           /*!< 0x00200000 */
#define COMP_CSR_COMP_VCDIV_2       (0x4UL << COMP_CSR_COMP_VCDIV_Pos)           /*!< 0x00400000 */
#define COMP_CSR_COMP_VCDIV_3       (0x8UL << COMP_CSR_COMP_VCDIV_Pos)           /*!< 0x00800000 */
#define COMP_CSR_COMP_VCDIV_4       (0x10UL << COMP_CSR_COMP_VCDIV_Pos)          /*!< 0x01000000 */
#define COMP_CSR_COMP_VCDIV_5       (0x20UL << COMP_CSR_COMP_VCDIV_Pos)          /*!< 0x02000000 */

#define COMP_CSR_COMP_VCDIV_EN_Pos   (26U)
#define COMP_CSR_COMP_VCDIV_EN_Msk   (0x1UL << COMP_CSR_COMP_VCDIV_EN_Pos)         /*!< 0x04000000 */
#define COMP_CSR_COMP_VCDIV_EN       COMP_CSR_COMP_VCDIV_EN_Msk                    /*!< desc COMPVCDIVEN */

#define COMP_CSR_COMP_VCSEL_Pos     (27U)
#define COMP_CSR_COMP_VCSEL_Msk     (0x1UL << COMP_CSR_COMP_VCSEL_Pos)           /*!< 0x08000000 */
#define COMP_CSR_COMP_VCSEL         COMP_CSR_COMP_VCSEL_Msk                      /*!< desc COMPVCSEL */

#define COMP_CSR_TIM_EXTI_OUT_SEL_Pos     (29U)
#define COMP_CSR_TIM_EXTI_OUT_SEL_Msk     (0x1UL << COMP_CSR_TIM_EXTI_OUT_SEL_Pos)       /*!< 0x20000000 */
#define COMP_CSR_TIM_EXTI_OUT_SEL         COMP_CSR_TIM_EXTI_OUT_SEL_Msk                  /*!< desc TIM_EXTI_OUT_SEL */

#define COMP_CSR_COMP_OUT_Pos       (30U)
#define COMP_CSR_COMP_OUT_Msk       (0x1UL << COMP_CSR_COMP_OUT_Pos)             /*!< 0x40000000 */
#define COMP_CSR_COMP_OUT           COMP_CSR_COMP_OUT_Msk                        /*!< desc COMP_OUT */

/**********************  Bit definition for COMP_FR register  ***************/
#define COMP_FR_FLTEN_Pos          (0U)
#define COMP_FR_FLTEN_Msk          (0x1UL << COMP_FR_FLTEN_Pos)                /*!< 0x00000001 */
#define COMP_FR_FLTEN              COMP_FR_FLTEN_Msk                           /*!< Comparator filter enable */

#define COMP_FR_FLTCNT_Pos         (16U)
#define COMP_FR_FLTCNT_Msk         (0xFFFFUL << COMP_FR_FLTCNT_Pos)            /*!< 0xFFFF0000 */
#define COMP_FR_FLTCNT             COMP_FR_FLTCNT_Msk                          /*!< Comparator filter counter */


/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit (CRC)                        */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define CRC_DR_DR_Pos            (0U)
#define CRC_DR_DR_Msk            (0xFFFFFFFFUL << CRC_DR_DR_Pos)                /*!< 0xFFFFFFFF */
#define CRC_DR_DR                CRC_DR_DR_Msk                                  /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define CRC_IDR_IDR_Pos          (0U)
#define CRC_IDR_IDR_Msk          (0xFFUL << CRC_IDR_IDR_Pos)                    /*!< 0x000000FF */
#define CRC_IDR_IDR              CRC_IDR_IDR_Msk                                /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define CRC_CR_RESET_Pos         (0U)
#define CRC_CR_RESET_Msk         (0x1UL << CRC_CR_RESET_Pos)                    /*!< 0x00000001 */
#define CRC_CR_RESET             CRC_CR_RESET_Msk                               /*!< RESET the CRC computation unit bit */

/******************************************************************************/
/*                                                                            */
/*                                Debug MCU (DBGMCU)                          */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBG_IDCODE register  *************/
#define DBGMCU_IDCODE_DEV_ID_Pos                          (0U)
#define DBGMCU_IDCODE_DEV_ID_Msk                          (0xFFFFFFFFUL << DBGMCU_IDCODE_DEV_ID_Pos)     /*!< 0xFFFFFFFF */
#define DBGMCU_IDCODE_DEV_ID                              DBGMCU_IDCODE_DEV_ID_Msk

/********************  Bit definition for DBGMCU_CR register  *****************/
#define DBGMCU_CR_DBG_SLEEP_Pos                           (0U)
#define DBGMCU_CR_DBG_SLEEP_Msk                           (0x1UL << DBGMCU_CR_DBG_SLEEP_Pos)       /*!< 0x00000001 */
#define DBGMCU_CR_DBG_SLEEP                               DBGMCU_CR_DBG_SLEEP_Msk

#define DBGMCU_CR_DBG_STOP_Pos                            (1U)
#define DBGMCU_CR_DBG_STOP_Msk                            (0x1UL << DBGMCU_CR_DBG_STOP_Pos)       /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP                                DBGMCU_CR_DBG_STOP_Msk

/********************  Bit definition for DBGMCU_APB_FZ1 register  ***********/
#define DBGMCU_APB_FZ1_DBG_RTC_STOP_Pos                   (10U)
#define DBGMCU_APB_FZ1_DBG_RTC_STOP_Msk                   (0x1UL << DBGMCU_APB_FZ1_DBG_RTC_STOP_Pos) /*!< 0x00000400 */
#define DBGMCU_APB_FZ1_DBG_RTC_STOP                       DBGMCU_APB_FZ1_DBG_RTC_STOP_Msk

#define DBGMCU_APB_FZ1_DBG_IWDG_STOP_Pos                  (12U)
#define DBGMCU_APB_FZ1_DBG_IWDG_STOP_Msk                  (0x1UL << DBGMCU_APB_FZ1_DBG_IWDG_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_APB_FZ1_DBG_IWDG_STOP                      DBGMCU_APB_FZ1_DBG_IWDG_STOP_Msk

/********************  Bit definition for DBGMCU_APB_FZ2 register  ************/
#define DBGMCU_APB_FZ2_DBG_TIM1_STOP_Pos                  (11U)
#define DBGMCU_APB_FZ2_DBG_TIM1_STOP_Msk                  (0x1UL << DBGMCU_APB_FZ2_DBG_TIM1_STOP_Pos)  /*!< 0x00000800 */
#define DBGMCU_APB_FZ2_DBG_TIM1_STOP                      DBGMCU_APB_FZ2_DBG_TIM1_STOP_Msk

#define DBGMCU_APB_FZ2_DBG_TIM14_STOP_Pos                 (15U)
#define DBGMCU_APB_FZ2_DBG_TIM14_STOP_Msk                 (0x1UL << DBGMCU_APB_FZ2_DBG_TIM14_STOP_Pos)  /*!< 0x00008000 */
#define DBGMCU_APB_FZ2_DBG_TIM14_STOP                     DBGMCU_APB_FZ2_DBG_TIM14_STOP_Msk

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller (EXTI)              */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for EXTI_RTSR register  ******************/
#define EXTI_RTSR_RT0_Pos           (0U)
#define EXTI_RTSR_RT0_Msk           (0x1UL << EXTI_RTSR_RT0_Pos)             /*!< 0x00000001 */
#define EXTI_RTSR_RT0               EXTI_RTSR_RT0_Msk                        /*!< Rising trigger configuration for input line 0 */

#define EXTI_RTSR_RT1_Pos           (1U)
#define EXTI_RTSR_RT1_Msk           (0x1UL << EXTI_RTSR_RT1_Pos)             /*!< 0x00000002 */
#define EXTI_RTSR_RT1               EXTI_RTSR_RT1_Msk                        /*!< Rising trigger configuration for input line 1 */

#define EXTI_RTSR_RT2_Pos           (2U)
#define EXTI_RTSR_RT2_Msk           (0x1UL << EXTI_RTSR_RT2_Pos)             /*!< 0x00000004 */
#define EXTI_RTSR_RT2               EXTI_RTSR_RT2_Msk                        /*!< Rising trigger configuration for input line 2 */

#define EXTI_RTSR_RT3_Pos           (3U)
#define EXTI_RTSR_RT3_Msk           (0x1UL << EXTI_RTSR_RT3_Pos)             /*!< 0x00000008 */
#define EXTI_RTSR_RT3               EXTI_RTSR_RT3_Msk                        /*!< Rising trigger configuration for input line 3 */

#define EXTI_RTSR_RT4_Pos           (4U)
#define EXTI_RTSR_RT4_Msk           (0x1UL << EXTI_RTSR_RT4_Pos)             /*!< 0x00000010 */
#define EXTI_RTSR_RT4               EXTI_RTSR_RT4_Msk                        /*!< Rising trigger configuration for input line 4 */

#define EXTI_RTSR_RT5_Pos           (5U)
#define EXTI_RTSR_RT5_Msk           (0x1UL << EXTI_RTSR_RT5_Pos)             /*!< 0x00000020 */
#define EXTI_RTSR_RT5               EXTI_RTSR_RT5_Msk                        /*!< Rising trigger configuration for input line 5 */

#define EXTI_RTSR_RT6_Pos           (6U)
#define EXTI_RTSR_RT6_Msk           (0x1UL << EXTI_RTSR_RT6_Pos)             /*!< 0x00000040 */
#define EXTI_RTSR_RT6               EXTI_RTSR_RT6_Msk                        /*!< Rising trigger configuration for input line 6 */

#define EXTI_RTSR_RT7_Pos           (7U)
#define EXTI_RTSR_RT7_Msk           (0x1UL << EXTI_RTSR_RT7_Pos)             /*!< 0x00000080 */
#define EXTI_RTSR_RT7               EXTI_RTSR_RT7_Msk                        /*!< Rising trigger configuration for input line 7 */

#define EXTI_RTSR_RT8_Pos           (8U)
#define EXTI_RTSR_RT8_Msk           (0x1UL << EXTI_RTSR_RT8_Pos)             /*!< 0x00000100 */
#define EXTI_RTSR_RT8               EXTI_RTSR_RT8_Msk                        /*!< Rising trigger configuration for input line 8 */

#define EXTI_RTSR_RT9_Pos           (9U)
#define EXTI_RTSR_RT9_Msk           (0x1UL << EXTI_RTSR_RT9_Pos)             /*!< 0x00000200 */
#define EXTI_RTSR_RT9               EXTI_RTSR_RT9_Msk                        /*!< Rising trigger configuration for input line 9 */

#define EXTI_RTSR_RT10_Pos          (10U)
#define EXTI_RTSR_RT10_Msk          (0x1UL << EXTI_RTSR_RT10_Pos)             /*!< 0x00000400 */
#define EXTI_RTSR_RT10              EXTI_RTSR_RT10_Msk                        /*!< Rising trigger configuration for input line 10 */

#define EXTI_RTSR_RT11_Pos          (11U)
#define EXTI_RTSR_RT11_Msk          (0x1UL << EXTI_RTSR_RT11_Pos)             /*!< 0x00000800 */
#define EXTI_RTSR_RT11              EXTI_RTSR_RT11_Msk                        /*!< Rising trigger configuration for input line 11 */

#define EXTI_RTSR_RT12_Pos          (12U)
#define EXTI_RTSR_RT12_Msk          (0x1UL << EXTI_RTSR_RT12_Pos)             /*!< 0x00001000 */
#define EXTI_RTSR_RT12              EXTI_RTSR_RT12_Msk                        /*!< Rising trigger configuration for input line 12 */

#define EXTI_RTSR_RT13_Pos          (13U)
#define EXTI_RTSR_RT13_Msk          (0x1UL << EXTI_RTSR_RT13_Pos)             /*!< 0x00002000 */
#define EXTI_RTSR_RT13              EXTI_RTSR_RT13_Msk                        /*!< Rising trigger configuration for input line 13 */

#define EXTI_RTSR_RT14_Pos          (14U)
#define EXTI_RTSR_RT14_Msk          (0x1UL << EXTI_RTSR_RT14_Pos)             /*!< 0x00004000 */
#define EXTI_RTSR_RT14              EXTI_RTSR_RT14_Msk                        /*!< Rising trigger configuration for input line 14 */

#define EXTI_RTSR_RT15_Pos          (15U)
#define EXTI_RTSR_RT15_Msk          (0x1UL << EXTI_RTSR_RT15_Pos)             /*!< 0x00008000 */
#define EXTI_RTSR_RT15              EXTI_RTSR_RT15_Msk                        /*!< Rising trigger configuration for input line 15 */

#define EXTI_RTSR_RT17_Pos          (17U)
#define EXTI_RTSR_RT17_Msk          (0x1UL << EXTI_RTSR_RT17_Pos)            /*!< 0x00020000 */
#define EXTI_RTSR_RT17              EXTI_RTSR_RT17_Msk                       /*!< Rising trigger configuration for input line 17 */

#define EXTI_RTSR_RT18_Pos          (18U)
#define EXTI_RTSR_RT18_Msk          (0x1UL << EXTI_RTSR_RT18_Pos)            /*!< 0x00040000 */
#define EXTI_RTSR_RT18              EXTI_RTSR_RT18_Msk                       /*!< Rising trigger configuration for input line 18 */

/******************  Bit definition for EXTI_FTSR register  ******************/
#define EXTI_FTSR_FT0_Pos           (0U)
#define EXTI_FTSR_FT0_Msk           (0x1UL << EXTI_FTSR_FT0_Pos)             /*!< 0x00000001 */
#define EXTI_FTSR_FT0               EXTI_FTSR_FT0_Msk                        /*!< Falling trigger configuration for input line 0 */

#define EXTI_FTSR_FT1_Pos           (1U)
#define EXTI_FTSR_FT1_Msk           (0x1UL << EXTI_FTSR_FT1_Pos)             /*!< 0x00000002 */
#define EXTI_FTSR_FT1               EXTI_FTSR_FT1_Msk                        /*!< Falling trigger configuration for input line 1 */

#define EXTI_FTSR_FT2_Pos           (2U)
#define EXTI_FTSR_FT2_Msk           (0x1UL << EXTI_FTSR_FT2_Pos)             /*!< 0x00000004 */
#define EXTI_FTSR_FT2               EXTI_FTSR_FT2_Msk                        /*!< Falling trigger configuration for input line 2 */

#define EXTI_FTSR_FT3_Pos           (3U)
#define EXTI_FTSR_FT3_Msk           (0x1UL << EXTI_FTSR_FT3_Pos)             /*!< 0x00000008 */
#define EXTI_FTSR_FT3               EXTI_FTSR_FT3_Msk                        /*!< Falling trigger configuration for input line 3 */

#define EXTI_FTSR_FT4_Pos           (4U)
#define EXTI_FTSR_FT4_Msk           (0x1UL << EXTI_FTSR_FT4_Pos)             /*!< 0x00000010 */
#define EXTI_FTSR_FT4               EXTI_FTSR_FT4_Msk                        /*!< Falling trigger configuration for input line 4 */

#define EXTI_FTSR_FT5_Pos           (5U)
#define EXTI_FTSR_FT5_Msk           (0x1UL << EXTI_FTSR_FT5_Pos)             /*!< 0x00000020 */
#define EXTI_FTSR_FT5               EXTI_FTSR_FT5_Msk                        /*!< Falling trigger configuration for input line 5 */

#define EXTI_FTSR_FT6_Pos           (6U)
#define EXTI_FTSR_FT6_Msk           (0x1UL << EXTI_FTSR_FT6_Pos)             /*!< 0x00000040 */
#define EXTI_FTSR_FT6               EXTI_FTSR_FT6_Msk                        /*!< Falling trigger configuration for input line 6 */

#define EXTI_FTSR_FT7_Pos           (7U)
#define EXTI_FTSR_FT7_Msk           (0x1UL << EXTI_FTSR_FT7_Pos)             /*!< 0x00000080 */
#define EXTI_FTSR_FT7               EXTI_FTSR_FT7_Msk                        /*!< Falling trigger configuration for input line 7 */

#define EXTI_FTSR_FT8_Pos           (8U)
#define EXTI_FTSR_FT8_Msk           (0x1UL << EXTI_FTSR_FT8_Pos)             /*!< 0x00000100 */
#define EXTI_FTSR_FT8               EXTI_FTSR_FT8_Msk                        /*!< Falling trigger configuration for input line 8 */

#define EXTI_FTSR_FT9_Pos           (9U)
#define EXTI_FTSR_FT9_Msk           (0x1UL << EXTI_FTSR_FT9_Pos)             /*!< 0x00000200 */
#define EXTI_FTSR_FT9               EXTI_FTSR_FT9_Msk                        /*!< Falling trigger configuration for input line 9 */

#define EXTI_FTSR_FT10_Pos          (10U)
#define EXTI_FTSR_FT10_Msk          (0x1UL << EXTI_FTSR_FT10_Pos)             /*!< 0x00000400 */
#define EXTI_FTSR_FT10              EXTI_FTSR_FT10_Msk                        /*!< Falling trigger configuration for input line 10 */

#define EXTI_FTSR_FT11_Pos          (11U)
#define EXTI_FTSR_FT11_Msk          (0x1UL << EXTI_FTSR_FT11_Pos)             /*!< 0x00000800 */
#define EXTI_FTSR_FT11              EXTI_FTSR_FT11_Msk                        /*!< Falling trigger configuration for input line 11 */

#define EXTI_FTSR_FT12_Pos          (12U)
#define EXTI_FTSR_FT12_Msk          (0x1UL << EXTI_FTSR_FT12_Pos)             /*!< 0x00001000 */
#define EXTI_FTSR_FT12              EXTI_FTSR_FT12_Msk                        /*!< Falling trigger configuration for input line 12 */

#define EXTI_FTSR_FT13_Pos          (13U)
#define EXTI_FTSR_FT13_Msk          (0x1UL << EXTI_FTSR_FT13_Pos)             /*!< 0x00002000 */
#define EXTI_FTSR_FT13              EXTI_FTSR_FT13_Msk                        /*!< Falling trigger configuration for input line 13 */

#define EXTI_FTSR_FT14_Pos          (14U)
#define EXTI_FTSR_FT14_Msk          (0x1UL << EXTI_FTSR_FT14_Pos)             /*!< 0x00004000 */
#define EXTI_FTSR_FT14              EXTI_FTSR_FT14_Msk                        /*!< Falling trigger configuration for input line 14 */

#define EXTI_FTSR_FT15_Pos          (15U)
#define EXTI_FTSR_FT15_Msk          (0x1UL << EXTI_FTSR_FT15_Pos)             /*!< 0x00008000 */
#define EXTI_FTSR_FT15              EXTI_FTSR_FT15_Msk                        /*!< Falling trigger configuration for input line 15 */

#define EXTI_FTSR_FT17_Pos          (17U)
#define EXTI_FTSR_FT17_Msk          (0x1UL << EXTI_FTSR_FT17_Pos)            /*!< 0x00020000 */
#define EXTI_FTSR_FT17              EXTI_FTSR_FT17_Msk                       /*!< Falling trigger configuration for input line 17 */

#define EXTI_FTSR_FT18_Pos          (18U)
#define EXTI_FTSR_FT18_Msk          (0x1UL << EXTI_FTSR_FT18_Pos)            /*!< 0x00040000 */
#define EXTI_FTSR_FT18              EXTI_FTSR_FT18_Msk                       /*!< Falling trigger configuration for input line 18 */

/******************  Bit definition for EXTI_SWIER register  *****************/
#define EXTI_SWIER_SWI0_Pos         (0U)
#define EXTI_SWIER_SWI0_Msk         (0x1UL << EXTI_SWIER_SWI0_Pos)           /*!< 0x00000001 */
#define EXTI_SWIER_SWI0             EXTI_SWIER_SWI0_Msk                      /*!< Software Interrupt on line 0 */

#define EXTI_SWIER_SWI1_Pos         (1U)
#define EXTI_SWIER_SWI1_Msk         (0x1UL << EXTI_SWIER_SWI1_Pos)           /*!< 0x00000002 */
#define EXTI_SWIER_SWI1             EXTI_SWIER_SWI1_Msk                      /*!< Software Interrupt on line 1 */

#define EXTI_SWIER_SWI2_Pos         (2U)
#define EXTI_SWIER_SWI2_Msk         (0x1UL << EXTI_SWIER_SWI2_Pos)           /*!< 0x00000004 */
#define EXTI_SWIER_SWI2             EXTI_SWIER_SWI2_Msk                      /*!< Software Interrupt on line 2 */

#define EXTI_SWIER_SWI3_Pos         (3U)
#define EXTI_SWIER_SWI3_Msk         (0x1UL << EXTI_SWIER_SWI3_Pos)           /*!< 0x00000008 */
#define EXTI_SWIER_SWI3             EXTI_SWIER_SWI3_Msk                      /*!< Software Interrupt on line 3 */

#define EXTI_SWIER_SWI4_Pos         (4U)
#define EXTI_SWIER_SWI4_Msk         (0x1UL << EXTI_SWIER_SWI4_Pos)           /*!< 0x00000010 */
#define EXTI_SWIER_SWI4             EXTI_SWIER_SWI4_Msk                      /*!< Software Interrupt on line 4 */

#define EXTI_SWIER_SWI5_Pos         (5U)
#define EXTI_SWIER_SWI5_Msk         (0x1UL << EXTI_SWIER_SWI5_Pos)           /*!< 0x00000020 */
#define EXTI_SWIER_SWI5             EXTI_SWIER_SWI5_Msk                      /*!< Software Interrupt on line 5 */

#define EXTI_SWIER_SWI6_Pos         (6U)
#define EXTI_SWIER_SWI6_Msk         (0x1UL << EXTI_SWIER_SWI6_Pos)           /*!< 0x00000040 */
#define EXTI_SWIER_SWI6             EXTI_SWIER_SWI6_Msk                      /*!< Software Interrupt on line 6 */

#define EXTI_SWIER_SWI7_Pos         (7U)
#define EXTI_SWIER_SWI7_Msk         (0x1UL << EXTI_SWIER_SWI7_Pos)           /*!< 0x00000080 */
#define EXTI_SWIER_SWI7             EXTI_SWIER_SWI7_Msk                      /*!< Software Interrupt on line 7 */

#define EXTI_SWIER_SWI8_Pos         (8U)
#define EXTI_SWIER_SWI8_Msk         (0x1UL << EXTI_SWIER_SWI8_Pos)           /*!< 0x00000100 */
#define EXTI_SWIER_SWI8             EXTI_SWIER_SWI8_Msk                      /*!< Software Interrupt on line 8 */

#define EXTI_SWIER_SWI9_Pos         (9U)
#define EXTI_SWIER_SWI9_Msk         (0x1UL << EXTI_SWIER_SWI9_Pos)           /*!< 0x00000200 */
#define EXTI_SWIER_SWI9             EXTI_SWIER_SWI9_Msk                      /*!< Software Interrupt on line 9 */

#define EXTI_SWIER_SWI10_Pos        (10U)
#define EXTI_SWIER_SWI10_Msk        (0x1UL << EXTI_SWIER_SWI10_Pos)           /*!< 0x00000400 */
#define EXTI_SWIER_SWI10            EXTI_SWIER_SWI10_Msk                      /*!< Software Interrupt on line 10 */

#define EXTI_SWIER_SWI11_Pos        (11U)
#define EXTI_SWIER_SWI11_Msk        (0x1UL << EXTI_SWIER_SWI11_Pos)           /*!< 0x00000800 */
#define EXTI_SWIER_SWI11            EXTI_SWIER_SWI11_Msk                      /*!< Software Interrupt on line 11 */

#define EXTI_SWIER_SWI12_Pos        (12U)
#define EXTI_SWIER_SWI12_Msk        (0x1UL << EXTI_SWIER_SWI12_Pos)           /*!< 0x00001000 */
#define EXTI_SWIER_SWI12            EXTI_SWIER_SWI12_Msk                      /*!< Software Interrupt on line 12 */

#define EXTI_SWIER_SWI13_Pos        (13U)
#define EXTI_SWIER_SWI13_Msk        (0x1UL << EXTI_SWIER_SWI13_Pos)           /*!< 0x00002000 */
#define EXTI_SWIER_SWI13            EXTI_SWIER_SWI13_Msk                      /*!< Software Interrupt on line 13 */

#define EXTI_SWIER_SWI14_Pos        (14U)
#define EXTI_SWIER_SWI14_Msk        (0x1UL << EXTI_SWIER_SWI14_Pos)           /*!< 0x00004000 */
#define EXTI_SWIER_SWI14            EXTI_SWIER_SWI14_Msk                      /*!< Software Interrupt on line 14 */

#define EXTI_SWIER_SWI15_Pos        (15U)
#define EXTI_SWIER_SWI15_Msk        (0x1UL << EXTI_SWIER_SWI15_Pos)           /*!< 0x00008000 */
#define EXTI_SWIER_SWI15            EXTI_SWIER_SWI15_Msk                      /*!< Software Interrupt on line 15 */

#define EXTI_SWIER_SWI17_Pos        (17U)
#define EXTI_SWIER_SWI17_Msk        (0x1UL << EXTI_SWIER_SWI17_Pos)          /*!< 0x00020000 */
#define EXTI_SWIER_SWI17            EXTI_SWIER_SWI17_Msk                     /*!< Software Interrupt on line 17 */

#define EXTI_SWIER_SWI18_Pos        (18U)
#define EXTI_SWIER_SWI18_Msk        (0x1UL << EXTI_SWIER_SWI18_Pos)          /*!< 0x00040000 */
#define EXTI_SWIER_SWI18            EXTI_SWIER_SWI18_Msk                     /*!< Software Interrupt on line 18 */

/*******************  Bit definition for EXTI_PR register  ******************/
#define EXTI_PR_PR0_Pos             (0U)
#define EXTI_PR_PR0_Msk             (0x1UL << EXTI_PR_PR0_Pos)            /*!< 0x00000001 */
#define EXTI_PR_PR0                 EXTI_PR_PR0_Msk                       /*!< Rising Pending Interrupt Flag on line 0 */

#define EXTI_PR_PR1_Pos             (1U)
#define EXTI_PR_PR1_Msk             (0x1UL << EXTI_PR_PR1_Pos)            /*!< 0x00000002 */
#define EXTI_PR_PR1                 EXTI_PR_PR1_Msk                       /*!< Rising Pending Interrupt Flag on line 1 */

#define EXTI_PR_PR2_Pos             (2U)
#define EXTI_PR_PR2_Msk             (0x1UL << EXTI_PR_PR2_Pos)            /*!< 0x00000004 */
#define EXTI_PR_PR2                 EXTI_PR_PR2_Msk                       /*!< Rising Pending Interrupt Flag on line 2 */

#define EXTI_PR_PR3_Pos             (3U)
#define EXTI_PR_PR3_Msk             (0x1UL << EXTI_PR_PR3_Pos)            /*!< 0x00000008 */
#define EXTI_PR_PR3                 EXTI_PR_PR3_Msk                       /*!< Rising Pending Interrupt Flag on line 3 */

#define EXTI_PR_PR4_Pos             (4U)
#define EXTI_PR_PR4_Msk             (0x1UL << EXTI_PR_PR4_Pos )           /*!< 0x00000010 */
#define EXTI_PR_PR4                 EXTI_PR_PR4_Msk                       /*!< Rising Pending Interrupt Flag on line 4 */

#define EXTI_PR_PR5_Pos             (5U)
#define EXTI_PR_PR5_Msk             (0x1UL << EXTI_PR_PR5_Pos )            /*!< 0x00000020 */
#define EXTI_PR_PR5                 EXTI_PR_PR5_Msk                       /*!< Rising Pending Interrupt Flag on line 5 */

#define EXTI_PR_PR6_Pos             (6U)
#define EXTI_PR_PR6_Msk             (0x1UL << EXTI_PR_PR6_Pos)            /*!< 0x00000040 */
#define EXTI_PR_PR6                 EXTI_PR_PR6_Msk                       /*!< Rising Pending Interrupt Flag on line 6 */

#define EXTI_PR_PR7_Pos             (7U)
#define EXTI_PR_PR7_Msk             (0x1UL << EXTI_PR_PR7_Pos)            /*!< 0x00000080 */
#define EXTI_PR_PR7                 EXTI_PR_PR7_Msk                       /*!< Rising Pending Interrupt Flag on line 7 */

#define EXTI_PR_PR8_Pos             (8U)
#define EXTI_PR_PR8_Msk             (0x1UL << EXTI_PR_PR8_Pos)            /*!< 0x00000100 */
#define EXTI_PR_PR8                 EXTI_PR_PR8_Msk                       /*!< Rising Pending Interrupt Flag on line 8 */

#define EXTI_PR_PR9_Pos             (9U)
#define EXTI_PR_PR9_Msk             (0x1UL << EXTI_PR_PR9_Pos)            /*!< 0x00000200 */
#define EXTI_PR_PR9                 EXTI_PR_PR9_Msk                       /*!< Rising Pending Interrupt Flag on line 9 */

#define EXTI_PR_PR10_Pos            (10U)
#define EXTI_PR_PR10_Msk            (0x1UL << EXTI_PR_PR10_Pos)            /*!< 0x00000400 */
#define EXTI_PR_PR10                EXTI_PR_PR10_Msk                       /*!< Rising Pending Interrupt Flag on line 10 */

#define EXTI_PR_PR11_Pos            (11U)
#define EXTI_PR_PR11_Msk            (0x1UL << EXTI_PR_PR11_Pos)            /*!< 0x00000800 */
#define EXTI_PR_PR11                EXTI_PR_PR11_Msk                       /*!< Rising Pending Interrupt Flag on line 11 */

#define EXTI_PR_PR12_Pos            (12U)
#define EXTI_PR_PR12_Msk            (0x1UL << EXTI_PR_PR12_Pos )           /*!< 0x00001000 */
#define EXTI_PR_PR12                EXTI_PR_PR12_Msk                       /*!< Rising Pending Interrupt Flag on line 12 */

#define EXTI_PR_PR13_Pos            (13U)
#define EXTI_PR_PR13_Msk            (0x1UL << EXTI_PR_PR13_Pos )            /*!< 0x00002000 */
#define EXTI_PR_PR13                EXTI_PR_PR13_Msk                       /*!< Rising Pending Interrupt Flag on line 13 */

#define EXTI_PR_PR14_Pos            (14U)
#define EXTI_PR_PR14_Msk            (0x1UL << EXTI_PR_PR14_Pos)            /*!< 0x00004000 */
#define EXTI_PR_PR14                EXTI_PR_PR14_Msk                       /*!< Rising Pending Interrupt Flag on line 14 */

#define EXTI_PR_PR15_Pos            (15U)
#define EXTI_PR_PR15_Msk            (0x1UL << EXTI_PR_PR15_Pos)            /*!< 0x00008000 */
#define EXTI_PR_PR15                EXTI_PR_PR15_Msk                       /*!< Rising Pending Interrupt Flag on line 15 */

#define EXTI_PR_PR17_Pos            (17U)
#define EXTI_PR_PR17_Msk            (0x1UL << EXTI_PR_PR17_Pos)           /*!< 0x00020000 */
#define EXTI_PR_PR17                EXTI_PR_PR17_Msk                      /*!< Rising Pending Interrupt Flag on line 17 */

#define EXTI_PR_PR18_Pos            (18U)
#define EXTI_PR_PR18_Msk            (0x1UL << EXTI_PR_PR18_Pos)           /*!< 0x00040000 */
#define EXTI_PR_PR18                EXTI_PR_PR18_Msk                      /*!< Rising Pending Interrupt Flag on line 18 */

/*****************  Bit definition for EXTI_EXTICR1 register  **************/
#define EXTI_EXTICR1_EXTI0_Pos       (0U)
#define EXTI_EXTICR1_EXTI0_Msk       (0x3UL << EXTI_EXTICR1_EXTI0_Pos)         /*!< 0x00000003 */
#define EXTI_EXTICR1_EXTI0           EXTI_EXTICR1_EXTI0_Msk                    /*!< EXTI 0 configuration */
#define EXTI_EXTICR1_EXTI0_0         (0x1UL << EXTI_EXTICR1_EXTI0_Pos)         /*!< 0x00000001 */
#define EXTI_EXTICR1_EXTI0_1         (0x2UL << EXTI_EXTICR1_EXTI0_Pos)         /*!< 0x00000002 */

#define EXTI_EXTICR1_EXTI1_Pos       (8U)
#define EXTI_EXTICR1_EXTI1_Msk       (0x3UL << EXTI_EXTICR1_EXTI1_Pos)         /*!< 0x00000300 */
#define EXTI_EXTICR1_EXTI1           EXTI_EXTICR1_EXTI1_Msk                    /*!< EXTI 1 configuration */
#define EXTI_EXTICR1_EXTI1_0         (0x1UL << EXTI_EXTICR1_EXTI1_Pos)         /*!< 0x00000100 */
#define EXTI_EXTICR1_EXTI1_1         (0x2UL << EXTI_EXTICR1_EXTI1_Pos)         /*!< 0x00000200 */

#define EXTI_EXTICR1_EXTI2_Pos       (16U)
#define EXTI_EXTICR1_EXTI2_Msk       (0x3UL << EXTI_EXTICR1_EXTI2_Pos)         /*!< 0x00030000 */
#define EXTI_EXTICR1_EXTI2           EXTI_EXTICR1_EXTI2_Msk                    /*!< EXTI 2 configuration */
#define EXTI_EXTICR1_EXTI2_0         (0x1UL << EXTI_EXTICR1_EXTI2_Pos)         /*!< 0x00010000 */
#define EXTI_EXTICR1_EXTI2_1         (0x2UL << EXTI_EXTICR1_EXTI2_Pos)         /*!< 0x00020000 */

#define EXTI_EXTICR1_EXTI3_Pos       (24U)
#define EXTI_EXTICR1_EXTI3_Msk       (0x3UL << EXTI_EXTICR1_EXTI3_Pos)         /*!< 0x03000000 */
#define EXTI_EXTICR1_EXTI3           EXTI_EXTICR1_EXTI3_Msk                    /*!< EXTI 3 configuration */
#define EXTI_EXTICR1_EXTI3_0         (0x1UL << EXTI_EXTICR1_EXTI3_Pos)         /*!< 0x01000000 */
#define EXTI_EXTICR1_EXTI3_1         (0x2UL << EXTI_EXTICR1_EXTI3_Pos)         /*!< 0x02000000 */

/*****************  Bit definition for EXTI_EXTICR2 register  **************/
#define EXTI_EXTICR2_EXTI4_Pos       (0U)
#define EXTI_EXTICR2_EXTI4_Msk       (0x3UL << EXTI_EXTICR2_EXTI4_Pos)         /*!< 0x00000003 */
#define EXTI_EXTICR2_EXTI4           EXTI_EXTICR2_EXTI4_Msk                    /*!< EXTI 4 configuration */
#define EXTI_EXTICR2_EXTI4_0         (0x1UL << EXTI_EXTICR2_EXTI4_Pos)         /*!< 0x00000001 */
#define EXTI_EXTICR2_EXTI4_1         (0x2UL << EXTI_EXTICR2_EXTI4_Pos)         /*!< 0x00000002 */

#define EXTI_EXTICR2_EXTI5_Pos       (8U)
#define EXTI_EXTICR2_EXTI5_Msk       (0x3UL << EXTI_EXTICR2_EXTI5_Pos)         /*!< 0x00000300 */
#define EXTI_EXTICR2_EXTI5           EXTI_EXTICR2_EXTI5_Msk                    /*!< EXTI 5 configuration */
#define EXTI_EXTICR2_EXTI5_0         (0x1UL << EXTI_EXTICR2_EXTI5_Pos)         /*!< 0x00000100 */
#define EXTI_EXTICR2_EXTI5_1         (0x2UL << EXTI_EXTICR2_EXTI5_Pos)         /*!< 0x00000200 */

/*******************  Bit definition for EXTI_IMR1 register  ******************/
#define EXTI_IMR_IM_Pos             (0U)
#define EXTI_IMR_IM_Msk             (0x108EFFFFUL << EXTI_IMR_IM_Pos)        /*!< 0x108EFFFF */
#define EXTI_IMR_IM                 EXTI_IMR_IM_Msk                          /*!< Interrupt Mask All */

#define EXTI_IMR_IM0_Pos            (0U)
#define EXTI_IMR_IM0_Msk            (0x1UL << EXTI_IMR_IM0_Pos)              /*!< 0x00000001 */
#define EXTI_IMR_IM0                EXTI_IMR_IM0_Msk                         /*!< Interrupt Mask on line 0 */

#define EXTI_IMR_IM1_Pos            (1U)
#define EXTI_IMR_IM1_Msk            (0x1UL << EXTI_IMR_IM1_Pos)              /*!< 0x00000002 */
#define EXTI_IMR_IM1                EXTI_IMR_IM1_Msk                         /*!< Interrupt Mask on line 1 */

#define EXTI_IMR_IM2_Pos            (2U)
#define EXTI_IMR_IM2_Msk            (0x1UL << EXTI_IMR_IM2_Pos)              /*!< 0x00000004 */
#define EXTI_IMR_IM2                EXTI_IMR_IM2_Msk                         /*!< Interrupt Mask on line 2 */

#define EXTI_IMR_IM3_Pos            (3U)
#define EXTI_IMR_IM3_Msk            (0x1UL << EXTI_IMR_IM3_Pos)              /*!< 0x00000008 */
#define EXTI_IMR_IM3                EXTI_IMR_IM3_Msk                         /*!< Interrupt Mask on line 3 */

#define EXTI_IMR_IM4_Pos            (4U)
#define EXTI_IMR_IM4_Msk            (0x1UL << EXTI_IMR_IM4_Pos)              /*!< 0x00000010 */
#define EXTI_IMR_IM4                EXTI_IMR_IM4_Msk                         /*!< Interrupt Mask on line 4 */

#define EXTI_IMR_IM5_Pos            (5U)
#define EXTI_IMR_IM5_Msk            (0x1UL << EXTI_IMR_IM5_Pos)              /*!< 0x00000020 */
#define EXTI_IMR_IM5                EXTI_IMR_IM5_Msk                         /*!< Interrupt Mask on line 5 */

#define EXTI_IMR_IM6_Pos            (6U)
#define EXTI_IMR_IM6_Msk            (0x1UL << EXTI_IMR_IM6_Pos)              /*!< 0x00000040 */
#define EXTI_IMR_IM6                EXTI_IMR_IM6_Msk                         /*!< Interrupt Mask on line 6 */

#define EXTI_IMR_IM7_Pos            (7U)
#define EXTI_IMR_IM7_Msk            (0x1UL << EXTI_IMR_IM7_Pos)              /*!< 0x00000080 */
#define EXTI_IMR_IM7                EXTI_IMR_IM7_Msk                         /*!< Interrupt Mask on line 7 */

#define EXTI_IMR_IM8_Pos            (8U)
#define EXTI_IMR_IM8_Msk            (0x1UL << EXTI_IMR_IM8_Pos)              /*!< 0x00000100 */
#define EXTI_IMR_IM8                EXTI_IMR_IM8_Msk                         /*!< Interrupt Mask on line 8 */

#define EXTI_IMR_IM9_Pos            (9U)
#define EXTI_IMR_IM9_Msk            (0x1UL << EXTI_IMR_IM9_Pos)              /*!< 0x00000200 */
#define EXTI_IMR_IM9                EXTI_IMR_IM9_Msk                         /*!< Interrupt Mask on line 9 */

#define EXTI_IMR_IM10_Pos           (10U)
#define EXTI_IMR_IM10_Msk           (0x1UL << EXTI_IMR_IM10_Pos)              /*!< 0x00000400 */
#define EXTI_IMR_IM10               EXTI_IMR_IM10_Msk                         /*!< Interrupt Mask on line 10 */

#define EXTI_IMR_IM11_Pos           (11U)
#define EXTI_IMR_IM11_Msk           (0x1UL << EXTI_IMR_IM11_Pos)              /*!< 0x00000800 */
#define EXTI_IMR_IM11               EXTI_IMR_IM11_Msk                         /*!< Interrupt Mask on line 11 */

#define EXTI_IMR_IM12_Pos           (12U)
#define EXTI_IMR_IM12_Msk           (0x1UL << EXTI_IMR_IM12_Pos)              /*!< 0x00001000 */
#define EXTI_IMR_IM12               EXTI_IMR_IM12_Msk                         /*!< Interrupt Mask on line 12 */

#define EXTI_IMR_IM13_Pos           (13U)
#define EXTI_IMR_IM13_Msk           (0x1UL << EXTI_IMR_IM13_Pos)              /*!< 0x00002000 */
#define EXTI_IMR_IM13               EXTI_IMR_IM13_Msk                         /*!< Interrupt Mask on line 13 */

#define EXTI_IMR_IM14_Pos           (14U)
#define EXTI_IMR_IM14_Msk           (0x1UL << EXTI_IMR_IM14_Pos)              /*!< 0x00004000 */
#define EXTI_IMR_IM14               EXTI_IMR_IM14_Msk                         /*!< Interrupt Mask on line 14 */

#define EXTI_IMR_IM15_Pos           (15U)
#define EXTI_IMR_IM15_Msk           (0x1UL << EXTI_IMR_IM15_Pos)              /*!< 0x00008000 */
#define EXTI_IMR_IM15               EXTI_IMR_IM15_Msk                         /*!< Interrupt Mask on line 15 */

#define EXTI_IMR_IM17_Pos           (17U)
#define EXTI_IMR_IM17_Msk           (0x1UL << EXTI_IMR_IM17_Pos)             /*!< 0x00020000 */
#define EXTI_IMR_IM17               EXTI_IMR_IM17_Msk                        /*!< Interrupt Mask on line 17 */

#define EXTI_IMR_IM18_Pos           (18U)
#define EXTI_IMR_IM18_Msk           (0x1UL << EXTI_IMR_IM18_Pos)             /*!< 0x00040000 */
#define EXTI_IMR_IM18               EXTI_IMR_IM18_Msk                        /*!< Interrupt Mask on line 18 */

#define EXTI_IMR_IM19_Pos           (19U)
#define EXTI_IMR_IM19_Msk           (0x1UL << EXTI_IMR_IM19_Pos)             /*!< 0x00080000 */
#define EXTI_IMR_IM19               EXTI_IMR_IM19_Msk                        /*!< Interrupt Mask on line 19 */

#define EXTI_IMR_IM23_Pos           (23U)
#define EXTI_IMR_IM23_Msk           (0x1UL << EXTI_IMR_IM23_Pos)             /*!< 0x00800000 */
#define EXTI_IMR_IM23               EXTI_IMR_IM23_Msk                        /*!< Interrupt Mask on line 23 */

#define EXTI_IMR_IM28_Pos           (28U)
#define EXTI_IMR_IM28_Msk           (0x1UL << EXTI_IMR_IM28_Pos)             /*!< 0x10000000 */
#define EXTI_IMR_IM28               EXTI_IMR_IM28_Msk                        /*!< Interrupt Mask on line 29 */

/*******************  Bit definition for EXTI_EMR1 register  ******************/
#define EXTI_EMR_EM_Pos             (0U)
#define EXTI_EMR_EM_Msk             (0x108EFFFFUL << EXTI_EMR_EM_Pos)        /*!< 0x108EFFFF */
#define EXTI_EMR_EM                 EXTI_EMR_EM_Msk                          /*!< Event Mask All */

#define EXTI_EMR_EM0_Pos            (0U)
#define EXTI_EMR_EM0_Msk            (0x1UL << EXTI_EMR_EM0_Pos)              /*!< 0x00000001 */
#define EXTI_EMR_EM0                EXTI_EMR_EM0_Msk                         /*!< Event Mask on line 0 */

#define EXTI_EMR_EM1_Pos            (1U)
#define EXTI_EMR_EM1_Msk            (0x1UL << EXTI_EMR_EM1_Pos)              /*!< 0x00000002 */
#define EXTI_EMR_EM1                EXTI_EMR_EM1_Msk                         /*!< Event Mask on line 1 */

#define EXTI_EMR_EM2_Pos            (2U)
#define EXTI_EMR_EM2_Msk            (0x1UL << EXTI_EMR_EM2_Pos)              /*!< 0x00000004 */
#define EXTI_EMR_EM2                EXTI_EMR_EM2_Msk                         /*!< Event Mask on line 2 */

#define EXTI_EMR_EM3_Pos            (3U)
#define EXTI_EMR_EM3_Msk            (0x1UL << EXTI_EMR_EM3_Pos)              /*!< 0x00000008 */
#define EXTI_EMR_EM3                EXTI_EMR_EM3_Msk                         /*!< Event Mask on line 3 */

#define EXTI_EMR_EM4_Pos            (4U)
#define EXTI_EMR_EM4_Msk            (0x1UL << EXTI_EMR_EM4_Pos)              /*!< 0x00000010 */
#define EXTI_EMR_EM4                EXTI_EMR_EM4_Msk                         /*!< Event Mask on line 4 */

#define EXTI_EMR_EM5_Pos            (5U)
#define EXTI_EMR_EM5_Msk            (0x1UL << EXTI_EMR_EM5_Pos)              /*!< 0x00000020 */
#define EXTI_EMR_EM5                EXTI_EMR_EM5_Msk                         /*!< Event Mask on line 5 */

#define EXTI_EMR_EM6_Pos            (6U)
#define EXTI_EMR_EM6_Msk            (0x1UL << EXTI_EMR_EM6_Pos)              /*!< 0x00000040 */
#define EXTI_EMR_EM6                EXTI_EMR_EM6_Msk                         /*!< Event Mask on line 6 */

#define EXTI_EMR_EM7_Pos            (7U)
#define EXTI_EMR_EM7_Msk            (0x1UL << EXTI_EMR_EM7_Pos)              /*!< 0x00000080 */
#define EXTI_EMR_EM7                EXTI_EMR_EM7_Msk                         /*!< Event Mask on line 7 */

#define EXTI_EMR_EM8_Pos            (8U)
#define EXTI_EMR_EM8_Msk            (0x1UL << EXTI_EMR_EM8_Pos)              /*!< 0x00000100 */
#define EXTI_EMR_EM8                EXTI_EMR_EM8_Msk                         /*!< Event Mask on line 8 */

#define EXTI_EMR_EM9_Pos            (9U)
#define EXTI_EMR_EM9_Msk            (0x1UL << EXTI_EMR_EM9_Pos)              /*!< 0x00000200 */
#define EXTI_EMR_EM9                EXTI_EMR_EM9_Msk                         /*!< Event Mask on line 9 */

#define EXTI_EMR_EM10_Pos           (10U)
#define EXTI_EMR_EM10_Msk           (0x1UL << EXTI_EMR_EM10_Pos)             /*!< 0x00000400 */
#define EXTI_EMR_EM10               EXTI_EMR_EM10_Msk                        /*!< Event Mask on line 10 */

#define EXTI_EMR_EM11_Pos           (11U)
#define EXTI_EMR_EM11_Msk           (0x1UL << EXTI_EMR_EM11_Pos)             /*!< 0x00000800 */
#define EXTI_EMR_EM11               EXTI_EMR_EM11_Msk                        /*!< Event Mask on line 11 */

#define EXTI_EMR_EM12_Pos           (12U)
#define EXTI_EMR_EM12_Msk           (0x1UL << EXTI_EMR_EM12_Pos)             /*!< 0x00001000 */
#define EXTI_EMR_EM12               EXTI_EMR_EM12_Msk                        /*!< Event Mask on line 12 */

#define EXTI_EMR_EM13_Pos           (13U)
#define EXTI_EMR_EM13_Msk           (0x1UL << EXTI_EMR_EM13_Pos)             /*!< 0x00002000 */
#define EXTI_EMR_EM13               EXTI_EMR_EM13_Msk                        /*!< Event Mask on line 13 */

#define EXTI_EMR_EM14_Pos           (14U)
#define EXTI_EMR_EM14_Msk           (0x1UL << EXTI_EMR_EM14_Pos)             /*!< 0x00004000 */
#define EXTI_EMR_EM14               EXTI_EMR_EM14_Msk                        /*!< Event Mask on line 14 */

#define EXTI_EMR_EM15_Pos           (15U)
#define EXTI_EMR_EM15_Msk           (0x1UL << EXTI_EMR_EM15_Pos)             /*!< 0x00008000 */
#define EXTI_EMR_EM15               EXTI_EMR_EM15_Msk                        /*!< Event Mask on line 15 */

#define EXTI_EMR_EM17_Pos           (17U)
#define EXTI_EMR_EM17_Msk           (0x1UL << EXTI_EMR_EM17_Pos)             /*!< 0x00020000 */
#define EXTI_EMR_EM17               EXTI_EMR_EM17_Msk                        /*!< Event Mask on line 17 */

#define EXTI_EMR_EM18_Pos           (18U)
#define EXTI_EMR_EM18_Msk           (0x1UL << EXTI_EMR_EM18_Pos)             /*!< 0x00040000 */
#define EXTI_EMR_EM18               EXTI_EMR_EM18_Msk                        /*!< Event Mask on line 18 */

#define EXTI_EMR_EM19_Pos           (19U)
#define EXTI_EMR_EM19_Msk           (0x1UL << EXTI_EMR_EM19_Pos)             /*!< 0x00080000 */
#define EXTI_EMR_EM19               EXTI_EMR_EM19_Msk                        /*!< Event Mask on line 19 */

#define EXTI_EMR_EM23_Pos           (23U)
#define EXTI_EMR_EM23_Msk           (0x1UL << EXTI_EMR_EM23_Pos)             /*!< 0x00800000 */
#define EXTI_EMR_EM23               EXTI_EMR_EM23_Msk                        /*!< Event Mask on line 23 */

#define EXTI_EMR_EM28_Pos           (28U)
#define EXTI_EMR_EM28_Msk           (0x1UL << EXTI_EMR_EM28_Pos)             /*!< 0x10000000 */
#define EXTI_EMR_EM28               EXTI_EMR_EM28_Msk                        /*!< Event Mask on line 28 */

/******************************************************************************/
/*                                                                            */
/*         FLASH and Option Bytes Registers                                   */
/*                                                                            */
/******************************************************************************/
#define GPIO_NRST_CONFIG_SUPPORT         /*!< GPIO feature available only on specific devices: Configure NRST pin */
#define FLASH_SECURABLE_MEMORY_SUPPORT   /*!< Flash feature available only on specific devices: allow to secure memory */
#define FLASH_PCROP_SUPPORT              /*!< Flash feature available only on specific devices: proprietary code read protection areas selected by option */

/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY_Pos             (0U)
#define FLASH_ACR_LATENCY_Msk             (0x3UL << FLASH_ACR_LATENCY_Pos)     /*!< 0x00000003 */
#define FLASH_ACR_LATENCY                 FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_LATENCY_0               (0x1UL << FLASH_ACR_LATENCY_Pos) /*!< 0x00000001 */
#define FLASH_ACR_LATENCY_1               (0x2UL << FLASH_ACR_LATENCY_Pos) /*!< 0x00000002 */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define FLASH_KEYR_KEY_Pos                (0U)
#define FLASH_KEYR_KEY_Msk                (0xFFFFFFFFUL << FLASH_KEYR_KEY_Pos) /*!< 0xFFFFFFFF */
#define FLASH_KEYR_KEY                    FLASH_KEYR_KEY_Msk                   /*!< FPEC Key */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define FLASH_OPTKEYR_OPTKEY_Pos          (0U)
#define FLASH_OPTKEYR_OPTKEY_Msk          (0xFFFFFFFFUL << FLASH_OPTKEYR_OPTKEY_Pos) /*!< 0xFFFFFFFF */
#define FLASH_OPTKEYR_OPTKEY              FLASH_OPTKEYR_OPTKEY_Msk             /*!< Option Byte Key */

/******************  FLASH Keys  **********************************************/
#define FLASH_KEY1_Pos                    (0U)
#define FLASH_KEY1_Msk                    (0x45670123UL << FLASH_KEY1_Pos)     /*!< 0x45670123 */
#define FLASH_KEY1                        FLASH_KEY1_Msk                       /*!< Flash program erase key1 */
#define FLASH_KEY2_Pos                    (0U)
#define FLASH_KEY2_Msk                    (0xCDEF89ABUL << FLASH_KEY2_Pos)     /*!< 0xCDEF89AB */
#define FLASH_KEY2                        FLASH_KEY2_Msk                       /*!< Flash program erase key2: used with FLASH_PEKEY1
                                                                                to unlock the write access to the FPEC. */

#define FLASH_OPTKEY1_Pos                 (0U)
#define FLASH_OPTKEY1_Msk                 (0x08192A3BUL << FLASH_OPTKEY1_Pos)  /*!< 0x08192A3B */
#define FLASH_OPTKEY1                     FLASH_OPTKEY1_Msk                    /*!< Flash option key1 */
#define FLASH_OPTKEY2_Pos                 (0U)
#define FLASH_OPTKEY2_Msk                 (0x4C5D6E7FUL << FLASH_OPTKEY2_Pos)  /*!< 0x4C5D6E7F */
#define FLASH_OPTKEY2                     FLASH_OPTKEY2_Msk                    /*!< Flash option key2: used with FLASH_OPTKEY1 to
                                                                                unlock the write access to the option byte block */

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP_Pos                  (0U)
#define FLASH_SR_EOP_Msk                  (0x1UL << FLASH_SR_EOP_Pos)      /*!< 0x00000001 */
#define FLASH_SR_EOP                      FLASH_SR_EOP_Msk

#define FLASH_SR_WRPERR_Pos               (4U)
#define FLASH_SR_WRPERR_Msk               (0x1UL << FLASH_SR_WRPERR_Pos)   /*!< 0x00000010 */
#define FLASH_SR_WRPERR                   FLASH_SR_WRPERR_Msk

#define FLASH_SR_USRLOCK_Pos              (13U)
#define FLASH_SR_USRLOCK_Msk              (0x1UL << FLASH_SR_USRLOCK_Pos)  /*!< 0x00002000 */
#define FLASH_SR_USRLOCK                  FLASH_SR_USRLOCK_Msk

#define FLASH_SR_OPTVERR_Pos              (15U)
#define FLASH_SR_OPTVERR_Msk              (0x1UL << FLASH_SR_OPTVERR_Pos)  /*!< 0x00008000 */
#define FLASH_SR_OPTVERR                  FLASH_SR_OPTVERR_Msk

#define FLASH_SR_BSY_Pos                  (16U)
#define FLASH_SR_BSY_Msk                  (0x1UL << FLASH_SR_BSY_Pos)     /*!< 0x00010000 */
#define FLASH_SR_BSY                      FLASH_SR_BSY_Msk

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG_Pos                 (0U)
#define FLASH_CR_PG_Msk                 (0x1UL << FLASH_CR_PG_Pos)       /*!< 0x00000001 */
#define FLASH_CR_PG                     FLASH_CR_PG_Msk

#define FLASH_CR_PER_Pos                (1U)
#define FLASH_CR_PER_Msk                (0x1UL << FLASH_CR_PER_Pos)      /*!< 0x00000002 */
#define FLASH_CR_PER                    FLASH_CR_PER_Msk

#define FLASH_CR_MER_Pos                (2U)
#define FLASH_CR_MER_Msk                (0x1UL << FLASH_CR_MER_Pos)     /*!< 0x00000004 */
#define FLASH_CR_MER                    FLASH_CR_MER_Msk

#define FLASH_CR_SER_Pos                (11U)
#define FLASH_CR_SER_Msk                (0x1UL << FLASH_CR_SER_Pos)     /*!< 0x00000800 */
#define FLASH_CR_SER                    FLASH_CR_SER_Msk

#define FLASH_CR_OPTSTRT_Pos            (17U)
#define FLASH_CR_OPTSTRT_Msk            (0x1UL << FLASH_CR_OPTSTRT_Pos)  /*!< 0x00020000 */
#define FLASH_CR_OPTSTRT                FLASH_CR_OPTSTRT_Msk

#define FLASH_CR_PGSTRT_Pos             (19U)
#define FLASH_CR_PGSTRT_Msk             (0x1UL << FLASH_CR_PGSTRT_Pos)     /*!< 0x00080000 */
#define FLASH_CR_PGSTRT                 FLASH_CR_PGSTRT_Msk

#define FLASH_CR_EOPIE_Pos              (24U)
#define FLASH_CR_EOPIE_Msk              (0x1UL << FLASH_CR_EOPIE_Pos)    /*!< 0x01000000 */
#define FLASH_CR_EOPIE                  FLASH_CR_EOPIE_Msk

#define FLASH_CR_ERRIE_Pos              (25U)
#define FLASH_CR_ERRIE_Msk              (0x1UL << FLASH_CR_ERRIE_Pos)    /*!< 0x0200000 */
#define FLASH_CR_ERRIE                  FLASH_CR_ERRIE_Msk

#define FLASH_CR_OBL_LAUNCH_Pos         (27U)
#define FLASH_CR_OBL_LAUNCH_Msk         (0x1UL << FLASH_CR_OBL_LAUNCH_Pos) /*!< 0x08000000 */
#define FLASH_CR_OBL_LAUNCH             FLASH_CR_OBL_LAUNCH_Msk

#define FLASH_CR_OPTLOCK_Pos            (30U)
#define FLASH_CR_OPTLOCK_Msk            (0x1UL << FLASH_CR_OPTLOCK_Pos)  /*!< 0x40000000 */
#define FLASH_CR_OPTLOCK                FLASH_CR_OPTLOCK_Msk

#define FLASH_CR_LOCK_Pos               (31U)
#define FLASH_CR_LOCK_Msk               (0x1UL << FLASH_CR_LOCK_Pos)     /*!< 0x80000000 */
#define FLASH_CR_LOCK                   FLASH_CR_LOCK_Msk

/*******************  Bits definition for FLASH_OPTR register  ****************/
#define FLASH_OPTR_RDP_Pos              (0U)
#define FLASH_OPTR_RDP_Msk              (0xFFUL << FLASH_OPTR_RDP_Pos) /*!< 0x000000FF */
#define FLASH_OPTR_RDP                  FLASH_OPTR_RDP_Msk

#define FLASH_OPTR_BOR_EN_Pos           (8U)
#define FLASH_OPTR_BOR_EN_Msk           (0x1UL << FLASH_OPTR_BOR_EN_Pos) /*!< 0x00000100 */
#define FLASH_OPTR_BOR_EN               FLASH_OPTR_BOR_EN_Msk

#define FLASH_OPTR_BOR_LEV_Pos          (9U)
#define FLASH_OPTR_BOR_LEV_Msk          (0x7UL << FLASH_OPTR_BOR_LEV_Pos) /*!< 0x00000E00 */
#define FLASH_OPTR_BOR_LEV              FLASH_OPTR_BOR_LEV_Msk
#define FLASH_OPTR_BOR_LEV_0            (0x1UL << FLASH_OPTR_BOR_LEV_Pos) /*!< 0x00000200 */
#define FLASH_OPTR_BOR_LEV_1            (0x2UL << FLASH_OPTR_BOR_LEV_Pos) /*!< 0x00000400 */
#define FLASH_OPTR_BOR_LEV_2            (0x4UL << FLASH_OPTR_BOR_LEV_Pos) /*!< 0x00000800 */

#define FLASH_OPTR_IWDG_SW_Pos          (12U)
#define FLASH_OPTR_IWDG_SW_Msk          (0x1UL << FLASH_OPTR_IWDG_SW_Pos)   /*!< 0x00001000 */
#define FLASH_OPTR_IWDG_SW              FLASH_OPTR_IWDG_SW_Msk

#define FLASH_OPTR_NRST_MODE_Pos        (14U)
#define FLASH_OPTR_NRST_MODE_Msk        (0x1UL << FLASH_OPTR_NRST_MODE_Pos) /*!< 0x00004000 */
#define FLASH_OPTR_NRST_MODE            FLASH_OPTR_NRST_MODE_Msk

#define FLASH_OPTR_IWDG_STOP_Pos        (15U)
#define FLASH_OPTR_IWDG_STOP_Msk        (0x1UL << FLASH_OPTR_IWDG_STOP_Pos) /*!< 0x00008000 */
#define FLASH_OPTR_IWDG_STOP            FLASH_OPTR_IWDG_STOP_Msk

/*******************  Bits definition for FLASH_SDKR register  ****************/
#define FLASH_SDKR_SDK_STRT_Pos         (0U)
#define FLASH_SDKR_SDK_STRT_Msk         (0xFUL << FLASH_SDKR_SDK_STRT_Pos)  /*!< 0x0000000F */
#define FLASH_SDKR_SDK_STRT             FLASH_SDKR_SDK_STRT_Msk
#define FLASH_SDKR_SDK_STRT_0           (0x01UL << FLASH_SDKR_SDK_STRT_Pos) /*!< 0x00000001 */
#define FLASH_SDKR_SDK_STRT_1           (0x02UL << FLASH_SDKR_SDK_STRT_Pos) /*!< 0x00000002 */
#define FLASH_SDKR_SDK_STRT_2           (0x04UL << FLASH_SDKR_SDK_STRT_Pos) /*!< 0x00000004 */
#define FLASH_SDKR_SDK_STRT_3           (0x08UL << FLASH_SDKR_SDK_STRT_Pos) /*!< 0x00000008 */

#define FLASH_SDKR_SDK_END_Pos          (8U)
#define FLASH_SDKR_SDK_END_Msk          (0xFUL << FLASH_SDKR_SDK_END_Pos)   /*!< 0x00000F00 */
#define FLASH_SDKR_SDK_END              FLASH_SDKR_SDK_END_Msk 
#define FLASH_SDKR_SDK_END_0            (0x01UL << FLASH_SDKR_SDK_END_Pos)  /*!< 0x00000100 */
#define FLASH_SDKR_SDK_END_1            (0x02UL << FLASH_SDKR_SDK_END_Pos)  /*!< 0x00000200 */
#define FLASH_SDKR_SDK_END_2            (0x04UL << FLASH_SDKR_SDK_END_Pos)  /*!< 0x00000400 */
#define FLASH_SDKR_SDK_END_3            (0x08UL << FLASH_SDKR_SDK_END_Pos)  /*!< 0x00000800 */

/******************  Bits definition for FLASH_BTCR register  ***************/
#define FLASH_BTCR_BOOT_SIZE_Pos        (0U)
#define FLASH_BTCR_BOOT_SIZE_Msk        (0x7UL << FLASH_BTCR_BOOT_SIZE_Pos)     /*!< 0x00000007 */
#define FLASH_BTCR_BOOT_SIZE            FLASH_BTCR_BOOT_SIZE_Msk
#define FLASH_BTCR_BOOT_SIZE_0          (0x1UL << FLASH_BTCR_BOOT_SIZE_Pos)     /*!< 0x00000001 */
#define FLASH_BTCR_BOOT_SIZE_1          (0x2UL << FLASH_BTCR_BOOT_SIZE_Pos)     /*!< 0x00000002 */
#define FLASH_BTCR_BOOT_SIZE_2          (0x4UL << FLASH_BTCR_BOOT_SIZE_Pos)     /*!< 0x00000004 */

#define FLASH_BTCR_SWD_MODE_Pos         (8U)
#define FLASH_BTCR_SWD_MODE_Msk         (0x3UL << FLASH_BTCR_SWD_MODE_Pos)     /*!< 0x00000300 */
#define FLASH_BTCR_SWD_MODE             FLASH_BTCR_SWD_MODE_Msk
#define FLASH_BTCR_SWD_MODE_0           (0x1UL << FLASH_BTCR_SWD_MODE_Pos)     /*!< 0x00000100 */
#define FLASH_BTCR_SWD_MODE_1           (0x2UL << FLASH_BTCR_SWD_MODE_Pos)     /*!< 0x00000200 */

#define FLASH_BTCR_BOOT0_Pos            (14U)
#define FLASH_BTCR_BOOT0_Msk            (0x1UL << FLASH_BTCR_BOOT0_Pos)         /*!< 0x00004000 */
#define FLASH_BTCR_BOOT0                FLASH_BTCR_BOOT0_Msk

#define FLASH_BTCR_NBOOT1_Pos           (15U)
#define FLASH_BTCR_NBOOT1_Msk           (0x1UL << FLASH_BTCR_NBOOT1_Pos)        /*!< 0x00008000 */
#define FLASH_BTCR_NBOOT1               FLASH_BTCR_NBOOT1_Msk

/******************  Bits definition for FLASH_WRPR register  ***************/
#define FLASH_WRPR_WRP_Pos              (0U)
#define FLASH_WRPR_WRP_Msk              (0xFFUL << FLASH_WRPR_WRP_Pos)          /*!< 0x000000FF */
#define FLASH_WRPR_WRP                  FLASH_WRPR_WRP_Msk
#define FLASH_WRPR_WRP_0                (0x0001UL << FLASH_WRPR_WRP_Pos)
#define FLASH_WRPR_WRP_1                (0x0002UL << FLASH_WRPR_WRP_Pos)
#define FLASH_WRPR_WRP_2                (0x0004UL << FLASH_WRPR_WRP_Pos)
#define FLASH_WRPR_WRP_3                (0x0008UL << FLASH_WRPR_WRP_Pos)
#define FLASH_WRPR_WRP_4                (0x0010UL << FLASH_WRPR_WRP_Pos)
#define FLASH_WRPR_WRP_5                (0x0020UL << FLASH_WRPR_WRP_Pos)
#define FLASH_WRPR_WRP_6                (0x0040UL << FLASH_WRPR_WRP_Pos)
#define FLASH_WRPR_WRP_7                (0x0080UL << FLASH_WRPR_WRP_Pos)

/******************  Bits definition for FLASH_STCR register  ***************/
#define FLASH_STCR_SLEEP_EN_Pos         (0U)  
#define FLASH_STCR_SLEEP_EN_Msk         (0x1UL << FLASH_STCR_SLEEP_EN_Pos)     /*!< 0x00000001 */
#define FLASH_STCR_SLEEP_EN             FLASH_STCR_SLEEP_EN_Msk

#define FLASH_STCR_SLEEP_TIME_Pos       (8U)
#define FLASH_STCR_SLEEP_TIME_Msk       (0xFFUL << FLASH_STCR_SLEEP_TIME_Pos)  /*!< 0x0000FF00 */
#define FLASH_STCR_SLEEP_TIME           FLASH_STCR_SLEEP_TIME_Msk

/******************  Bits definition for FLASH_TS0 register  ***************/
#define FLASH_TS0_TS0_Pos               (0U)
#define FLASH_TS0_TS0_Msk               (0x1FFUL << FLASH_TS0_TS0_Pos) /*!< 0x000001FF */
#define FLASH_TS0_TS0                   FLASH_TS0_TS0_Msk

/******************  Bits definition for FLASH_TS1 register  ***************/
#define FLASH_TS1_TS1_Pos               (0U)
#define FLASH_TS1_TS1_Msk               (0x3FFUL << FLASH_TS1_TS1_Pos) /*!< 0x000003FF */
#define FLASH_TS1_TS1                   FLASH_TS1_TS1_Msk

/******************  Bits definition for FLASH_TS2P register  ***************/
#define FLASH_TS2P_TS2P_Pos             (0U)
#define FLASH_TS2P_TS2P_Msk             (0x1FFUL << FLASH_TS2P_TS2P_Pos) /*!< 0x000001FF */
#define FLASH_TS2P_TS2P                 FLASH_TS2P_TS2P_Msk

/******************  Bits definition for FLASH_TPS3 register  ***************/
#define FLASH_TPS3_TPS3_Pos             (0U)
#define FLASH_TPS3_TPS3_Msk             (0xFFFUL << FLASH_TPS3_TPS3_Pos) /*!< 0x00000FFF */
#define FLASH_TPS3_TPS3                 FLASH_TPS3_TPS3_Msk

/******************  Bits definition for FLASH_TS3 register  ***************/
#define FLASH_TS3_TS3_Pos               (0U)
#define FLASH_TS3_TS3_Msk               (0x1FFUL << FLASH_TS3_TS3_Pos) /*!< 0x000001FF */
#define FLASH_TS3_TS3                   FLASH_TS3_TS3_Msk

/******************  Bits definition for FLASH_PERTPE register  ***************/
#define FLASH_PERTPE_PERTPE_Pos         (0U)
#define FLASH_PERTPE_PERTPE_Msk         (0x3FFFFUL << FLASH_PERTPE_PERTPE_Pos) /*!< 0x0003FFFF */
#define FLASH_PERTPE_PERTPE             FLASH_PERTPE_PERTPE_Msk

/******************  Bits definition for FLASH_SMERTPE register  ***************/
#define FLASH_SMERTPE_SMERTPE_Pos       (0U)
#define FLASH_SMERTPE_SMERTPE_Msk       (0x3FFFFUL << FLASH_SMERTPE_SMERTPE_Pos) /*!< 0x0003FFFF */
#define FLASH_SMERTPE_SMERTPE           FLASH_SMERTPE_SMERTPE_Msk

/******************  Bits definition for FLASH_PRGTPE register  ***************/
#define FLASH_PRGTPE_PRGTPE_Pos         (0U)
#define FLASH_PRGTPE_PRGTPE_Msk         (0xFFFFUL << FLASH_PRGTPE_PRGTPE_Pos) /*!< 0x0000FFFF */
#define FLASH_PRGTPE_PRGTPE             FLASH_PRGTPE_PRGTPE_Msk

/******************  Bits definition for FLASH_PRETPE register  ***************/
#define FLASH_PRETPE_PRETPE_Pos         (0U)
#define FLASH_PRETPE_PRETPE_Msk         (0x3FFFUL << FLASH_PRETPE_PRETPE_Pos) /*!< 0x00003FFF */
#define FLASH_PRETPE_PRETPE             FLASH_PRETPE_PRETPE_Msk

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O (GPIO)                      */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODE0_Pos           (0U)
#define GPIO_MODER_MODE0_Msk           (0x3UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000003 */
#define GPIO_MODER_MODE0               GPIO_MODER_MODE0_Msk
#define GPIO_MODER_MODE0_0             (0x1UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000001 */
#define GPIO_MODER_MODE0_1             (0x2UL << GPIO_MODER_MODE0_Pos)          /*!< 0x00000002 */
#define GPIO_MODER_MODE1_Pos           (2U)
#define GPIO_MODER_MODE1_Msk           (0x3UL << GPIO_MODER_MODE1_Pos)          /*!< 0x0000000C */
#define GPIO_MODER_MODE1               GPIO_MODER_MODE1_Msk
#define GPIO_MODER_MODE1_0             (0x1UL << GPIO_MODER_MODE1_Pos)          /*!< 0x00000004 */
#define GPIO_MODER_MODE1_1             (0x2UL << GPIO_MODER_MODE1_Pos)          /*!< 0x00000008 */
#define GPIO_MODER_MODE2_Pos           (4U)
#define GPIO_MODER_MODE2_Msk           (0x3UL << GPIO_MODER_MODE2_Pos)          /*!< 0x00000030 */
#define GPIO_MODER_MODE2               GPIO_MODER_MODE2_Msk
#define GPIO_MODER_MODE2_0             (0x1UL << GPIO_MODER_MODE2_Pos)          /*!< 0x00000010 */
#define GPIO_MODER_MODE2_1             (0x2UL << GPIO_MODER_MODE2_Pos)          /*!< 0x00000020 */
#define GPIO_MODER_MODE3_Pos           (6U)
#define GPIO_MODER_MODE3_Msk           (0x3UL << GPIO_MODER_MODE3_Pos)          /*!< 0x000000C0 */
#define GPIO_MODER_MODE3               GPIO_MODER_MODE3_Msk
#define GPIO_MODER_MODE3_0             (0x1UL << GPIO_MODER_MODE3_Pos)          /*!< 0x00000040 */
#define GPIO_MODER_MODE3_1             (0x2UL << GPIO_MODER_MODE3_Pos)          /*!< 0x00000080 */
#define GPIO_MODER_MODE4_Pos           (8U)
#define GPIO_MODER_MODE4_Msk           (0x3UL << GPIO_MODER_MODE4_Pos)          /*!< 0x00000300 */
#define GPIO_MODER_MODE4               GPIO_MODER_MODE4_Msk
#define GPIO_MODER_MODE4_0             (0x1UL << GPIO_MODER_MODE4_Pos)          /*!< 0x00000100 */
#define GPIO_MODER_MODE4_1             (0x2UL << GPIO_MODER_MODE4_Pos)          /*!< 0x00000200 */
#define GPIO_MODER_MODE5_Pos           (10U)
#define GPIO_MODER_MODE5_Msk           (0x3UL << GPIO_MODER_MODE5_Pos)          /*!< 0x00000C00 */
#define GPIO_MODER_MODE5               GPIO_MODER_MODE5_Msk
#define GPIO_MODER_MODE5_0             (0x1UL << GPIO_MODER_MODE5_Pos)          /*!< 0x00000400 */
#define GPIO_MODER_MODE5_1             (0x2UL << GPIO_MODER_MODE5_Pos)          /*!< 0x00000800 */
#define GPIO_MODER_MODE6_Pos           (12U)
#define GPIO_MODER_MODE6_Msk           (0x3UL << GPIO_MODER_MODE6_Pos)          /*!< 0x00003000 */
#define GPIO_MODER_MODE6               GPIO_MODER_MODE6_Msk
#define GPIO_MODER_MODE6_0             (0x1UL << GPIO_MODER_MODE6_Pos)          /*!< 0x00001000 */
#define GPIO_MODER_MODE6_1             (0x2UL << GPIO_MODER_MODE6_Pos)          /*!< 0x00002000 */
#define GPIO_MODER_MODE7_Pos           (14U)
#define GPIO_MODER_MODE7_Msk           (0x3UL << GPIO_MODER_MODE7_Pos)          /*!< 0x0000C000 */
#define GPIO_MODER_MODE7               GPIO_MODER_MODE7_Msk
#define GPIO_MODER_MODE7_0             (0x1UL << GPIO_MODER_MODE7_Pos)          /*!< 0x00004000 */
#define GPIO_MODER_MODE7_1             (0x2UL << GPIO_MODER_MODE7_Pos)          /*!< 0x00008000 */
#define GPIO_MODER_MODE8_Pos           (16U)
#define GPIO_MODER_MODE8_Msk           (0x3UL << GPIO_MODER_MODE8_Pos)          /*!< 0x00030000 */
#define GPIO_MODER_MODE8               GPIO_MODER_MODE8_Msk
#define GPIO_MODER_MODE8_0             (0x1UL << GPIO_MODER_MODE8_Pos)          /*!< 0x00010000 */
#define GPIO_MODER_MODE8_1             (0x2UL << GPIO_MODER_MODE8_Pos)          /*!< 0x00020000 */
#define GPIO_MODER_MODE9_Pos           (18U)
#define GPIO_MODER_MODE9_Msk           (0x3UL << GPIO_MODER_MODE9_Pos)          /*!< 0x000C0000 */
#define GPIO_MODER_MODE9               GPIO_MODER_MODE9_Msk
#define GPIO_MODER_MODE9_0             (0x1UL << GPIO_MODER_MODE9_Pos)          /*!< 0x00040000 */
#define GPIO_MODER_MODE9_1             (0x2UL << GPIO_MODER_MODE9_Pos)          /*!< 0x00080000 */
#define GPIO_MODER_MODE10_Pos          (20U)
#define GPIO_MODER_MODE10_Msk          (0x3UL << GPIO_MODER_MODE10_Pos)         /*!< 0x00300000 */
#define GPIO_MODER_MODE10              GPIO_MODER_MODE10_Msk
#define GPIO_MODER_MODE10_0            (0x1UL << GPIO_MODER_MODE10_Pos)         /*!< 0x00100000 */
#define GPIO_MODER_MODE10_1            (0x2UL << GPIO_MODER_MODE10_Pos)         /*!< 0x00200000 */
#define GPIO_MODER_MODE11_Pos          (22U)
#define GPIO_MODER_MODE11_Msk          (0x3UL << GPIO_MODER_MODE11_Pos)         /*!< 0x00C00000 */
#define GPIO_MODER_MODE11              GPIO_MODER_MODE11_Msk
#define GPIO_MODER_MODE11_0            (0x1UL << GPIO_MODER_MODE11_Pos)         /*!< 0x00400000 */
#define GPIO_MODER_MODE11_1            (0x2UL << GPIO_MODER_MODE11_Pos)         /*!< 0x00800000 */
#define GPIO_MODER_MODE12_Pos          (24U)
#define GPIO_MODER_MODE12_Msk          (0x3UL << GPIO_MODER_MODE12_Pos)         /*!< 0x03000000 */
#define GPIO_MODER_MODE12              GPIO_MODER_MODE12_Msk
#define GPIO_MODER_MODE12_0            (0x1UL << GPIO_MODER_MODE12_Pos)         /*!< 0x01000000 */
#define GPIO_MODER_MODE12_1            (0x2UL << GPIO_MODER_MODE12_Pos)         /*!< 0x02000000 */
#define GPIO_MODER_MODE13_Pos          (26U)
#define GPIO_MODER_MODE13_Msk          (0x3UL << GPIO_MODER_MODE13_Pos)         /*!< 0x0C000000 */
#define GPIO_MODER_MODE13              GPIO_MODER_MODE13_Msk
#define GPIO_MODER_MODE13_0            (0x1UL << GPIO_MODER_MODE13_Pos)         /*!< 0x04000000 */
#define GPIO_MODER_MODE13_1            (0x2UL << GPIO_MODER_MODE13_Pos)         /*!< 0x08000000 */
#define GPIO_MODER_MODE14_Pos          (28U)
#define GPIO_MODER_MODE14_Msk          (0x3UL << GPIO_MODER_MODE14_Pos)         /*!< 0x30000000 */
#define GPIO_MODER_MODE14              GPIO_MODER_MODE14_Msk
#define GPIO_MODER_MODE14_0            (0x1UL << GPIO_MODER_MODE14_Pos)         /*!< 0x10000000 */
#define GPIO_MODER_MODE14_1            (0x2UL << GPIO_MODER_MODE14_Pos)         /*!< 0x20000000 */
#define GPIO_MODER_MODE15_Pos          (30U)
#define GPIO_MODER_MODE15_Msk          (0x3UL << GPIO_MODER_MODE15_Pos)         /*!< 0xC0000000 */
#define GPIO_MODER_MODE15              GPIO_MODER_MODE15_Msk
#define GPIO_MODER_MODE15_0            (0x1UL << GPIO_MODER_MODE15_Pos)         /*!< 0x40000000 */
#define GPIO_MODER_MODE15_1            (0x2UL << GPIO_MODER_MODE15_Pos)         /*!< 0x80000000 */

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos            (0U)
#define GPIO_OTYPER_OT0_Msk            (0x1UL << GPIO_OTYPER_OT0_Pos)           /*!< 0x00000001 */
#define GPIO_OTYPER_OT0                GPIO_OTYPER_OT0_Msk
#define GPIO_OTYPER_OT1_Pos            (1U)
#define GPIO_OTYPER_OT1_Msk            (0x1UL << GPIO_OTYPER_OT1_Pos)           /*!< 0x00000002 */
#define GPIO_OTYPER_OT1                GPIO_OTYPER_OT1_Msk
#define GPIO_OTYPER_OT2_Pos            (2U)
#define GPIO_OTYPER_OT2_Msk            (0x1UL << GPIO_OTYPER_OT2_Pos)           /*!< 0x00000004 */
#define GPIO_OTYPER_OT2                GPIO_OTYPER_OT2_Msk
#define GPIO_OTYPER_OT3_Pos            (3U)
#define GPIO_OTYPER_OT3_Msk            (0x1UL << GPIO_OTYPER_OT3_Pos)           /*!< 0x00000008 */
#define GPIO_OTYPER_OT3                GPIO_OTYPER_OT3_Msk
#define GPIO_OTYPER_OT4_Pos            (4U)
#define GPIO_OTYPER_OT4_Msk            (0x1UL << GPIO_OTYPER_OT4_Pos)           /*!< 0x00000010 */
#define GPIO_OTYPER_OT4                GPIO_OTYPER_OT4_Msk
#define GPIO_OTYPER_OT5_Pos            (5U)
#define GPIO_OTYPER_OT5_Msk            (0x1UL << GPIO_OTYPER_OT5_Pos)           /*!< 0x00000020 */
#define GPIO_OTYPER_OT5                GPIO_OTYPER_OT5_Msk
#define GPIO_OTYPER_OT6_Pos            (6U)
#define GPIO_OTYPER_OT6_Msk            (0x1UL << GPIO_OTYPER_OT6_Pos)           /*!< 0x00000040 */
#define GPIO_OTYPER_OT6                GPIO_OTYPER_OT6_Msk
#define GPIO_OTYPER_OT7_Pos            (7U)
#define GPIO_OTYPER_OT7_Msk            (0x1UL << GPIO_OTYPER_OT7_Pos)           /*!< 0x00000080 */
#define GPIO_OTYPER_OT7                GPIO_OTYPER_OT7_Msk
#define GPIO_OTYPER_OT8_Pos            (8U)
#define GPIO_OTYPER_OT8_Msk            (0x1UL << GPIO_OTYPER_OT8_Pos)           /*!< 0x00000100 */
#define GPIO_OTYPER_OT8                GPIO_OTYPER_OT8_Msk
#define GPIO_OTYPER_OT9_Pos            (9U)
#define GPIO_OTYPER_OT9_Msk            (0x1UL << GPIO_OTYPER_OT9_Pos)           /*!< 0x00000200 */
#define GPIO_OTYPER_OT9                GPIO_OTYPER_OT9_Msk
#define GPIO_OTYPER_OT10_Pos           (10U)
#define GPIO_OTYPER_OT10_Msk           (0x1UL << GPIO_OTYPER_OT10_Pos)          /*!< 0x00000400 */
#define GPIO_OTYPER_OT10               GPIO_OTYPER_OT10_Msk
#define GPIO_OTYPER_OT11_Pos           (11U)
#define GPIO_OTYPER_OT11_Msk           (0x1UL << GPIO_OTYPER_OT11_Pos)          /*!< 0x00000800 */
#define GPIO_OTYPER_OT11               GPIO_OTYPER_OT11_Msk
#define GPIO_OTYPER_OT12_Pos           (12U)
#define GPIO_OTYPER_OT12_Msk           (0x1UL << GPIO_OTYPER_OT12_Pos)          /*!< 0x00001000 */
#define GPIO_OTYPER_OT12               GPIO_OTYPER_OT12_Msk
#define GPIO_OTYPER_OT13_Pos           (13U)
#define GPIO_OTYPER_OT13_Msk           (0x1UL << GPIO_OTYPER_OT13_Pos)          /*!< 0x00002000 */
#define GPIO_OTYPER_OT13               GPIO_OTYPER_OT13_Msk
#define GPIO_OTYPER_OT14_Pos           (14U)
#define GPIO_OTYPER_OT14_Msk           (0x1UL << GPIO_OTYPER_OT14_Pos)          /*!< 0x00004000 */
#define GPIO_OTYPER_OT14               GPIO_OTYPER_OT14_Msk
#define GPIO_OTYPER_OT15_Pos           (15U)
#define GPIO_OTYPER_OT15_Msk           (0x1UL << GPIO_OTYPER_OT15_Pos)          /*!< 0x00008000 */
#define GPIO_OTYPER_OT15               GPIO_OTYPER_OT15_Msk

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos       (0U)
#define GPIO_OSPEEDR_OSPEED0_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0           GPIO_OSPEEDR_OSPEED0_Msk
#define GPIO_OSPEEDR_OSPEED0_0         (0x1UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEED0_1         (0x2UL << GPIO_OSPEEDR_OSPEED0_Pos)      /*!< 0x00000002 */
#define GPIO_OSPEEDR_OSPEED1_Pos       (2U)
#define GPIO_OSPEEDR_OSPEED1_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED1_Pos)      /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEED1           GPIO_OSPEEDR_OSPEED1_Msk
#define GPIO_OSPEEDR_OSPEED1_0         (0x1UL << GPIO_OSPEEDR_OSPEED1_Pos)      /*!< 0x00000004 */
#define GPIO_OSPEEDR_OSPEED1_1         (0x2UL << GPIO_OSPEEDR_OSPEED1_Pos)      /*!< 0x00000008 */
#define GPIO_OSPEEDR_OSPEED2_Pos       (4U)
#define GPIO_OSPEEDR_OSPEED2_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED2_Pos)      /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEED2           GPIO_OSPEEDR_OSPEED2_Msk
#define GPIO_OSPEEDR_OSPEED2_0         (0x1UL << GPIO_OSPEEDR_OSPEED2_Pos)      /*!< 0x00000010 */
#define GPIO_OSPEEDR_OSPEED2_1         (0x2UL << GPIO_OSPEEDR_OSPEED2_Pos)      /*!< 0x00000020 */
#define GPIO_OSPEEDR_OSPEED3_Pos       (6U)
#define GPIO_OSPEEDR_OSPEED3_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED3_Pos)      /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEED3           GPIO_OSPEEDR_OSPEED3_Msk
#define GPIO_OSPEEDR_OSPEED3_0         (0x1UL << GPIO_OSPEEDR_OSPEED3_Pos)      /*!< 0x00000040 */
#define GPIO_OSPEEDR_OSPEED3_1         (0x2UL << GPIO_OSPEEDR_OSPEED3_Pos)      /*!< 0x00000080 */
#define GPIO_OSPEEDR_OSPEED4_Pos       (8U)
#define GPIO_OSPEEDR_OSPEED4_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED4_Pos)      /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEED4           GPIO_OSPEEDR_OSPEED4_Msk
#define GPIO_OSPEEDR_OSPEED4_0         (0x1UL << GPIO_OSPEEDR_OSPEED4_Pos)      /*!< 0x00000100 */
#define GPIO_OSPEEDR_OSPEED4_1         (0x2UL << GPIO_OSPEEDR_OSPEED4_Pos)      /*!< 0x00000200 */
#define GPIO_OSPEEDR_OSPEED5_Pos       (10U)
#define GPIO_OSPEEDR_OSPEED5_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED5_Pos)      /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEED5           GPIO_OSPEEDR_OSPEED5_Msk
#define GPIO_OSPEEDR_OSPEED5_0         (0x1UL << GPIO_OSPEEDR_OSPEED5_Pos)      /*!< 0x00000400 */
#define GPIO_OSPEEDR_OSPEED5_1         (0x2UL << GPIO_OSPEEDR_OSPEED5_Pos)      /*!< 0x00000800 */
#define GPIO_OSPEEDR_OSPEED6_Pos       (12U)
#define GPIO_OSPEEDR_OSPEED6_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED6_Pos)      /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEED6           GPIO_OSPEEDR_OSPEED6_Msk
#define GPIO_OSPEEDR_OSPEED6_0         (0x1UL << GPIO_OSPEEDR_OSPEED6_Pos)      /*!< 0x00001000 */
#define GPIO_OSPEEDR_OSPEED6_1         (0x2UL << GPIO_OSPEEDR_OSPEED6_Pos)      /*!< 0x00002000 */
#define GPIO_OSPEEDR_OSPEED7_Pos       (14U)
#define GPIO_OSPEEDR_OSPEED7_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED7_Pos)      /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEED7           GPIO_OSPEEDR_OSPEED7_Msk
#define GPIO_OSPEEDR_OSPEED7_0         (0x1UL << GPIO_OSPEEDR_OSPEED7_Pos)      /*!< 0x00004000 */
#define GPIO_OSPEEDR_OSPEED7_1         (0x2UL << GPIO_OSPEEDR_OSPEED7_Pos)      /*!< 0x00008000 */
#define GPIO_OSPEEDR_OSPEED8_Pos       (16U)
#define GPIO_OSPEEDR_OSPEED8_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED8_Pos)      /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEED8           GPIO_OSPEEDR_OSPEED8_Msk
#define GPIO_OSPEEDR_OSPEED8_0         (0x1UL << GPIO_OSPEEDR_OSPEED8_Pos)      /*!< 0x00010000 */
#define GPIO_OSPEEDR_OSPEED8_1         (0x2UL << GPIO_OSPEEDR_OSPEED8_Pos)      /*!< 0x00020000 */
#define GPIO_OSPEEDR_OSPEED9_Pos       (18U)
#define GPIO_OSPEEDR_OSPEED9_Msk       (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos)      /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEED9           GPIO_OSPEEDR_OSPEED9_Msk
#define GPIO_OSPEEDR_OSPEED9_0         (0x1UL << GPIO_OSPEEDR_OSPEED9_Pos)      /*!< 0x00040000 */
#define GPIO_OSPEEDR_OSPEED9_1         (0x2UL << GPIO_OSPEEDR_OSPEED9_Pos)      /*!< 0x00080000 */
#define GPIO_OSPEEDR_OSPEED10_Pos      (20U)
#define GPIO_OSPEEDR_OSPEED10_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED10_Pos)     /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEED10          GPIO_OSPEEDR_OSPEED10_Msk
#define GPIO_OSPEEDR_OSPEED10_0        (0x1UL << GPIO_OSPEEDR_OSPEED10_Pos)     /*!< 0x00100000 */
#define GPIO_OSPEEDR_OSPEED10_1        (0x2UL << GPIO_OSPEEDR_OSPEED10_Pos)     /*!< 0x00200000 */
#define GPIO_OSPEEDR_OSPEED11_Pos      (22U)
#define GPIO_OSPEEDR_OSPEED11_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos)     /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEED11          GPIO_OSPEEDR_OSPEED11_Msk
#define GPIO_OSPEEDR_OSPEED11_0        (0x1UL << GPIO_OSPEEDR_OSPEED11_Pos)     /*!< 0x00400000 */
#define GPIO_OSPEEDR_OSPEED11_1        (0x2UL << GPIO_OSPEEDR_OSPEED11_Pos)     /*!< 0x00800000 */
#define GPIO_OSPEEDR_OSPEED12_Pos      (24U)
#define GPIO_OSPEEDR_OSPEED12_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos)     /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEED12          GPIO_OSPEEDR_OSPEED12_Msk
#define GPIO_OSPEEDR_OSPEED12_0        (0x1UL << GPIO_OSPEEDR_OSPEED12_Pos)     /*!< 0x01000000 */
#define GPIO_OSPEEDR_OSPEED12_1        (0x2UL << GPIO_OSPEEDR_OSPEED12_Pos)     /*!< 0x02000000 */
#define GPIO_OSPEEDR_OSPEED13_Pos      (26U)
#define GPIO_OSPEEDR_OSPEED13_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos)     /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEED13          GPIO_OSPEEDR_OSPEED13_Msk
#define GPIO_OSPEEDR_OSPEED13_0        (0x1UL << GPIO_OSPEEDR_OSPEED13_Pos)     /*!< 0x04000000 */
#define GPIO_OSPEEDR_OSPEED13_1        (0x2UL << GPIO_OSPEEDR_OSPEED13_Pos)     /*!< 0x08000000 */
#define GPIO_OSPEEDR_OSPEED14_Pos      (28U)
#define GPIO_OSPEEDR_OSPEED14_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED14_Pos)     /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEED14          GPIO_OSPEEDR_OSPEED14_Msk
#define GPIO_OSPEEDR_OSPEED14_0        (0x1UL << GPIO_OSPEEDR_OSPEED14_Pos)     /*!< 0x10000000 */
#define GPIO_OSPEEDR_OSPEED14_1        (0x2UL << GPIO_OSPEEDR_OSPEED14_Pos)     /*!< 0x20000000 */
#define GPIO_OSPEEDR_OSPEED15_Pos      (30U)
#define GPIO_OSPEEDR_OSPEED15_Msk      (0x3UL << GPIO_OSPEEDR_OSPEED15_Pos)     /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEED15          GPIO_OSPEEDR_OSPEED15_Msk
#define GPIO_OSPEEDR_OSPEED15_0        (0x1UL << GPIO_OSPEEDR_OSPEED15_Pos)     /*!< 0x40000000 */
#define GPIO_OSPEEDR_OSPEED15_1        (0x2UL << GPIO_OSPEEDR_OSPEED15_Pos)     /*!< 0x80000000 */

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos           (0U)
#define GPIO_PUPDR_PUPD0_Msk           (0x3UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0               GPIO_PUPDR_PUPD0_Msk
#define GPIO_PUPDR_PUPD0_0             (0x1UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000001 */
#define GPIO_PUPDR_PUPD0_1             (0x2UL << GPIO_PUPDR_PUPD0_Pos)          /*!< 0x00000002 */
#define GPIO_PUPDR_PUPD1_Pos           (2U)
#define GPIO_PUPDR_PUPD1_Msk           (0x3UL << GPIO_PUPDR_PUPD1_Pos)          /*!< 0x0000000C */
#define GPIO_PUPDR_PUPD1               GPIO_PUPDR_PUPD1_Msk
#define GPIO_PUPDR_PUPD1_0             (0x1UL << GPIO_PUPDR_PUPD1_Pos)          /*!< 0x00000004 */
#define GPIO_PUPDR_PUPD1_1             (0x2UL << GPIO_PUPDR_PUPD1_Pos)          /*!< 0x00000008 */
#define GPIO_PUPDR_PUPD2_Pos           (4U)
#define GPIO_PUPDR_PUPD2_Msk           (0x3UL << GPIO_PUPDR_PUPD2_Pos)          /*!< 0x00000030 */
#define GPIO_PUPDR_PUPD2               GPIO_PUPDR_PUPD2_Msk
#define GPIO_PUPDR_PUPD2_0             (0x1UL << GPIO_PUPDR_PUPD2_Pos)          /*!< 0x00000010 */
#define GPIO_PUPDR_PUPD2_1             (0x2UL << GPIO_PUPDR_PUPD2_Pos)          /*!< 0x00000020 */
#define GPIO_PUPDR_PUPD3_Pos           (6U)
#define GPIO_PUPDR_PUPD3_Msk           (0x3UL << GPIO_PUPDR_PUPD3_Pos)          /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPD3               GPIO_PUPDR_PUPD3_Msk
#define GPIO_PUPDR_PUPD3_0             (0x1UL << GPIO_PUPDR_PUPD3_Pos)          /*!< 0x00000040 */
#define GPIO_PUPDR_PUPD3_1             (0x2UL << GPIO_PUPDR_PUPD3_Pos)          /*!< 0x00000080 */
#define GPIO_PUPDR_PUPD4_Pos           (8U)
#define GPIO_PUPDR_PUPD4_Msk           (0x3UL << GPIO_PUPDR_PUPD4_Pos)          /*!< 0x00000300 */
#define GPIO_PUPDR_PUPD4               GPIO_PUPDR_PUPD4_Msk
#define GPIO_PUPDR_PUPD4_0             (0x1UL << GPIO_PUPDR_PUPD4_Pos)          /*!< 0x00000100 */
#define GPIO_PUPDR_PUPD4_1             (0x2UL << GPIO_PUPDR_PUPD4_Pos)          /*!< 0x00000200 */
#define GPIO_PUPDR_PUPD5_Pos           (10U)
#define GPIO_PUPDR_PUPD5_Msk           (0x3UL << GPIO_PUPDR_PUPD5_Pos)          /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPD5               GPIO_PUPDR_PUPD5_Msk
#define GPIO_PUPDR_PUPD5_0             (0x1UL << GPIO_PUPDR_PUPD5_Pos)          /*!< 0x00000400 */
#define GPIO_PUPDR_PUPD5_1             (0x2UL << GPIO_PUPDR_PUPD5_Pos)          /*!< 0x00000800 */
#define GPIO_PUPDR_PUPD6_Pos           (12U)
#define GPIO_PUPDR_PUPD6_Msk           (0x3UL << GPIO_PUPDR_PUPD6_Pos)          /*!< 0x00003000 */
#define GPIO_PUPDR_PUPD6               GPIO_PUPDR_PUPD6_Msk
#define GPIO_PUPDR_PUPD6_0             (0x1UL << GPIO_PUPDR_PUPD6_Pos)          /*!< 0x00001000 */
#define GPIO_PUPDR_PUPD6_1             (0x2UL << GPIO_PUPDR_PUPD6_Pos)          /*!< 0x00002000 */
#define GPIO_PUPDR_PUPD7_Pos           (14U)
#define GPIO_PUPDR_PUPD7_Msk           (0x3UL << GPIO_PUPDR_PUPD7_Pos)          /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPD7               GPIO_PUPDR_PUPD7_Msk
#define GPIO_PUPDR_PUPD7_0             (0x1UL << GPIO_PUPDR_PUPD7_Pos)          /*!< 0x00004000 */
#define GPIO_PUPDR_PUPD7_1             (0x2UL << GPIO_PUPDR_PUPD7_Pos)          /*!< 0x00008000 */
#define GPIO_PUPDR_PUPD8_Pos           (16U)
#define GPIO_PUPDR_PUPD8_Msk           (0x3UL << GPIO_PUPDR_PUPD8_Pos)          /*!< 0x00030000 */
#define GPIO_PUPDR_PUPD8               GPIO_PUPDR_PUPD8_Msk
#define GPIO_PUPDR_PUPD8_0             (0x1UL << GPIO_PUPDR_PUPD8_Pos)          /*!< 0x00010000 */
#define GPIO_PUPDR_PUPD8_1             (0x2UL << GPIO_PUPDR_PUPD8_Pos)          /*!< 0x00020000 */
#define GPIO_PUPDR_PUPD9_Pos           (18U)
#define GPIO_PUPDR_PUPD9_Msk           (0x3UL << GPIO_PUPDR_PUPD9_Pos)          /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPD9               GPIO_PUPDR_PUPD9_Msk
#define GPIO_PUPDR_PUPD9_0             (0x1UL << GPIO_PUPDR_PUPD9_Pos)          /*!< 0x00040000 */
#define GPIO_PUPDR_PUPD9_1             (0x2UL << GPIO_PUPDR_PUPD9_Pos)          /*!< 0x00080000 */
#define GPIO_PUPDR_PUPD10_Pos          (20U)
#define GPIO_PUPDR_PUPD10_Msk          (0x3UL << GPIO_PUPDR_PUPD10_Pos)         /*!< 0x00300000 */
#define GPIO_PUPDR_PUPD10              GPIO_PUPDR_PUPD10_Msk
#define GPIO_PUPDR_PUPD10_0            (0x1UL << GPIO_PUPDR_PUPD10_Pos)         /*!< 0x00100000 */
#define GPIO_PUPDR_PUPD10_1            (0x2UL << GPIO_PUPDR_PUPD10_Pos)         /*!< 0x00200000 */
#define GPIO_PUPDR_PUPD11_Pos          (22U)
#define GPIO_PUPDR_PUPD11_Msk          (0x3UL << GPIO_PUPDR_PUPD11_Pos)         /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPD11              GPIO_PUPDR_PUPD11_Msk
#define GPIO_PUPDR_PUPD11_0            (0x1UL << GPIO_PUPDR_PUPD11_Pos)         /*!< 0x00400000 */
#define GPIO_PUPDR_PUPD11_1            (0x2UL << GPIO_PUPDR_PUPD11_Pos)         /*!< 0x00800000 */
#define GPIO_PUPDR_PUPD12_Pos          (24U)
#define GPIO_PUPDR_PUPD12_Msk          (0x3UL << GPIO_PUPDR_PUPD12_Pos)         /*!< 0x03000000 */
#define GPIO_PUPDR_PUPD12              GPIO_PUPDR_PUPD12_Msk
#define GPIO_PUPDR_PUPD12_0            (0x1UL << GPIO_PUPDR_PUPD12_Pos)         /*!< 0x01000000 */
#define GPIO_PUPDR_PUPD12_1            (0x2UL << GPIO_PUPDR_PUPD12_Pos)         /*!< 0x02000000 */
#define GPIO_PUPDR_PUPD13_Pos          (26U)
#define GPIO_PUPDR_PUPD13_Msk          (0x3UL << GPIO_PUPDR_PUPD13_Pos)         /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPD13              GPIO_PUPDR_PUPD13_Msk
#define GPIO_PUPDR_PUPD13_0            (0x1UL << GPIO_PUPDR_PUPD13_Pos)         /*!< 0x04000000 */
#define GPIO_PUPDR_PUPD13_1            (0x2UL << GPIO_PUPDR_PUPD13_Pos)         /*!< 0x08000000 */
#define GPIO_PUPDR_PUPD14_Pos          (28U)
#define GPIO_PUPDR_PUPD14_Msk          (0x3UL << GPIO_PUPDR_PUPD14_Pos)         /*!< 0x30000000 */
#define GPIO_PUPDR_PUPD14              GPIO_PUPDR_PUPD14_Msk
#define GPIO_PUPDR_PUPD14_0            (0x1UL << GPIO_PUPDR_PUPD14_Pos)         /*!< 0x10000000 */
#define GPIO_PUPDR_PUPD14_1            (0x2UL << GPIO_PUPDR_PUPD14_Pos)         /*!< 0x20000000 */
#define GPIO_PUPDR_PUPD15_Pos          (30U)
#define GPIO_PUPDR_PUPD15_Msk          (0x3UL << GPIO_PUPDR_PUPD15_Pos)         /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPD15              GPIO_PUPDR_PUPD15_Msk
#define GPIO_PUPDR_PUPD15_0            (0x1UL << GPIO_PUPDR_PUPD15_Pos)         /*!< 0x40000000 */
#define GPIO_PUPDR_PUPD15_1            (0x2UL << GPIO_PUPDR_PUPD15_Pos)         /*!< 0x80000000 */

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos               (0U)
#define GPIO_IDR_ID0_Msk               (0x1UL << GPIO_IDR_ID0_Pos)              /*!< 0x00000001 */
#define GPIO_IDR_ID0                   GPIO_IDR_ID0_Msk
#define GPIO_IDR_ID1_Pos               (1U)
#define GPIO_IDR_ID1_Msk               (0x1UL << GPIO_IDR_ID1_Pos)              /*!< 0x00000002 */
#define GPIO_IDR_ID1                   GPIO_IDR_ID1_Msk
#define GPIO_IDR_ID2_Pos               (2U)
#define GPIO_IDR_ID2_Msk               (0x1UL << GPIO_IDR_ID2_Pos)              /*!< 0x00000004 */
#define GPIO_IDR_ID2                   GPIO_IDR_ID2_Msk
#define GPIO_IDR_ID3_Pos               (3U)
#define GPIO_IDR_ID3_Msk               (0x1UL << GPIO_IDR_ID3_Pos)              /*!< 0x00000008 */
#define GPIO_IDR_ID3                   GPIO_IDR_ID3_Msk
#define GPIO_IDR_ID4_Pos               (4U)
#define GPIO_IDR_ID4_Msk               (0x1UL << GPIO_IDR_ID4_Pos)              /*!< 0x00000010 */
#define GPIO_IDR_ID4                   GPIO_IDR_ID4_Msk
#define GPIO_IDR_ID5_Pos               (5U)
#define GPIO_IDR_ID5_Msk               (0x1UL << GPIO_IDR_ID5_Pos)              /*!< 0x00000020 */
#define GPIO_IDR_ID5                   GPIO_IDR_ID5_Msk
#define GPIO_IDR_ID6_Pos               (6U)
#define GPIO_IDR_ID6_Msk               (0x1UL << GPIO_IDR_ID6_Pos)              /*!< 0x00000040 */
#define GPIO_IDR_ID6                   GPIO_IDR_ID6_Msk
#define GPIO_IDR_ID7_Pos               (7U)
#define GPIO_IDR_ID7_Msk               (0x1UL << GPIO_IDR_ID7_Pos)              /*!< 0x00000080 */
#define GPIO_IDR_ID7                   GPIO_IDR_ID7_Msk
#define GPIO_IDR_ID8_Pos               (8U)
#define GPIO_IDR_ID8_Msk               (0x1UL << GPIO_IDR_ID8_Pos)              /*!< 0x00000100 */
#define GPIO_IDR_ID8                   GPIO_IDR_ID8_Msk
#define GPIO_IDR_ID9_Pos               (9U)
#define GPIO_IDR_ID9_Msk               (0x1UL << GPIO_IDR_ID9_Pos)              /*!< 0x00000200 */
#define GPIO_IDR_ID9                   GPIO_IDR_ID9_Msk
#define GPIO_IDR_ID10_Pos              (10U)
#define GPIO_IDR_ID10_Msk              (0x1UL << GPIO_IDR_ID10_Pos)             /*!< 0x00000400 */
#define GPIO_IDR_ID10                  GPIO_IDR_ID10_Msk
#define GPIO_IDR_ID11_Pos              (11U)
#define GPIO_IDR_ID11_Msk              (0x1UL << GPIO_IDR_ID11_Pos)             /*!< 0x00000800 */
#define GPIO_IDR_ID11                  GPIO_IDR_ID11_Msk
#define GPIO_IDR_ID12_Pos              (12U)
#define GPIO_IDR_ID12_Msk              (0x1UL << GPIO_IDR_ID12_Pos)             /*!< 0x00001000 */
#define GPIO_IDR_ID12                  GPIO_IDR_ID12_Msk
#define GPIO_IDR_ID13_Pos              (13U)
#define GPIO_IDR_ID13_Msk              (0x1UL << GPIO_IDR_ID13_Pos)             /*!< 0x00002000 */
#define GPIO_IDR_ID13                  GPIO_IDR_ID13_Msk
#define GPIO_IDR_ID14_Pos              (14U)
#define GPIO_IDR_ID14_Msk              (0x1UL << GPIO_IDR_ID14_Pos)             /*!< 0x00004000 */
#define GPIO_IDR_ID14                  GPIO_IDR_ID14_Msk
#define GPIO_IDR_ID15_Pos              (15U)
#define GPIO_IDR_ID15_Msk              (0x1UL << GPIO_IDR_ID15_Pos)             /*!< 0x00008000 */
#define GPIO_IDR_ID15                  GPIO_IDR_ID15_Msk

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos               (0U)
#define GPIO_ODR_OD0_Msk               (0x1UL << GPIO_ODR_OD0_Pos)              /*!< 0x00000001 */
#define GPIO_ODR_OD0                   GPIO_ODR_OD0_Msk
#define GPIO_ODR_OD1_Pos               (1U)
#define GPIO_ODR_OD1_Msk               (0x1UL << GPIO_ODR_OD1_Pos)              /*!< 0x00000002 */
#define GPIO_ODR_OD1                   GPIO_ODR_OD1_Msk
#define GPIO_ODR_OD2_Pos               (2U)
#define GPIO_ODR_OD2_Msk               (0x1UL << GPIO_ODR_OD2_Pos)              /*!< 0x00000004 */
#define GPIO_ODR_OD2                   GPIO_ODR_OD2_Msk
#define GPIO_ODR_OD3_Pos               (3U)
#define GPIO_ODR_OD3_Msk               (0x1UL << GPIO_ODR_OD3_Pos)              /*!< 0x00000008 */
#define GPIO_ODR_OD3                   GPIO_ODR_OD3_Msk
#define GPIO_ODR_OD4_Pos               (4U)
#define GPIO_ODR_OD4_Msk               (0x1UL << GPIO_ODR_OD4_Pos)              /*!< 0x00000010 */
#define GPIO_ODR_OD4                   GPIO_ODR_OD4_Msk
#define GPIO_ODR_OD5_Pos               (5U)
#define GPIO_ODR_OD5_Msk               (0x1UL << GPIO_ODR_OD5_Pos)              /*!< 0x00000020 */
#define GPIO_ODR_OD5                   GPIO_ODR_OD5_Msk
#define GPIO_ODR_OD6_Pos               (6U)
#define GPIO_ODR_OD6_Msk               (0x1UL << GPIO_ODR_OD6_Pos)              /*!< 0x00000040 */
#define GPIO_ODR_OD6                   GPIO_ODR_OD6_Msk
#define GPIO_ODR_OD7_Pos               (7U)
#define GPIO_ODR_OD7_Msk               (0x1UL << GPIO_ODR_OD7_Pos)              /*!< 0x00000080 */
#define GPIO_ODR_OD7                   GPIO_ODR_OD7_Msk
#define GPIO_ODR_OD8_Pos               (8U)
#define GPIO_ODR_OD8_Msk               (0x1UL << GPIO_ODR_OD8_Pos)              /*!< 0x00000100 */
#define GPIO_ODR_OD8                   GPIO_ODR_OD8_Msk
#define GPIO_ODR_OD9_Pos               (9U)
#define GPIO_ODR_OD9_Msk               (0x1UL << GPIO_ODR_OD9_Pos)              /*!< 0x00000200 */
#define GPIO_ODR_OD9                   GPIO_ODR_OD9_Msk
#define GPIO_ODR_OD10_Pos              (10U)
#define GPIO_ODR_OD10_Msk              (0x1UL << GPIO_ODR_OD10_Pos)             /*!< 0x00000400 */
#define GPIO_ODR_OD10                  GPIO_ODR_OD10_Msk
#define GPIO_ODR_OD11_Pos              (11U)
#define GPIO_ODR_OD11_Msk              (0x1UL << GPIO_ODR_OD11_Pos)             /*!< 0x00000800 */
#define GPIO_ODR_OD11                  GPIO_ODR_OD11_Msk
#define GPIO_ODR_OD12_Pos              (12U)
#define GPIO_ODR_OD12_Msk              (0x1UL << GPIO_ODR_OD12_Pos)             /*!< 0x00001000 */
#define GPIO_ODR_OD12                  GPIO_ODR_OD12_Msk
#define GPIO_ODR_OD13_Pos              (13U)
#define GPIO_ODR_OD13_Msk              (0x1UL << GPIO_ODR_OD13_Pos)             /*!< 0x00002000 */
#define GPIO_ODR_OD13                  GPIO_ODR_OD13_Msk
#define GPIO_ODR_OD14_Pos              (14U)
#define GPIO_ODR_OD14_Msk              (0x1UL << GPIO_ODR_OD14_Pos)             /*!< 0x00004000 */
#define GPIO_ODR_OD14                  GPIO_ODR_OD14_Msk
#define GPIO_ODR_OD15_Pos              (15U)
#define GPIO_ODR_OD15_Msk              (0x1UL << GPIO_ODR_OD15_Pos)             /*!< 0x00008000 */
#define GPIO_ODR_OD15                  GPIO_ODR_OD15_Msk

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos              (0U)
#define GPIO_BSRR_BS0_Msk              (0x1UL << GPIO_BSRR_BS0_Pos)             /*!< 0x00000001 */
#define GPIO_BSRR_BS0                  GPIO_BSRR_BS0_Msk
#define GPIO_BSRR_BS1_Pos              (1U)
#define GPIO_BSRR_BS1_Msk              (0x1UL << GPIO_BSRR_BS1_Pos)             /*!< 0x00000002 */
#define GPIO_BSRR_BS1                  GPIO_BSRR_BS1_Msk
#define GPIO_BSRR_BS2_Pos              (2U)
#define GPIO_BSRR_BS2_Msk              (0x1UL << GPIO_BSRR_BS2_Pos)             /*!< 0x00000004 */
#define GPIO_BSRR_BS2                  GPIO_BSRR_BS2_Msk
#define GPIO_BSRR_BS3_Pos              (3U)
#define GPIO_BSRR_BS3_Msk              (0x1UL << GPIO_BSRR_BS3_Pos)             /*!< 0x00000008 */
#define GPIO_BSRR_BS3                  GPIO_BSRR_BS3_Msk
#define GPIO_BSRR_BS4_Pos              (4U)
#define GPIO_BSRR_BS4_Msk              (0x1UL << GPIO_BSRR_BS4_Pos)             /*!< 0x00000010 */
#define GPIO_BSRR_BS4                  GPIO_BSRR_BS4_Msk
#define GPIO_BSRR_BS5_Pos              (5U)
#define GPIO_BSRR_BS5_Msk              (0x1UL << GPIO_BSRR_BS5_Pos)             /*!< 0x00000020 */
#define GPIO_BSRR_BS5                  GPIO_BSRR_BS5_Msk
#define GPIO_BSRR_BS6_Pos              (6U)
#define GPIO_BSRR_BS6_Msk              (0x1UL << GPIO_BSRR_BS6_Pos)             /*!< 0x00000040 */
#define GPIO_BSRR_BS6                  GPIO_BSRR_BS6_Msk
#define GPIO_BSRR_BS7_Pos              (7U)
#define GPIO_BSRR_BS7_Msk              (0x1UL << GPIO_BSRR_BS7_Pos)             /*!< 0x00000080 */
#define GPIO_BSRR_BS7                  GPIO_BSRR_BS7_Msk
#define GPIO_BSRR_BS8_Pos              (8U)
#define GPIO_BSRR_BS8_Msk              (0x1UL << GPIO_BSRR_BS8_Pos)             /*!< 0x00000100 */
#define GPIO_BSRR_BS8                  GPIO_BSRR_BS8_Msk
#define GPIO_BSRR_BS9_Pos              (9U)
#define GPIO_BSRR_BS9_Msk              (0x1UL << GPIO_BSRR_BS9_Pos)             /*!< 0x00000200 */
#define GPIO_BSRR_BS9                  GPIO_BSRR_BS9_Msk
#define GPIO_BSRR_BS10_Pos             (10U)
#define GPIO_BSRR_BS10_Msk             (0x1UL << GPIO_BSRR_BS10_Pos)            /*!< 0x00000400 */
#define GPIO_BSRR_BS10                 GPIO_BSRR_BS10_Msk
#define GPIO_BSRR_BS11_Pos             (11U)
#define GPIO_BSRR_BS11_Msk             (0x1UL << GPIO_BSRR_BS11_Pos)            /*!< 0x00000800 */
#define GPIO_BSRR_BS11                 GPIO_BSRR_BS11_Msk
#define GPIO_BSRR_BS12_Pos             (12U)
#define GPIO_BSRR_BS12_Msk             (0x1UL << GPIO_BSRR_BS12_Pos)            /*!< 0x00001000 */
#define GPIO_BSRR_BS12                 GPIO_BSRR_BS12_Msk
#define GPIO_BSRR_BS13_Pos             (13U)
#define GPIO_BSRR_BS13_Msk             (0x1UL << GPIO_BSRR_BS13_Pos)            /*!< 0x00002000 */
#define GPIO_BSRR_BS13                 GPIO_BSRR_BS13_Msk
#define GPIO_BSRR_BS14_Pos             (14U)
#define GPIO_BSRR_BS14_Msk             (0x1UL << GPIO_BSRR_BS14_Pos)            /*!< 0x00004000 */
#define GPIO_BSRR_BS14                 GPIO_BSRR_BS14_Msk
#define GPIO_BSRR_BS15_Pos             (15U)
#define GPIO_BSRR_BS15_Msk             (0x1UL << GPIO_BSRR_BS15_Pos)            /*!< 0x00008000 */
#define GPIO_BSRR_BS15                 GPIO_BSRR_BS15_Msk
#define GPIO_BSRR_BR0_Pos              (16U)
#define GPIO_BSRR_BR0_Msk              (0x1UL << GPIO_BSRR_BR0_Pos)             /*!< 0x00010000 */
#define GPIO_BSRR_BR0                  GPIO_BSRR_BR0_Msk
#define GPIO_BSRR_BR1_Pos              (17U)
#define GPIO_BSRR_BR1_Msk              (0x1UL << GPIO_BSRR_BR1_Pos)             /*!< 0x00020000 */
#define GPIO_BSRR_BR1                  GPIO_BSRR_BR1_Msk
#define GPIO_BSRR_BR2_Pos              (18U)
#define GPIO_BSRR_BR2_Msk              (0x1UL << GPIO_BSRR_BR2_Pos)             /*!< 0x00040000 */
#define GPIO_BSRR_BR2                  GPIO_BSRR_BR2_Msk
#define GPIO_BSRR_BR3_Pos              (19U)
#define GPIO_BSRR_BR3_Msk              (0x1UL << GPIO_BSRR_BR3_Pos)             /*!< 0x00080000 */
#define GPIO_BSRR_BR3                  GPIO_BSRR_BR3_Msk
#define GPIO_BSRR_BR4_Pos              (20U)
#define GPIO_BSRR_BR4_Msk              (0x1UL << GPIO_BSRR_BR4_Pos)             /*!< 0x00100000 */
#define GPIO_BSRR_BR4                  GPIO_BSRR_BR4_Msk
#define GPIO_BSRR_BR5_Pos              (21U)
#define GPIO_BSRR_BR5_Msk              (0x1UL << GPIO_BSRR_BR5_Pos)             /*!< 0x00200000 */
#define GPIO_BSRR_BR5                  GPIO_BSRR_BR5_Msk
#define GPIO_BSRR_BR6_Pos              (22U)
#define GPIO_BSRR_BR6_Msk              (0x1UL << GPIO_BSRR_BR6_Pos)             /*!< 0x00400000 */
#define GPIO_BSRR_BR6                  GPIO_BSRR_BR6_Msk
#define GPIO_BSRR_BR7_Pos              (23U)
#define GPIO_BSRR_BR7_Msk              (0x1UL << GPIO_BSRR_BR7_Pos)             /*!< 0x00800000 */
#define GPIO_BSRR_BR7                  GPIO_BSRR_BR7_Msk
#define GPIO_BSRR_BR8_Pos              (24U)
#define GPIO_BSRR_BR8_Msk              (0x1UL << GPIO_BSRR_BR8_Pos)             /*!< 0x01000000 */
#define GPIO_BSRR_BR8                  GPIO_BSRR_BR8_Msk
#define GPIO_BSRR_BR9_Pos              (25U)
#define GPIO_BSRR_BR9_Msk              (0x1UL << GPIO_BSRR_BR9_Pos)             /*!< 0x02000000 */
#define GPIO_BSRR_BR9                  GPIO_BSRR_BR9_Msk
#define GPIO_BSRR_BR10_Pos             (26U)
#define GPIO_BSRR_BR10_Msk             (0x1UL << GPIO_BSRR_BR10_Pos)            /*!< 0x04000000 */
#define GPIO_BSRR_BR10                 GPIO_BSRR_BR10_Msk
#define GPIO_BSRR_BR11_Pos             (27U)
#define GPIO_BSRR_BR11_Msk             (0x1UL << GPIO_BSRR_BR11_Pos)            /*!< 0x08000000 */
#define GPIO_BSRR_BR11                 GPIO_BSRR_BR11_Msk
#define GPIO_BSRR_BR12_Pos             (28U)
#define GPIO_BSRR_BR12_Msk             (0x1UL << GPIO_BSRR_BR12_Pos)            /*!< 0x10000000 */
#define GPIO_BSRR_BR12                 GPIO_BSRR_BR12_Msk
#define GPIO_BSRR_BR13_Pos             (29U)
#define GPIO_BSRR_BR13_Msk             (0x1UL << GPIO_BSRR_BR13_Pos)            /*!< 0x20000000 */
#define GPIO_BSRR_BR13                 GPIO_BSRR_BR13_Msk
#define GPIO_BSRR_BR14_Pos             (30U)
#define GPIO_BSRR_BR14_Msk             (0x1UL << GPIO_BSRR_BR14_Pos)            /*!< 0x40000000 */
#define GPIO_BSRR_BR14                 GPIO_BSRR_BR14_Msk
#define GPIO_BSRR_BR15_Pos             (31U)
#define GPIO_BSRR_BR15_Msk             (0x1UL << GPIO_BSRR_BR15_Pos)            /*!< 0x80000000 */
#define GPIO_BSRR_BR15                 GPIO_BSRR_BR15_Msk

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos             (0U)
#define GPIO_LCKR_LCK0_Msk             (0x1UL << GPIO_LCKR_LCK0_Pos)            /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                 GPIO_LCKR_LCK0_Msk
#define GPIO_LCKR_LCK1_Pos             (1U)
#define GPIO_LCKR_LCK1_Msk             (0x1UL << GPIO_LCKR_LCK1_Pos)            /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                 GPIO_LCKR_LCK1_Msk
#define GPIO_LCKR_LCK2_Pos             (2U)
#define GPIO_LCKR_LCK2_Msk             (0x1UL << GPIO_LCKR_LCK2_Pos)            /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                 GPIO_LCKR_LCK2_Msk
#define GPIO_LCKR_LCK3_Pos             (3U)
#define GPIO_LCKR_LCK3_Msk             (0x1UL << GPIO_LCKR_LCK3_Pos)            /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                 GPIO_LCKR_LCK3_Msk
#define GPIO_LCKR_LCK4_Pos             (4U)
#define GPIO_LCKR_LCK4_Msk             (0x1UL << GPIO_LCKR_LCK4_Pos)            /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                 GPIO_LCKR_LCK4_Msk
#define GPIO_LCKR_LCK5_Pos             (5U)
#define GPIO_LCKR_LCK5_Msk             (0x1UL << GPIO_LCKR_LCK5_Pos)            /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                 GPIO_LCKR_LCK5_Msk
#define GPIO_LCKR_LCK6_Pos             (6U)
#define GPIO_LCKR_LCK6_Msk             (0x1UL << GPIO_LCKR_LCK6_Pos)            /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                 GPIO_LCKR_LCK6_Msk
#define GPIO_LCKR_LCK7_Pos             (7U)
#define GPIO_LCKR_LCK7_Msk             (0x1UL << GPIO_LCKR_LCK7_Pos)            /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                 GPIO_LCKR_LCK7_Msk
#define GPIO_LCKR_LCK8_Pos             (8U)
#define GPIO_LCKR_LCK8_Msk             (0x1UL << GPIO_LCKR_LCK8_Pos)            /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                 GPIO_LCKR_LCK8_Msk
#define GPIO_LCKR_LCK9_Pos             (9U)
#define GPIO_LCKR_LCK9_Msk             (0x1UL << GPIO_LCKR_LCK9_Pos)            /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                 GPIO_LCKR_LCK9_Msk
#define GPIO_LCKR_LCK10_Pos            (10U)
#define GPIO_LCKR_LCK10_Msk            (0x1UL << GPIO_LCKR_LCK10_Pos)           /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                GPIO_LCKR_LCK10_Msk
#define GPIO_LCKR_LCK11_Pos            (11U)
#define GPIO_LCKR_LCK11_Msk            (0x1UL << GPIO_LCKR_LCK11_Pos)           /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                GPIO_LCKR_LCK11_Msk
#define GPIO_LCKR_LCK12_Pos            (12U)
#define GPIO_LCKR_LCK12_Msk            (0x1UL << GPIO_LCKR_LCK12_Pos)           /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                GPIO_LCKR_LCK12_Msk
#define GPIO_LCKR_LCK13_Pos            (13U)
#define GPIO_LCKR_LCK13_Msk            (0x1UL << GPIO_LCKR_LCK13_Pos)           /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                GPIO_LCKR_LCK13_Msk
#define GPIO_LCKR_LCK14_Pos            (14U)
#define GPIO_LCKR_LCK14_Msk            (0x1UL << GPIO_LCKR_LCK14_Pos)           /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                GPIO_LCKR_LCK14_Msk
#define GPIO_LCKR_LCK15_Pos            (15U)
#define GPIO_LCKR_LCK15_Msk            (0x1UL << GPIO_LCKR_LCK15_Pos)           /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                GPIO_LCKR_LCK15_Msk
#define GPIO_LCKR_LCKK_Pos             (16U)
#define GPIO_LCKR_LCKK_Msk             (0x1UL << GPIO_LCKR_LCKK_Pos)            /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                 GPIO_LCKR_LCKK_Msk

/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFSEL0_Pos           (0U)
#define GPIO_AFRL_AFSEL0_Msk           (0x7UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0               GPIO_AFRL_AFSEL0_Msk
#define GPIO_AFRL_AFSEL0_0             (0x1UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x00000001 */
#define GPIO_AFRL_AFSEL0_1             (0x2UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x00000002 */
#define GPIO_AFRL_AFSEL0_2             (0x4UL << GPIO_AFRL_AFSEL0_Pos)          /*!< 0x00000004 */

#define GPIO_AFRL_AFSEL1_Pos           (4U)
#define GPIO_AFRL_AFSEL1_Msk           (0x7UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1               GPIO_AFRL_AFSEL1_Msk
#define GPIO_AFRL_AFSEL1_0             (0x1UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x00000010 */
#define GPIO_AFRL_AFSEL1_1             (0x2UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x00000020 */
#define GPIO_AFRL_AFSEL1_2             (0x4UL << GPIO_AFRL_AFSEL1_Pos)          /*!< 0x00000040 */

#define GPIO_AFRL_AFSEL2_Pos           (8U)
#define GPIO_AFRL_AFSEL2_Msk           (0x7UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2               GPIO_AFRL_AFSEL2_Msk
#define GPIO_AFRL_AFSEL2_0             (0x1UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000100 */
#define GPIO_AFRL_AFSEL2_1             (0x2UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000200 */
#define GPIO_AFRL_AFSEL2_2             (0x4UL << GPIO_AFRL_AFSEL2_Pos)          /*!< 0x00000400 */

#define GPIO_AFRL_AFSEL3_Pos           (12U)
#define GPIO_AFRL_AFSEL3_Msk           (0x7UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3               GPIO_AFRL_AFSEL3_Msk
#define GPIO_AFRL_AFSEL3_0             (0x1UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x00001000 */
#define GPIO_AFRL_AFSEL3_1             (0x2UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x00002000 */
#define GPIO_AFRL_AFSEL3_2             (0x4UL << GPIO_AFRL_AFSEL3_Pos)          /*!< 0x00004000 */

#define GPIO_AFRL_AFSEL4_Pos           (16U)
#define GPIO_AFRL_AFSEL4_Msk           (0x7UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4               GPIO_AFRL_AFSEL4_Msk
#define GPIO_AFRL_AFSEL4_0             (0x1UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x00010000 */
#define GPIO_AFRL_AFSEL4_1             (0x2UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x00020000 */
#define GPIO_AFRL_AFSEL4_2             (0x4UL << GPIO_AFRL_AFSEL4_Pos)          /*!< 0x00040000 */

#define GPIO_AFRL_AFSEL5_Pos           (20U)
#define GPIO_AFRL_AFSEL5_Msk           (0x7UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5               GPIO_AFRL_AFSEL5_Msk
#define GPIO_AFRL_AFSEL5_0             (0x1UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00100000 */
#define GPIO_AFRL_AFSEL5_1             (0x2UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00200000 */
#define GPIO_AFRL_AFSEL5_2             (0x4UL << GPIO_AFRL_AFSEL5_Pos)          /*!< 0x00400000 */

#define GPIO_AFRL_AFSEL6_Pos           (24U)
#define GPIO_AFRL_AFSEL6_Msk           (0x7UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6               GPIO_AFRL_AFSEL6_Msk
#define GPIO_AFRL_AFSEL6_0             (0x1UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x01000000 */
#define GPIO_AFRL_AFSEL6_1             (0x2UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x02000000 */
#define GPIO_AFRL_AFSEL6_2             (0x4UL << GPIO_AFRL_AFSEL6_Pos)          /*!< 0x04000000 */

#define GPIO_AFRL_AFSEL7_Pos           (28U)
#define GPIO_AFRL_AFSEL7_Msk           (0x7UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x70000000 */
#define GPIO_AFRL_AFSEL7               GPIO_AFRL_AFSEL7_Msk
#define GPIO_AFRL_AFSEL7_0             (0x1UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x10000000 */
#define GPIO_AFRL_AFSEL7_1             (0x2UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x20000000 */
#define GPIO_AFRL_AFSEL7_2             (0x4UL << GPIO_AFRL_AFSEL7_Pos)          /*!< 0x40000000 */

/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFSEL8_Pos           (0U)
#define GPIO_AFRH_AFSEL8_Msk           (0x7UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8               GPIO_AFRH_AFSEL8_Msk
#define GPIO_AFRH_AFSEL8_0             (0x1UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x00000001 */
#define GPIO_AFRH_AFSEL8_1             (0x2UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x00000002 */
#define GPIO_AFRH_AFSEL8_2             (0x4UL << GPIO_AFRH_AFSEL8_Pos)          /*!< 0x00000004 */

#define GPIO_AFRH_AFSEL9_Pos           (4U)
#define GPIO_AFRH_AFSEL9_Msk           (0x7UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9               GPIO_AFRH_AFSEL9_Msk
#define GPIO_AFRH_AFSEL9_0             (0x1UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x00000010 */
#define GPIO_AFRH_AFSEL9_1             (0x2UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x00000020 */
#define GPIO_AFRH_AFSEL9_2             (0x4UL << GPIO_AFRH_AFSEL9_Pos)          /*!< 0x00000040 */

#define GPIO_AFRH_AFSEL10_Pos          (8U)
#define GPIO_AFRH_AFSEL10_Msk          (0x7UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10              GPIO_AFRH_AFSEL10_Msk
#define GPIO_AFRH_AFSEL10_0            (0x1UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000100 */
#define GPIO_AFRH_AFSEL10_1            (0x2UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000200 */
#define GPIO_AFRH_AFSEL10_2            (0x4UL << GPIO_AFRH_AFSEL10_Pos)         /*!< 0x00000400 */

#define GPIO_AFRH_AFSEL11_Pos          (12U)
#define GPIO_AFRH_AFSEL11_Msk          (0x7UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11              GPIO_AFRH_AFSEL11_Msk
#define GPIO_AFRH_AFSEL11_0            (0x1UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x00001000 */
#define GPIO_AFRH_AFSEL11_1            (0x2UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x00002000 */
#define GPIO_AFRH_AFSEL11_2            (0x4UL << GPIO_AFRH_AFSEL11_Pos)         /*!< 0x00004000 */

#define GPIO_AFRH_AFSEL12_Pos          (16U)
#define GPIO_AFRH_AFSEL12_Msk          (0x7UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12              GPIO_AFRH_AFSEL12_Msk
#define GPIO_AFRH_AFSEL12_0            (0x1UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x00010000 */
#define GPIO_AFRH_AFSEL12_1            (0x2UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x00020000 */
#define GPIO_AFRH_AFSEL12_2            (0x4UL << GPIO_AFRH_AFSEL12_Pos)         /*!< 0x00040000 */

#define GPIO_AFRH_AFSEL13_Pos          (20U)
#define GPIO_AFRH_AFSEL13_Msk          (0x7UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13              GPIO_AFRH_AFSEL13_Msk
#define GPIO_AFRH_AFSEL13_0            (0x1UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00100000 */
#define GPIO_AFRH_AFSEL13_1            (0x2UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00200000 */
#define GPIO_AFRH_AFSEL13_2            (0x4UL << GPIO_AFRH_AFSEL13_Pos)         /*!< 0x00400000 */

#define GPIO_AFRH_AFSEL14_Pos          (24U)
#define GPIO_AFRH_AFSEL14_Msk          (0x7UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14              GPIO_AFRH_AFSEL14_Msk
#define GPIO_AFRH_AFSEL14_0            (0x1UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x01000000 */
#define GPIO_AFRH_AFSEL14_1            (0x2UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x02000000 */
#define GPIO_AFRH_AFSEL14_2            (0x4UL << GPIO_AFRH_AFSEL14_Pos)         /*!< 0x04000000 */

#define GPIO_AFRH_AFSEL15_Pos          (28U)
#define GPIO_AFRH_AFSEL15_Msk          (0x7UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15              GPIO_AFRH_AFSEL15_Msk
#define GPIO_AFRH_AFSEL15_0            (0x1UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0x10000000 */
#define GPIO_AFRH_AFSEL15_1            (0x2UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0x20000000 */
#define GPIO_AFRH_AFSEL15_2            (0x4UL << GPIO_AFRH_AFSEL15_Pos)         /*!< 0x40000000 */

/******************  Bits definition for GPIO_BRR register  ******************/
#define GPIO_BRR_BR0_Pos               (0U)
#define GPIO_BRR_BR0_Msk               (0x1UL << GPIO_BRR_BR0_Pos)              /*!< 0x00000001 */
#define GPIO_BRR_BR0                   GPIO_BRR_BR0_Msk
#define GPIO_BRR_BR1_Pos               (1U)
#define GPIO_BRR_BR1_Msk               (0x1UL << GPIO_BRR_BR1_Pos)              /*!< 0x00000002 */
#define GPIO_BRR_BR1                   GPIO_BRR_BR1_Msk
#define GPIO_BRR_BR2_Pos               (2U)
#define GPIO_BRR_BR2_Msk               (0x1UL << GPIO_BRR_BR2_Pos)              /*!< 0x00000004 */
#define GPIO_BRR_BR2                   GPIO_BRR_BR2_Msk
#define GPIO_BRR_BR3_Pos               (3U)
#define GPIO_BRR_BR3_Msk               (0x1UL << GPIO_BRR_BR3_Pos)              /*!< 0x00000008 */
#define GPIO_BRR_BR3                   GPIO_BRR_BR3_Msk
#define GPIO_BRR_BR4_Pos               (4U)
#define GPIO_BRR_BR4_Msk               (0x1UL << GPIO_BRR_BR4_Pos)              /*!< 0x00000010 */
#define GPIO_BRR_BR4                   GPIO_BRR_BR4_Msk
#define GPIO_BRR_BR5_Pos               (5U)
#define GPIO_BRR_BR5_Msk               (0x1UL << GPIO_BRR_BR5_Pos)              /*!< 0x00000020 */
#define GPIO_BRR_BR5                   GPIO_BRR_BR5_Msk
#define GPIO_BRR_BR6_Pos               (6U)
#define GPIO_BRR_BR6_Msk               (0x1UL << GPIO_BRR_BR6_Pos)              /*!< 0x00000040 */
#define GPIO_BRR_BR6                   GPIO_BRR_BR6_Msk
#define GPIO_BRR_BR7_Pos               (7U)
#define GPIO_BRR_BR7_Msk               (0x1UL << GPIO_BRR_BR7_Pos)              /*!< 0x00000080 */
#define GPIO_BRR_BR7                   GPIO_BRR_BR7_Msk
#define GPIO_BRR_BR8_Pos               (8U)
#define GPIO_BRR_BR8_Msk               (0x1UL << GPIO_BRR_BR8_Pos)              /*!< 0x00000100 */
#define GPIO_BRR_BR8                   GPIO_BRR_BR8_Msk
#define GPIO_BRR_BR9_Pos               (9U)
#define GPIO_BRR_BR9_Msk               (0x1UL << GPIO_BRR_BR9_Pos)              /*!< 0x00000200 */
#define GPIO_BRR_BR9                   GPIO_BRR_BR9_Msk
#define GPIO_BRR_BR10_Pos              (10U)
#define GPIO_BRR_BR10_Msk              (0x1UL << GPIO_BRR_BR10_Pos)             /*!< 0x00000400 */
#define GPIO_BRR_BR10                  GPIO_BRR_BR10_Msk
#define GPIO_BRR_BR11_Pos              (11U)
#define GPIO_BRR_BR11_Msk              (0x1UL << GPIO_BRR_BR11_Pos)             /*!< 0x00000800 */
#define GPIO_BRR_BR11                  GPIO_BRR_BR11_Msk
#define GPIO_BRR_BR12_Pos              (12U)
#define GPIO_BRR_BR12_Msk              (0x1UL << GPIO_BRR_BR12_Pos)             /*!< 0x00001000 */
#define GPIO_BRR_BR12                  GPIO_BRR_BR12_Msk
#define GPIO_BRR_BR13_Pos              (13U)
#define GPIO_BRR_BR13_Msk              (0x1UL << GPIO_BRR_BR13_Pos)             /*!< 0x00002000 */
#define GPIO_BRR_BR13                  GPIO_BRR_BR13_Msk
#define GPIO_BRR_BR14_Pos              (14U)
#define GPIO_BRR_BR14_Msk              (0x1UL << GPIO_BRR_BR14_Pos)             /*!< 0x00004000 */
#define GPIO_BRR_BR14                  GPIO_BRR_BR14_Msk
#define GPIO_BRR_BR15_Pos              (15U)
#define GPIO_BRR_BR15_Msk              (0x1UL << GPIO_BRR_BR15_Pos)             /*!< 0x00008000 */
#define GPIO_BRR_BR15                  GPIO_BRR_BR15_Msk

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface (I2C)              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE_Pos                      (0U)
#define I2C_CR1_PE_Msk                      (0x1UL << I2C_CR1_PE_Pos)           /*!< 0x00000001 */
#define I2C_CR1_PE                          I2C_CR1_PE_Msk                      /*!< Peripheral Enable */

#define I2C_CR1_WUPEN_Pos                   (2U)
#define I2C_CR1_WUPEN_Msk                   (0x1UL << I2C_CR1_WUPEN_Pos)        /*!< 0x00000004 */
#define I2C_CR1_WUPEN                       I2C_CR1_WUPEN_Msk                   /*!< Wakeup from lowpower mode Enable */

#define I2C_CR1_ENGC_Pos                    (6U)
#define I2C_CR1_ENGC_Msk                    (0x1UL << I2C_CR1_ENGC_Pos)         /*!< 0x00000040 */
#define I2C_CR1_ENGC                        I2C_CR1_ENGC_Msk                    /*!< General Call Enable */

#define I2C_CR1_NOSTRETCH_Pos               (7U)
#define I2C_CR1_NOSTRETCH_Msk               (0x1UL << I2C_CR1_NOSTRETCH_Pos)    /*!< 0x00000080 */
#define I2C_CR1_NOSTRETCH                   I2C_CR1_NOSTRETCH_Msk               /*!< Clock Stretching Disable (Slave mode) */

#define I2C_CR1_START_Pos                   (8U)
#define I2C_CR1_START_Msk                   (0x1UL << I2C_CR1_START_Pos)        /*!< 0x00000100 */
#define I2C_CR1_START                       I2C_CR1_START_Msk                   /*!< Start Generation */

#define I2C_CR1_STOP_Pos                    (9U)
#define I2C_CR1_STOP_Msk                    (0x1UL << I2C_CR1_STOP_Pos)         /*!< 0x00000200 */
#define I2C_CR1_STOP                        I2C_CR1_STOP_Msk                    /*!< Stop Generation */

#define I2C_CR1_ACK_Pos                     (10U)
#define I2C_CR1_ACK_Msk                     (0x1UL << I2C_CR1_ACK_Pos)          /*!< 0x00000400 */
#define I2C_CR1_ACK                         I2C_CR1_ACK_Msk                     /*!< Acknowledge Enable */

#define I2C_CR1_POS_Pos                     (11U)
#define I2C_CR1_POS_Msk                     (0x1UL << I2C_CR1_POS_Pos)          /*!< 0x00000800 */
#define I2C_CR1_POS                         I2C_CR1_POS_Msk                     /*!< Acknowledge/PEC Position (for data reception) */

#define I2C_CR1_SWRST_Pos                   (15U)
#define I2C_CR1_SWRST_Msk                   (0x1UL << I2C_CR1_SWRST_Pos)        /*!< 0x00008000 */
#define I2C_CR1_SWRST                       I2C_CR1_SWRST_Msk                   /*!< Software Reset */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ_Pos                    (0U)
#define I2C_CR2_FREQ_Msk                    (0x7FUL << I2C_CR2_FREQ_Pos)        /*!< 0x0000007F */
#define I2C_CR2_FREQ                        I2C_CR2_FREQ_Msk                   /*!< FREQ[6:0] bits (Peripheral Clock Frequency) */
#define I2C_CR2_FREQ_0                      (0x01UL << I2C_CR2_FREQ_Pos)        /*!< 0x00000001 */
#define I2C_CR2_FREQ_1                      (0x02UL << I2C_CR2_FREQ_Pos)        /*!< 0x00000002 */
#define I2C_CR2_FREQ_2                      (0x04UL << I2C_CR2_FREQ_Pos)        /*!< 0x00000004 */
#define I2C_CR2_FREQ_3                      (0x08UL << I2C_CR2_FREQ_Pos)        /*!< 0x00000008 */
#define I2C_CR2_FREQ_4                      (0x10UL << I2C_CR2_FREQ_Pos)        /*!< 0x00000010 */
#define I2C_CR2_FREQ_5                      (0x20UL << I2C_CR2_FREQ_Pos)        /*!< 0x00000020 */
#define I2C_CR2_FREQ_6                      (0x40UL << I2C_CR2_FREQ_Pos)        /*!< 0x00000040 */

#define I2C_CR2_ITERREN_Pos                 (8U)
#define I2C_CR2_ITERREN_Msk                 (0x1UL << I2C_CR2_ITERREN_Pos)      /*!< 0x00000100 */
#define I2C_CR2_ITERREN                     I2C_CR2_ITERREN_Msk                /*!< Error Interrupt Enable */

#define I2C_CR2_ITEVTEN_Pos                 (9U)
#define I2C_CR2_ITEVTEN_Msk                 (0x1UL << I2C_CR2_ITEVTEN_Pos)      /*!< 0x00000200 */
#define I2C_CR2_ITEVTEN                     I2C_CR2_ITEVTEN_Msk                /*!< Event Interrupt Enable */

#define I2C_CR2_ITBUFEN_Pos                 (10U)
#define I2C_CR2_ITBUFEN_Msk                 (0x1UL << I2C_CR2_ITBUFEN_Pos)      /*!< 0x00000400 */
#define I2C_CR2_ITBUFEN                     I2C_CR2_ITBUFEN_Msk                /*!< Buffer Interrupt Enable */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD1_7                     0x000000FEU             /*!< Interface Address */

#define I2C_OAR1_ADD1_Pos                   (1U)
#define I2C_OAR1_ADD1_Msk                   (0x1UL << I2C_OAR1_ADD1_Pos)        /*!< 0x00000002 */
#define I2C_OAR1_ADD1                       I2C_OAR1_ADD1_Msk                   /*!< Bit 1 */

#define I2C_OAR1_ADD2_Pos                   (2U)
#define I2C_OAR1_ADD2_Msk                   (0x1UL << I2C_OAR1_ADD2_Pos)        /*!< 0x00000004 */
#define I2C_OAR1_ADD2                       I2C_OAR1_ADD2_Msk                   /*!< Bit 2 */

#define I2C_OAR1_ADD3_Pos                   (3U)
#define I2C_OAR1_ADD3_Msk                   (0x1UL << I2C_OAR1_ADD3_Pos)        /*!< 0x00000008 */
#define I2C_OAR1_ADD3                       I2C_OAR1_ADD3_Msk                   /*!< Bit 3 */

#define I2C_OAR1_ADD4_Pos                   (4U)
#define I2C_OAR1_ADD4_Msk                   (0x1UL << I2C_OAR1_ADD4_Pos)        /*!< 0x00000010 */
#define I2C_OAR1_ADD4                       I2C_OAR1_ADD4_Msk                   /*!< Bit 4 */

#define I2C_OAR1_ADD5_Pos                   (5U)
#define I2C_OAR1_ADD5_Msk                   (0x1UL << I2C_OAR1_ADD5_Pos)        /*!< 0x00000020 */
#define I2C_OAR1_ADD5                       I2C_OAR1_ADD5_Msk                   /*!< Bit 5 */

#define I2C_OAR1_ADD6_Pos                   (6U)
#define I2C_OAR1_ADD6_Msk                   (0x1UL << I2C_OAR1_ADD6_Pos)        /*!< 0x00000040 */
#define I2C_OAR1_ADD6                       I2C_OAR1_ADD6_Msk                   /*!< Bit 6 */

#define I2C_OAR1_ADD7_Pos                   (7U)
#define I2C_OAR1_ADD7_Msk                   (0x1UL << I2C_OAR1_ADD7_Pos)        /*!< 0x00000080 */
#define I2C_OAR1_ADD7                       I2C_OAR1_ADD7_Msk                   /*!< Bit 7 */

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR_Pos                       (0U)
#define I2C_DR_DR_Msk                       (0xFFUL << I2C_DR_DR_Pos)           /*!< 0x000000FF */
#define I2C_DR_DR                           I2C_DR_DR_Msk                       /*!< 8-bit Data Register         */
#define I2C_DR_DR_0                         (0x01UL << I2C_DR_DR_Pos)
#define I2C_DR_DR_1                         (0x02UL << I2C_DR_DR_Pos)
#define I2C_DR_DR_2                         (0x04UL << I2C_DR_DR_Pos)
#define I2C_DR_DR_3                         (0x08UL << I2C_DR_DR_Pos)
#define I2C_DR_DR_4                         (0x10UL << I2C_DR_DR_Pos)
#define I2C_DR_DR_5                         (0x20UL << I2C_DR_DR_Pos)
#define I2C_DR_DR_6                         (0x40UL << I2C_DR_DR_Pos)
#define I2C_DR_DR_7                         (0x80UL << I2C_DR_DR_Pos)

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB_Pos                      (0U)
#define I2C_SR1_SB_Msk                      (0x1UL << I2C_SR1_SB_Pos)           /*!< 0x00000001 */
#define I2C_SR1_SB                          I2C_SR1_SB_Msk                      /*!< Start Bit (Master mode) */

#define I2C_SR1_ADDR_Pos                    (1U)
#define I2C_SR1_ADDR_Msk                    (0x1UL << I2C_SR1_ADDR_Pos)         /*!< 0x00000002 */
#define I2C_SR1_ADDR                        I2C_SR1_ADDR_Msk                    /*!< Address sent (master mode)/matched (slave mode) */

#define I2C_SR1_BTF_Pos                     (2U)
#define I2C_SR1_BTF_Msk                     (0x1UL << I2C_SR1_BTF_Pos)          /*!< 0x00000004 */
#define I2C_SR1_BTF                         I2C_SR1_BTF_Msk                     /*!< Byte Transfer Finished */

#define I2C_SR1_STOPF_Pos                   (4U)
#define I2C_SR1_STOPF_Msk                   (0x1UL << I2C_SR1_STOPF_Pos)        /*!< 0x00000010 */
#define I2C_SR1_STOPF                       I2C_SR1_STOPF_Msk                   /*!< Stop detection (Slave mode) */

#define I2C_SR1_RXNE_Pos                    (6U)
#define I2C_SR1_RXNE_Msk                    (0x1UL << I2C_SR1_RXNE_Pos)         /*!< 0x00000040 */
#define I2C_SR1_RXNE                        I2C_SR1_RXNE_Msk                    /*!< Data Register not Empty (receivers) */

#define I2C_SR1_TXE_Pos                     (7U)
#define I2C_SR1_TXE_Msk                     (0x1UL << I2C_SR1_TXE_Pos)          /*!< 0x00000080 */
#define I2C_SR1_TXE                         I2C_SR1_TXE_Msk                     /*!< Data Register Empty (transmitters) */

#define I2C_SR1_BERR_Pos                    (8U)
#define I2C_SR1_BERR_Msk                    (0x1UL << I2C_SR1_BERR_Pos)         /*!< 0x00000100 */
#define I2C_SR1_BERR                        I2C_SR1_BERR_Msk                    /*!< Bus Error */

#define I2C_SR1_ARLO_Pos                    (9U)
#define I2C_SR1_ARLO_Msk                    (0x1UL << I2C_SR1_ARLO_Pos)         /*!< 0x00000200 */
#define I2C_SR1_ARLO                        I2C_SR1_ARLO_Msk                    /*!< Arbitration Lost (master mode) */

#define I2C_SR1_AF_Pos                      (10U)
#define I2C_SR1_AF_Msk                      (0x1UL << I2C_SR1_AF_Pos)           /*!< 0x00000400 */
#define I2C_SR1_AF                          I2C_SR1_AF_Msk                      /*!< Acknowledge Failure */

#define I2C_SR1_OVR_Pos                     (11U)
#define I2C_SR1_OVR_Msk                     (0x1UL << I2C_SR1_OVR_Pos)          /*!< 0x00000800 */
#define I2C_SR1_OVR                         I2C_SR1_OVR_Msk                     /*!< Overrun/Underrun */

#define I2C_SR1_TIMEOUT_Pos                 (14U)
#define I2C_SR1_TIMEOUT_Msk                 (0x1UL << I2C_SR1_TIMEOUT_Pos)       /*!< 0x00004000 */
#define I2C_SR1_TIMEOUT                     I2C_SR1_TIMEOUT_Msk                  /*!< PEC Error in reception */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL_Pos                     (0U)
#define I2C_SR2_MSL_Msk                     (0x1UL << I2C_SR2_MSL_Pos)          /*!< 0x00000001 */
#define I2C_SR2_MSL                         I2C_SR2_MSL_Msk                     /*!< Master/Slave */

#define I2C_SR2_BUSY_Pos                    (1U)
#define I2C_SR2_BUSY_Msk                    (0x1UL << I2C_SR2_BUSY_Pos)         /*!< 0x00000002 */
#define I2C_SR2_BUSY                        I2C_SR2_BUSY_Msk                    /*!< Bus Busy */

#define I2C_SR2_TRA_Pos                     (2U)
#define I2C_SR2_TRA_Msk                     (0x1UL << I2C_SR2_TRA_Pos)          /*!< 0x00000004 */
#define I2C_SR2_TRA                         I2C_SR2_TRA_Msk                     /*!< Transmitter/Receiver */

#define I2C_SR2_GENCALL_Pos                 (4U)
#define I2C_SR2_GENCALL_Msk                 (0x1UL << I2C_SR2_GENCALL_Pos)      /*!< 0x00000010 */
#define I2C_SR2_GENCALL                     I2C_SR2_GENCALL_Msk                 /*!< General Call Address (Slave mode) */

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR_Pos                     (0U)
#define I2C_CCR_CCR_Msk                     (0xFFFUL << I2C_CCR_CCR_Pos)        /*!< 0x00000FFF */
#define I2C_CCR_CCR                         I2C_CCR_CCR_Msk                     /*!< Clock Control Register in Fast/Standard mode (Master mode) */

#define I2C_CCR_FP_Pos                      (13U)
#define I2C_CCR_FP_Msk                      (0x1UL << I2C_CCR_FP_Pos)           /*!< 0x00002000 */
#define I2C_CCR_FP                          I2C_CCR_FP_Msk                      /*!< I2C Master Mode Selection */

#define I2C_CCR_DUTY_Pos                    (14U)
#define I2C_CCR_DUTY_Msk                    (0x1UL << I2C_CCR_DUTY_Pos)         /*!< 0x00004000 */
#define I2C_CCR_DUTY                        I2C_CCR_DUTY_Msk                    /*!< Fast Mode Duty Cycle */

#define I2C_CCR_FS_Pos                      (15U)
#define I2C_CCR_FS_Msk                      (0x1UL << I2C_CCR_FS_Pos)           /*!< 0x00008000 */
#define I2C_CCR_FS                          I2C_CCR_FS_Msk                      /*!< I2C Master Mode Selection */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE_Pos                 (0U)
#define I2C_TRISE_TRISE_Msk                 (0x7FUL << I2C_TRISE_TRISE_Pos)        /*!< 0x0000007F */
#define I2C_TRISE_TRISE                     I2C_TRISE_TRISE_Msk                    /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */

#define I2C_TRISE_THOLDDATA_Pos              (7U)
#define I2C_TRISE_THOLDDATA_Msk              (0x1FUL << I2C_TRISE_THOLDDATA_Pos)     /*!< 0x00000F80 */
#define I2C_TRISE_THOLDDATA                  I2C_TRISE_THOLDDATA_Msk                 /*!< Maximum data hold Time in Fast/Standard mode (Master mode) */

#define I2C_TRISE_THOLDDATA_SEL_Pos         (12U)
#define I2C_TRISE_THOLDDATA_SEL_Msk         (0x1UL << I2C_TRISE_THOLDDATA_SEL_Pos) /*!< 0x00001000 */
#define I2C_TRISE_THOLDDATA_SEL             I2C_TRISE_THOLDDATA_SEL_Msk            /*!< Data hold time select */

/******************  Bit definition for I2C_WT register  *******************/
#define I2C_WT_DIV_Pos                      (0U)
#define I2C_WT_DIV_Msk                      (0x3UL << I2C_WT_DIV_Pos)              /*!< 0x00000003 */
#define I2C_WT_DIV                          I2C_WT_DIV_Msk                         /*!< Pclk divider         */
#define I2C_WT_DIV_0                        (0x01UL << I2C_WT_DIV_Pos)             /*!< 0x00000001 */
#define I2C_WT_DIV_1                        (0x02UL << I2C_WT_DIV_Pos)             /*!< 0x00000002 */

#define I2C_WT_CNT_Pos                      (2U)
#define I2C_WT_CNT_Msk                      (0x3UL << I2C_WT_CNT_Pos)              /*!< 0x0000000C */
#define I2C_WT_CNT                          I2C_WT_CNT_Msk                         /*!< The numbers of timeout counts  */
#define I2C_WT_CNT_0                        (0x01UL << I2C_WT_CNT_Pos)             /*!< 0x00000004 */
#define I2C_WT_CNT_1                        (0x02UL << I2C_WT_CNT_Pos)             /*!< 0x00000008 */

/******************************************************************************/
/*                                                                            */
/*                        Independent WATCHDOG (IWDG)                         */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define IWDG_KR_KEY_Pos      (0U)
#define IWDG_KR_KEY_Msk      (0xFFFFUL << IWDG_KR_KEY_Pos)                     /*!< 0x0000FFFF */
#define IWDG_KR_KEY          IWDG_KR_KEY_Msk                                   /*!< Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define IWDG_PR_PR_Pos       (0U)
#define IWDG_PR_PR_Msk       (0x7UL << IWDG_PR_PR_Pos)                         /*!< 0x00000007 */
#define IWDG_PR_PR           IWDG_PR_PR_Msk                                    /*!< PR[2:0] (Prescaler divider)         */
#define IWDG_PR_PR_0         (0x1UL << IWDG_PR_PR_Pos)                         /*!< 0x00000001 */
#define IWDG_PR_PR_1         (0x2UL << IWDG_PR_PR_Pos)                         /*!< 0x00000002 */
#define IWDG_PR_PR_2         (0x4UL << IWDG_PR_PR_Pos)                         /*!< 0x00000004 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define IWDG_RLR_RL_Pos      (0U)
#define IWDG_RLR_RL_Msk      (0xFFFUL << IWDG_RLR_RL_Pos)                      /*!< 0x00000FFF */
#define IWDG_RLR_RL          IWDG_RLR_RL_Msk                                   /*!< Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define IWDG_SR_PVU_Pos      (0U)
#define IWDG_SR_PVU_Msk      (0x1UL << IWDG_SR_PVU_Pos)                        /*!< 0x00000001 */
#define IWDG_SR_PVU          IWDG_SR_PVU_Msk                                   /*!< Watchdog prescaler value update */

#define IWDG_SR_RVU_Pos      (1U)
#define IWDG_SR_RVU_Msk      (0x1UL << IWDG_SR_RVU_Pos)                        /*!< 0x00000002 */
#define IWDG_SR_RVU          IWDG_SR_RVU_Msk                                   /*!< Watchdog counter reload value update */

/******************************************************************************/
/*                                                                            */
/*                        Power Control (PWR)                                 */
/*                                                                            */
/******************************************************************************/
#define PWR_DEEPSTOP_SUPPORT                  /*!< PWR feature available only on specific devices: Deep stop feature */

/********************  Bit definition for PWR_CR1 register  ********************/
#define PWR_CR1_DBP_Pos                (8U)
#define PWR_CR1_DBP_Msk                (0x1UL << PWR_CR1_DBP_Pos)                    /*!< 0x00000100 */
#define PWR_CR1_DBP                    PWR_CR1_DBP_Msk                               /*!< dbg mode protect enable */

#define PWR_CR1_FLS_SLPTIME_Pos        (12U)
#define PWR_CR1_FLS_SLPTIME_Msk        (0x3UL << PWR_CR1_FLS_SLPTIME_Pos)            /*!< 0x00003000 */
#define PWR_CR1_FLS_SLPTIME            PWR_CR1_FLS_SLPTIME_Msk                       /*!< flash sleep time control */
#define PWR_CR1_FLS_SLPTIME_0          (0x1UL << PWR_CR1_FLS_SLPTIME_Pos)            /*!< 0x00001000 */
#define PWR_CR1_FLS_SLPTIME_1          (0x2UL << PWR_CR1_FLS_SLPTIME_Pos)            /*!< 0x00002000 */

#define PWR_CR1_LPR_Pos                (14U)
#define PWR_CR1_LPR_Msk                (0x3UL << PWR_CR1_LPR_Pos)                    /*!< 0x0000C000 */
#define PWR_CR1_LPR                    PWR_CR1_LPR_Msk                               /*!< pmu mode control */
#define PWR_CR1_LPR_0                  (0x1UL << PWR_CR1_LPR_Pos)                    /*!< 0x00004000 */
#define PWR_CR1_LPR_1                  (0x2UL << PWR_CR1_LPR_Pos)                    /*!< 0x00008000 */

#define PWR_CR1_SRAM_RETV_CTRL_Pos     (17U)
#define PWR_CR1_SRAM_RETV_CTRL_Msk     (0x3UL << PWR_CR1_SRAM_RETV_CTRL_Pos)         /*!< 0x00060000 */
#define PWR_CR1_SRAM_RETV_CTRL         PWR_CR1_SRAM_RETV_CTRL_Msk                    /*!< sram retev  control */
#define PWR_CR1_SRAM_RETV_CTRL_0       (0x1UL << PWR_CR1_SRAM_RETV_CTRL_Pos)         /*!< 0x00020000 */
#define PWR_CR1_SRAM_RETV_CTRL_1       (0x2UL << PWR_CR1_SRAM_RETV_CTRL_Pos)         /*!< 0x00040000 */

#define PWR_CR1_HSION_CTRL_Pos         (19U)
#define PWR_CR1_HSION_CTRL_Msk         (0x1UL << PWR_CR1_HSION_CTRL_Pos)             /*!< 0x00080000 */
#define PWR_CR1_HSION_CTRL             PWR_CR1_HSION_CTRL_Msk                        /*!< HSION CTRL */

/******************************************************************************/
/*                                                                            */
/*                           Reset and Clock Control (RCC)                    */
/*                                                                            */
/******************************************************************************/
/*
* @brief Specific device feature definitions
*/
#define RCC_LSE_SUPPORT

/********************  Bit definition for RCC_CR register  *****************/
#define RCC_CR_HSION_Pos                 (8U)
#define RCC_CR_HSION_Msk                 (0x1UL << RCC_CR_HSION_Pos)           /*!< 0x00000100 */
#define RCC_CR_HSION                     RCC_CR_HSION_Msk                      /*!< Internal High Speed clock enable */

#define RCC_CR_HSIRDY_Pos                (10U)
#define RCC_CR_HSIRDY_Msk                (0x1UL << RCC_CR_HSIRDY_Pos)          /*!< 0x00000400 */
#define RCC_CR_HSIRDY                    RCC_CR_HSIRDY_Msk                     /*!< Internal High Speed clock ready flag */

#define RCC_CR_HSIDIV_Pos                (11U)
#define RCC_CR_HSIDIV_Msk                (0x7UL << RCC_CR_HSIDIV_Pos)          /*!< 0x00003800 */
#define RCC_CR_HSIDIV                    RCC_CR_HSIDIV_Msk                     /*!< HSIDIV[13:11] Internal High Speed clock division factor */
#define RCC_CR_HSIDIV_0                  (0x1UL << RCC_CR_HSIDIV_Pos)          /*!< 0x00000800 */
#define RCC_CR_HSIDIV_1                  (0x2UL << RCC_CR_HSIDIV_Pos)          /*!< 0x00001000 */
#define RCC_CR_HSIDIV_2                  (0x4UL << RCC_CR_HSIDIV_Pos)          /*!< 0x00002000 */

#define RCC_CR_HSEON_Pos                 (16U)
#define RCC_CR_HSEON_Msk                 (0x1UL << RCC_CR_HSEON_Pos)           /*!< 0x00010000 */
#define RCC_CR_HSEON                     RCC_CR_HSEON_Msk                      /*!< External High Speed clock enable */

#define RCC_CR_HSERDY_Pos                (17U)
#define RCC_CR_HSERDY_Msk                (0x1UL << RCC_CR_HSERDY_Pos)          /*!< 0x00020000 */
#define RCC_CR_HSERDY                    RCC_CR_HSERDY_Msk                     /*!< External High Speed clock Ready flag */

#define RCC_CR_HSEBYP_Pos                (18U)
#define RCC_CR_HSEBYP_Msk                (0x1UL << RCC_CR_HSEBYP_Pos)          /*!< 0x00040000 */
#define RCC_CR_HSEBYP                    RCC_CR_HSEBYP_Msk                     /*!< External High Speed clock bypass enable */

#define RCC_CR_HSE_CSSON_Pos             (19U)
#define RCC_CR_HSE_CSSON_Msk             (0x1UL << RCC_CR_HSE_CSSON_Pos)       /*!< 0x00080000 */
#define RCC_CR_HSE_CSSON                 RCC_CR_HSE_CSSON_Msk                  /*!< External High Speed clock CSS enable */

/********************  Bit definition for RCC_ICSCR register  ***************/
#define RCC_ICSCR_HSI_TRIM_Pos            (0U)
#define RCC_ICSCR_HSI_TRIM_Msk            (0x1FFFUL << RCC_ICSCR_HSI_TRIM_Pos)   /*!< 0x00001FFF */
#define RCC_ICSCR_HSI_TRIM                RCC_ICSCR_HSI_TRIM_Msk                 /*!< HSITRIM[12:0] bits */
#define RCC_ICSCR_HSI_TRIM_0              (0x01UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000001 */
#define RCC_ICSCR_HSI_TRIM_1              (0x02UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000002 */
#define RCC_ICSCR_HSI_TRIM_2              (0x04UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000004 */
#define RCC_ICSCR_HSI_TRIM_3              (0x08UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000008 */
#define RCC_ICSCR_HSI_TRIM_4              (0x10UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000010 */
#define RCC_ICSCR_HSI_TRIM_5              (0x20UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000020 */
#define RCC_ICSCR_HSI_TRIM_6              (0x40UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000040 */
#define RCC_ICSCR_HSI_TRIM_7              (0x80UL << RCC_ICSCR_HSI_TRIM_Pos)     /*!< 0x00000080 */
#define RCC_ICSCR_HSI_TRIM_8              (0x100UL << RCC_ICSCR_HSI_TRIM_Pos)    /*!< 0x00000100 */
#define RCC_ICSCR_HSI_TRIM_9              (0x200UL << RCC_ICSCR_HSI_TRIM_Pos)    /*!< 0x00000200 */
#define RCC_ICSCR_HSI_TRIM_10             (0x400UL << RCC_ICSCR_HSI_TRIM_Pos)    /*!< 0x00000400 */
#define RCC_ICSCR_HSI_TRIM_11             (0x800UL << RCC_ICSCR_HSI_TRIM_Pos)    /*!< 0x00000800 */
#define RCC_ICSCR_HSI_TRIM_12             (0x1000UL << RCC_ICSCR_HSI_TRIM_Pos)   /*!< 0x00001000 */

#define RCC_ICSCR_HSI_FS_Pos              (13U)
#define RCC_ICSCR_HSI_FS_Msk              (0x7UL << RCC_ICSCR_HSI_FS_Pos)        /*!< 0x0000E000 */
#define RCC_ICSCR_HSI_FS                  RCC_ICSCR_HSI_FS_Msk                   /*!< HSIFS[15:13] bits */
#define RCC_ICSCR_HSI_FS_0                (0x01UL << RCC_ICSCR_HSI_FS_Pos)       /*!< 0x00002000 */
#define RCC_ICSCR_HSI_FS_1                (0x02UL << RCC_ICSCR_HSI_FS_Pos)       /*!< 0x00004000 */
#define RCC_ICSCR_HSI_FS_2                (0x04UL << RCC_ICSCR_HSI_FS_Pos)       /*!< 0x00008000 */

#define RCC_ICSCR_LSI_TRIM_Pos            (16U)
#define RCC_ICSCR_LSI_TRIM_Msk            (0x1FFUL << RCC_ICSCR_LSI_TRIM_Pos)    /*!< 0x01FF0000 */
#define RCC_ICSCR_LSI_TRIM                RCC_ICSCR_LSI_TRIM_Msk                 /*!< LSITRIM[24:16] bits */
#define RCC_ICSCR_LSI_TRIM_0              (0x01UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00010000 */
#define RCC_ICSCR_LSI_TRIM_1              (0x02UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00020000 */ 
#define RCC_ICSCR_LSI_TRIM_2              (0x04UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00040000 */ 
#define RCC_ICSCR_LSI_TRIM_3              (0x08UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00080000 */ 
#define RCC_ICSCR_LSI_TRIM_4              (0x10UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00100000 */ 
#define RCC_ICSCR_LSI_TRIM_5              (0x20UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00200000 */ 
#define RCC_ICSCR_LSI_TRIM_6              (0x40UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00400000 */ 
#define RCC_ICSCR_LSI_TRIM_7              (0x80UL << RCC_ICSCR_LSI_TRIM_Pos)     /*!< 0x00800000 */ 
#define RCC_ICSCR_LSI_TRIM_8              (0x100UL << RCC_ICSCR_LSI_TRIM_Pos)    /*!< 0x01000000 */ 

/********************  Bit definition for RCC_CFGR register  ***************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                (0U)
#define RCC_CFGR_SW_Msk                (0x7UL << RCC_CFGR_SW_Pos)              /*!< 0x00000007 */
#define RCC_CFGR_SW                    RCC_CFGR_SW_Msk                         /*!< SW[2:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                  (0x1UL << RCC_CFGR_SW_Pos)              /*!< 0x00000001 */
#define RCC_CFGR_SW_1                  (0x2UL << RCC_CFGR_SW_Pos)              /*!< 0x00000002 */
#define RCC_CFGR_SW_2                  (0x4UL << RCC_CFGR_SW_Pos)              /*!< 0x00000004 */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos               (3U)
#define RCC_CFGR_SWS_Msk               (0x7UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000038 */
#define RCC_CFGR_SWS                   RCC_CFGR_SWS_Msk                        /*!< SWS[2:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                 (0x1UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000008 */
#define RCC_CFGR_SWS_1                 (0x2UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000010 */
#define RCC_CFGR_SWS_2                 (0x4UL << RCC_CFGR_SWS_Pos)             /*!< 0x00000020 */

#define RCC_CFGR_SWS_HSISYS            (0UL)                                   /*!< HSISYS used as system clock */
#define RCC_CFGR_SWS_HSE               (0x00000008UL)                          /*!< HSE used as system clock */
#define RCC_CFGR_SWS_LSI               (0x00000018UL)                          /*!< LSI used as system clock */
#define RCC_CFGR_SWS_LSE               (0x00000020UL)                          /*!< LSE used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos              (8U)
#define RCC_CFGR_HPRE_Msk              (0xFUL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000F00 */
#define RCC_CFGR_HPRE                  RCC_CFGR_HPRE_Msk                       /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                (0x1UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000100 */
#define RCC_CFGR_HPRE_1                (0x2UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000200 */
#define RCC_CFGR_HPRE_2                (0x4UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000400 */
#define RCC_CFGR_HPRE_3                (0x8UL << RCC_CFGR_HPRE_Pos)            /*!< 0x00000800 */

/*!< PPRE configuration */
#define RCC_CFGR_PPRE_Pos              (12U)
#define RCC_CFGR_PPRE_Msk              (0x7UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00007000 */
#define RCC_CFGR_PPRE                  RCC_CFGR_PPRE_Msk                       /*!< PRE1[2:0] bits (APB prescaler) */
#define RCC_CFGR_PPRE_0                (0x1UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00001000 */
#define RCC_CFGR_PPRE_1                (0x2UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00002000 */
#define RCC_CFGR_PPRE_2                (0x4UL << RCC_CFGR_PPRE_Pos)            /*!< 0x00004000 */

/*!< MCOSEL configuration */
#define RCC_CFGR_MCOSEL_Pos            (24U)
#define RCC_CFGR_MCOSEL_Msk            (0xFUL << RCC_CFGR_MCOSEL_Pos)          /*!< 0x0F000000 */
#define RCC_CFGR_MCOSEL                RCC_CFGR_MCOSEL_Msk                     /*!< MCOSEL [3:0] bits (Clock output selection) */
#define RCC_CFGR_MCOSEL_0              (0x1UL << RCC_CFGR_MCOSEL_Pos)          /*!< 0x01000000 */
#define RCC_CFGR_MCOSEL_1              (0x2UL << RCC_CFGR_MCOSEL_Pos)          /*!< 0x02000000 */
#define RCC_CFGR_MCOSEL_2              (0x4UL << RCC_CFGR_MCOSEL_Pos)          /*!< 0x04000000 */
#define RCC_CFGR_MCOSEL_3              (0x8UL << RCC_CFGR_MCOSEL_Pos)          /*!< 0x08000000 */
/*!< MCO Prescaler configuration */
#define RCC_CFGR_MCOPRE_Pos            (28U)
#define RCC_CFGR_MCOPRE_Msk            (0x7UL << RCC_CFGR_MCOPRE_Pos)          /*!< 0x70000000 */
#define RCC_CFGR_MCOPRE                RCC_CFGR_MCOPRE_Msk                     /*!< MCO prescaler [2:0] */
#define RCC_CFGR_MCOPRE_0              (0x1UL << RCC_CFGR_MCOPRE_Pos)          /*!< 0x10000000 */
#define RCC_CFGR_MCOPRE_1              (0x2UL << RCC_CFGR_MCOPRE_Pos)          /*!< 0x20000000 */
#define RCC_CFGR_MCOPRE_2              (0x4UL << RCC_CFGR_MCOPRE_Pos)          /*!< 0x40000000 */

/********************  Bit definition for RCC_ECSCR register  ***************/
#define RCC_ECSCR_HSE_DRV_Pos          (0U)
#define RCC_ECSCR_HSE_DRV_Msk          (0x1UL << RCC_ECSCR_HSE_DRV_Pos)        /*!< 0x00000001 */
#define RCC_ECSCR_HSE_DRV              RCC_ECSCR_HSE_DRV_Msk                   /*!< HSE drive capacity setting */      

#define RCC_ECSCR_HSE_STARTUP_Pos      (3U)
#define RCC_ECSCR_HSE_STARTUP_Msk      (0x3UL << RCC_ECSCR_HSE_STARTUP_Pos)    /*!< 0x00000018 */
#define RCC_ECSCR_HSE_STARTUP          RCC_ECSCR_HSE_STARTUP_Msk               /*!< HSE start up time select */  
#define RCC_ECSCR_HSE_STARTUP_0        (0x01UL << RCC_ECSCR_HSE_STARTUP_Pos)   /*!< 0x00000008 */
#define RCC_ECSCR_HSE_STARTUP_1        (0x02UL << RCC_ECSCR_HSE_STARTUP_Pos)   /*!< 0x00000010 */  

#define RCC_ECSCR_LSE_DRIVER_Pos       (16U)
#define RCC_ECSCR_LSE_DRIVER_Msk       (0x3UL << RCC_ECSCR_LSE_DRIVER_Pos)     /*!< 0x00030000 */
#define RCC_ECSCR_LSE_DRIVER           RCC_ECSCR_LSE_DRIVER_Msk                /*!< LSE drive capacity setting */ 
#define RCC_ECSCR_LSE_DRIVER_0         (0x1UL <<RCC_ECSCR_LSE_DRIVER_Pos)      /*!< 0x00010000 */
#define RCC_ECSCR_LSE_DRIVER_1         (0x2UL <<RCC_ECSCR_LSE_DRIVER_Pos)      /*!< 0x00020000 */

#define RCC_ECSCR_LSE_STARTUP_Pos      (20U)
#define RCC_ECSCR_LSE_STARTUP_Msk      (0x3UL << RCC_ECSCR_LSE_STARTUP_Pos)    /*!< 0x00300000 */
#define RCC_ECSCR_LSE_STARTUP          RCC_ECSCR_LSE_STARTUP_Msk               /*!< LSE start up time select */ 
#define RCC_ECSCR_LSE_STARTUP_0        (0x1UL <<RCC_ECSCR_LSE_STARTUP_Pos)     /*!< 0x00100000 */
#define RCC_ECSCR_LSE_STARTUP_1        (0x2UL <<RCC_ECSCR_LSE_STARTUP_Pos)     /*!< 0x00200000 */

/********************  Bit definition for RCC_CIER register  ******************/
#define RCC_CIER_LSIRDYIE_Pos            (0U)
#define RCC_CIER_LSIRDYIE_Msk            (0x1UL << RCC_CIER_LSIRDYIE_Pos)      /*!< 0x00000001 */
#define RCC_CIER_LSIRDYIE                RCC_CIER_LSIRDYIE_Msk                 /*!< LSI ready interrupt enable */

#define RCC_CIER_LSERDYIE_Pos            (1U)
#define RCC_CIER_LSERDYIE_Msk            (0x1UL << RCC_CIER_LSERDYIE_Pos)      /*!< 0x00000002 */
#define RCC_CIER_LSERDYIE                RCC_CIER_LSERDYIE_Msk                 /*!< LSE ready interrupt enable */

#define RCC_CIER_HSIRDYIE_Pos            (3U)
#define RCC_CIER_HSIRDYIE_Msk            (0x1UL << RCC_CIER_HSIRDYIE_Pos)      /*!< 0x00000008 */
#define RCC_CIER_HSIRDYIE                RCC_CIER_HSIRDYIE_Msk                 /*!< HSI ready interrupt enable */

#define RCC_CIER_HSERDYIE_Pos            (4U)
#define RCC_CIER_HSERDYIE_Msk            (0x1UL << RCC_CIER_HSERDYIE_Pos)      /*!< 0x00000010 */
#define RCC_CIER_HSERDYIE                RCC_CIER_HSERDYIE_Msk                 /*!< HSE ready interrupt enable */

/********************  Bit definition for RCC_CIFR register  ******************/
#define RCC_CIFR_LSIRDYF_Pos             (0U)
#define RCC_CIFR_LSIRDYF_Msk             (0x1UL << RCC_CIFR_LSIRDYF_Pos)       /*!< 0x00000001 */
#define RCC_CIFR_LSIRDYF                 RCC_CIFR_LSIRDYF_Msk                  /*!< LSI ready interrupt flag */

#define RCC_CIFR_LSERDYF_Pos             (1U)
#define RCC_CIFR_LSERDYF_Msk             (0x1UL << RCC_CIFR_LSERDYF_Pos)       /*!< 0x00000002 */
#define RCC_CIFR_LSERDYF                 RCC_CIFR_LSERDYF_Msk                  /*!< LSE ready interrupt flag */

#define RCC_CIFR_HSIRDYF_Pos             (3U)
#define RCC_CIFR_HSIRDYF_Msk             (0x1UL << RCC_CIFR_HSIRDYF_Pos)       /*!< 0x00000008 */
#define RCC_CIFR_HSIRDYF                 RCC_CIFR_HSIRDYF_Msk                  /*!< HSI ready interrupt flag */

#define RCC_CIFR_HSERDYF_Pos             (4U)
#define RCC_CIFR_HSERDYF_Msk             (0x1UL << RCC_CIFR_HSERDYF_Pos)       /*!< 0x00000010 */
#define RCC_CIFR_HSERDYF                 RCC_CIFR_HSERDYF_Msk                  /*!< HSE readt interrupt flag */

#define RCC_CIFR_CSSF_Pos                (8U)
#define RCC_CIFR_CSSF_Msk                (0x1UL << RCC_CIFR_CSSF_Pos)          /*!< 0x00000100 */
#define RCC_CIFR_CSSF                    RCC_CIFR_CSSF_Msk                     /*!< HSE clock safety system interrupt flag */

#define RCC_CIFR_LSECSSF_Pos             (9U)
#define RCC_CIFR_LSECSSF_Msk             (0x1UL << RCC_CIFR_LSECSSF_Pos)       /*!< 0x00000200 */
#define RCC_CIFR_LSECSSF                 RCC_CIFR_LSECSSF_Msk                  /*!< LSE clock safety system interrupt flag */

/********************  Bit definition for RCC_CICR register  ******************/
#define RCC_CICR_LSIRDYC_Pos             (0U)
#define RCC_CICR_LSIRDYC_Msk             (0x1UL << RCC_CICR_LSIRDYC_Pos)       /*!< 0x00000001 */
#define RCC_CICR_LSIRDYC                 RCC_CICR_LSIRDYC_Msk                  /*!< LSI ready interrupt flag clear */

#define RCC_CICR_LSERDYC_Pos             (1U)
#define RCC_CICR_LSERDYC_Msk             (0x1UL << RCC_CICR_LSERDYC_Pos)       /*!< 0x00000002 */
#define RCC_CICR_LSERDYC                 RCC_CICR_LSERDYC_Msk                  /*!< LSE ready interrupt flag clear */

#define RCC_CICR_HSIRDYC_Pos             (3U)
#define RCC_CICR_HSIRDYC_Msk             (0x1UL << RCC_CICR_HSIRDYC_Pos)       /*!< 0x00000008 */
#define RCC_CICR_HSIRDYC                 RCC_CICR_HSIRDYC_Msk                  /*!< HSI ready interrupt flag clear */

#define RCC_CICR_HSERDYC_Pos             (4U)
#define RCC_CICR_HSERDYC_Msk             (0x1UL << RCC_CICR_HSERDYC_Pos)       /*!< 0x00000010 */
#define RCC_CICR_HSERDYC                 RCC_CICR_HSERDYC_Msk                  /*!< HSE ready interrupt flag clear */

#define RCC_CICR_CSSC_Pos                (8U)
#define RCC_CICR_CSSC_Msk                (0x1UL << RCC_CICR_CSSC_Pos)          /*!< 0x00000100 */
#define RCC_CICR_CSSC                    RCC_CICR_CSSC_Msk                     /*!< HSE clock safety system interrupt flag clear */

#define RCC_CICR_LSECSSC_Pos             (9U)
#define RCC_CICR_LSECSSC_Msk             (0x1UL << RCC_CICR_LSECSSC_Pos)       /*!< 0x00000200 */
#define RCC_CICR_LSECSSC                 RCC_CICR_LSECSSC_Msk                  /*!< LSE clock safety system interrupt flag clear */

/********************  Bit definition for RCC_IOPRSTR register  ****************/
#define RCC_IOPRSTR_GPIOARST_Pos         (0U)
#define RCC_IOPRSTR_GPIOARST_Msk         (0x1UL << RCC_IOPRSTR_GPIOARST_Pos)   /*!< 0x00000001 */
#define RCC_IOPRSTR_GPIOARST             RCC_IOPRSTR_GPIOARST_Msk              /*!< PortA reset */

#define RCC_IOPRSTR_GPIOBRST_Pos         (1U)
#define RCC_IOPRSTR_GPIOBRST_Msk         (0x1UL << RCC_IOPRSTR_GPIOBRST_Pos)   /*!< 0x00000002 */
#define RCC_IOPRSTR_GPIOBRST             RCC_IOPRSTR_GPIOBRST_Msk              /*!< PortB reset */

#define RCC_IOPRSTR_GPIOFRST_Pos         (5U)
#define RCC_IOPRSTR_GPIOFRST_Msk         (0x1UL << RCC_IOPRSTR_GPIOFRST_Pos)   /*!< 0x00000004 */
#define RCC_IOPRSTR_GPIOFRST             RCC_IOPRSTR_GPIOFRST_Msk              /*!< PortF reset */

/********************  Bit definition for RCC_AHBRSTR register  ***************/
#define RCC_AHBRSTR_CRCRST_Pos           (12U)
#define RCC_AHBRSTR_CRCRST_Msk           (0x1UL << RCC_AHBRSTR_CRCRST_Pos)     /*!< 0x00001000 */
#define RCC_AHBRSTR_CRCRST               RCC_AHBRSTR_CRCRST_Msk                /*!< CRC reset */

/********************  Bit definition for RCC_APBRSTR1 register  **************/
#define RCC_APBRSTR1_RTCAPBRST_Pos       (10U)
#define RCC_APBRSTR1_RTCAPBRST_Msk       (0x1UL << RCC_APBRSTR1_RTCAPBRST_Pos)  /*!< 0x00000400 */
#define RCC_APBRSTR1_RTCAPBRST           RCC_APBRSTR1_RTCAPBRST_Msk             /*!< RTC reset */

#define RCC_APBRSTR1_UART1RST_Pos        (17U)
#define RCC_APBRSTR1_UART1RST_Msk        (0x1UL << RCC_APBRSTR1_UART1RST_Pos)   /*!< 0x00020000 */
#define RCC_APBRSTR1_UART1RST            RCC_APBRSTR1_UART1RST_Msk              /*!< UART1 reset */

#define RCC_APBRSTR1_UART2RST_Pos        (18U)
#define RCC_APBRSTR1_UART2RST_Msk        (0x1UL << RCC_APBRSTR1_UART2RST_Pos)   /*!< 0x00040000 */
#define RCC_APBRSTR1_UART2RST            RCC_APBRSTR1_UART2RST_Msk              /*!< UART2 reset */

#define RCC_APBRSTR1_I2CRST_Pos          (21U)
#define RCC_APBRSTR1_I2CRST_Msk          (0x1UL << RCC_APBRSTR1_I2CRST_Pos)     /*!< 0x00200000 */
#define RCC_APBRSTR1_I2CRST              RCC_APBRSTR1_I2CRST_Msk                /*!< I2C reset */

#define RCC_APBRSTR1_DBGRST_Pos          (27U)
#define RCC_APBRSTR1_DBGRST_Msk          (0x1UL << RCC_APBRSTR1_DBGRST_Pos)     /*!< 0x08000000 */
#define RCC_APBRSTR1_DBGRST              RCC_APBRSTR1_DBGRST_Msk                /*!< DBG reset */

#define RCC_APBRSTR1_PWRRST_Pos          (28U)
#define RCC_APBRSTR1_PWRRST_Msk          (0x1UL << RCC_APBRSTR1_PWRRST_Pos)     /*!< 0x10000000 */
#define RCC_APBRSTR1_PWRRST              RCC_APBRSTR1_PWRRST_Msk                /*!< PWR reset */

/********************  Bit definition for RCC_APBRSTR2 register  **************/
#define RCC_APBRSTR2_SYSCFGRST_Pos       (0U)
#define RCC_APBRSTR2_SYSCFGRST_Msk       (0x1UL << RCC_APBRSTR2_SYSCFGRST_Pos)  /*!< 0x00000001 */
#define RCC_APBRSTR2_SYSCFGRST           RCC_APBRSTR2_SYSCFGRST_Msk             /*!< SYSCFG reset */

#define RCC_APBRSTR2_TIM1RST_Pos         (11U)
#define RCC_APBRSTR2_TIM1RST_Msk         (0x1UL << RCC_APBRSTR2_TIM1RST_Pos)    /*!< 0x00000800 */
#define RCC_APBRSTR2_TIM1RST             RCC_APBRSTR2_TIM1RST_Msk               /*!< TIM1 reset */

#define RCC_APBRSTR2_SPI1RST_Pos         (12U)
#define RCC_APBRSTR2_SPI1RST_Msk         (0x1UL << RCC_APBRSTR2_SPI1RST_Pos)    /*!< 0x00001000 */
#define RCC_APBRSTR2_SPI1RST             RCC_APBRSTR2_SPI1RST_Msk               /*!< SPI reset */

#define RCC_APBRSTR2_UART3RST_Pos        (13U)
#define RCC_APBRSTR2_UART3RST_Msk        (0x1UL << RCC_APBRSTR2_UART3RST_Pos)   /*!< 0x00002000 */
#define RCC_APBRSTR2_UART3RST            RCC_APBRSTR2_UART3RST_Msk              /*!< UART3 reset */

#define RCC_APBRSTR2_TIM14RST_Pos        (15U)
#define RCC_APBRSTR2_TIM14RST_Msk        (0x1UL << RCC_APBRSTR2_TIM14RST_Pos)   /*!< 0x00008000 */
#define RCC_APBRSTR2_TIM14RST            RCC_APBRSTR2_TIM14RST_Msk              /*!< TIM14 reset */

#define RCC_APBRSTR2_ADCRST_Pos          (20U)
#define RCC_APBRSTR2_ADCRST_Msk          (0x1UL << RCC_APBRSTR2_ADCRST_Pos)     /*!< 0x00100000 */
#define RCC_APBRSTR2_ADCRST              RCC_APBRSTR2_ADCRST_Msk                /*!< ADC reset */

#define RCC_APBRSTR2_COMP1RST_Pos        (21U)
#define RCC_APBRSTR2_COMP1RST_Msk        (0x1UL << RCC_APBRSTR2_COMP1RST_Pos)   /*!< 0x00200000 */
#define RCC_APBRSTR2_COMP1RST            RCC_APBRSTR2_COMP1RST_Msk              /*!< COMP1 reset */

#define RCC_APBRSTR2_COMP2RST_Pos        (22U)
#define RCC_APBRSTR2_COMP2RST_Msk        (0x1UL << RCC_APBRSTR2_COMP2RST_Pos)   /*!< 0x00400000 */
#define RCC_APBRSTR2_COMP2RST            RCC_APBRSTR2_COMP2RST_Msk              /*!< COMP2 reset */

#define RCC_APBRSTR2_TKRST_Pos           (25U)
#define RCC_APBRSTR2_TKRST_Msk           (0x1UL << RCC_APBRSTR2_TKRST_Pos)      /*!< 0x02000000 */
#define RCC_APBRSTR2_TKRST               RCC_APBRSTR2_TKRST_Msk                 /*!< TK reset */

/********************  Bit definition for RCC_IOPENR register  ****************/
#define RCC_IOPENR_GPIOAEN_Pos           (0U)
#define RCC_IOPENR_GPIOAEN_Msk           (0x1UL << RCC_IOPENR_GPIOAEN_Pos)      /*!< 0x00000001 */
#define RCC_IOPENR_GPIOAEN               RCC_IOPENR_GPIOAEN_Msk                 /*!< GPIOA clock enable */

#define RCC_IOPENR_GPIOBEN_Pos           (1U)
#define RCC_IOPENR_GPIOBEN_Msk           (0x1UL << RCC_IOPENR_GPIOBEN_Pos)      /*!< 0x00000002 */
#define RCC_IOPENR_GPIOBEN               RCC_IOPENR_GPIOBEN_Msk                 /*!< GPIOB clock enable */

#define RCC_IOPENR_GPIOFEN_Pos           (5U)
#define RCC_IOPENR_GPIOFEN_Msk           (0x1UL << RCC_IOPENR_GPIOFEN_Pos)      /*!< 0x00000020 */
#define RCC_IOPENR_GPIOFEN               RCC_IOPENR_GPIOFEN_Msk                 /*!< GPIOF clock enable */

/********************  Bit definition for RCC_AHBENR register  ****************/
#define RCC_AHBENR_FLASHEN_Pos           (8U)
#define RCC_AHBENR_FLASHEN_Msk           (0x1UL << RCC_AHBENR_FLASHEN_Pos)      /*!< 0x00000100 */
#define RCC_AHBENR_FLASHEN               RCC_AHBENR_FLASHEN_Msk                 /*!< Flash interface clock enable in sleep mode */

#define RCC_AHBENR_SRAMEN_Pos            (9U)
#define RCC_AHBENR_SRAMEN_Msk            (0x1UL << RCC_AHBENR_SRAMEN_Pos)       /*!< 0x00000200 */
#define RCC_AHBENR_SRAMEN                RCC_AHBENR_SRAMEN_Msk                  /*!< SRAM clock enable in sleep mode */

#define RCC_AHBENR_CRCEN_Pos             (12U)
#define RCC_AHBENR_CRCEN_Msk             (0x1UL << RCC_AHBENR_CRCEN_Pos)        /*!< 0x00001000 */
#define RCC_AHBENR_CRCEN                 RCC_AHBENR_CRCEN_Msk                   /*!< CRC clock enable */

/********************  Bit definition for RCC_APBENR1 register  ***************/
#define RCC_APBENR1_RTCAPBEN_Pos         (10U)
#define RCC_APBENR1_RTCAPBEN_Msk         (0x1UL << RCC_APBENR1_RTCAPBEN_Pos)    /*!< 0x00000400 */
#define RCC_APBENR1_RTCAPBEN             RCC_APBENR1_RTCAPBEN_Msk               /*!< RTC clock enable */

#define RCC_APBENR1_UART1EN_Pos          (17U)
#define RCC_APBENR1_UART1EN_Msk          (0x1UL << RCC_APBENR1_UART1EN_Pos)     /*!< 0x00020000 */
#define RCC_APBENR1_UART1EN              RCC_APBENR1_UART1EN_Msk                /*!< UART1 clock enable */

#define RCC_APBENR1_UART2EN_Pos          (18U)
#define RCC_APBENR1_UART2EN_Msk          (0x1UL << RCC_APBENR1_UART2EN_Pos)     /*!< 0x00040000 */
#define RCC_APBENR1_UART2EN              RCC_APBENR1_UART2EN_Msk                /*!< UART2 clock enable */

#define RCC_APBENR1_I2CEN_Pos            (21U)
#define RCC_APBENR1_I2CEN_Msk            (0x1UL << RCC_APBENR1_I2CEN_Pos)       /*!< 0x00200000 */
#define RCC_APBENR1_I2CEN                RCC_APBENR1_I2CEN_Msk                  /*!< I2C clock enable */

#define RCC_APBENR1_DBGEN_Pos            (27U)
#define RCC_APBENR1_DBGEN_Msk            (0x1UL << RCC_APBENR1_DBGEN_Pos)       /*!< 0x08000000 */
#define RCC_APBENR1_DBGEN                RCC_APBENR1_DBGEN_Msk                  /*!< DBG clock enable */

#define RCC_APBENR1_PWREN_Pos            (28U)
#define RCC_APBENR1_PWREN_Msk            (0x1UL << RCC_APBENR1_PWREN_Pos)       /*!< 0x10000000 */
#define RCC_APBENR1_PWREN                RCC_APBENR1_PWREN_Msk                  /*!< PWR clock enable */

/********************  Bit definition for RCC_APBENR2 register  **************/
#define RCC_APBENR2_SYSCFGEN_Pos         (0U)
#define RCC_APBENR2_SYSCFGEN_Msk         (0x1UL << RCC_APBENR2_SYSCFGEN_Pos)    /*!< 0x00000001 */
#define RCC_APBENR2_SYSCFGEN             RCC_APBENR2_SYSCFGEN_Msk               /*!< SYSCFG clock enable */

#define RCC_APBENR2_TIM1EN_Pos           (11U)
#define RCC_APBENR2_TIM1EN_Msk           (0x1UL << RCC_APBENR2_TIM1EN_Pos)      /*!< 0x00000800 */
#define RCC_APBENR2_TIM1EN               RCC_APBENR2_TIM1EN_Msk                 /*!< TIM1 clock enable */

#define RCC_APBENR2_SPI1EN_Pos           (12U)
#define RCC_APBENR2_SPI1EN_Msk           (0x1UL << RCC_APBENR2_SPI1EN_Pos)      /*!< 0x00001000 */
#define RCC_APBENR2_SPI1EN               RCC_APBENR2_SPI1EN_Msk                 /*!< SPI clock enable */

#define RCC_APBENR2_UART3EN_Pos          (13U)
#define RCC_APBENR2_UART3EN_Msk          (0x1UL << RCC_APBENR2_UART3EN_Pos)     /*!< 0x00002000 */
#define RCC_APBENR2_UART3EN              RCC_APBENR2_UART3EN_Msk                /*!< UART3 clock enable */

#define RCC_APBENR2_TIM14EN_Pos          (15U)
#define RCC_APBENR2_TIM14EN_Msk          (0x1UL << RCC_APBENR2_TIM14EN_Pos)     /*!< 0x00008000 */
#define RCC_APBENR2_TIM14EN              RCC_APBENR2_TIM14EN_Msk                /*!< TIM14 clock enable */

#define RCC_APBENR2_ADCEN_Pos            (20U)
#define RCC_APBENR2_ADCEN_Msk            (0x1UL << RCC_APBENR2_ADCEN_Pos)       /*!< 0x00100000 */
#define RCC_APBENR2_ADCEN                RCC_APBENR2_ADCEN_Msk                  /*!< ADC clock enable */

#define RCC_APBENR2_COMP1EN_Pos          (21U)
#define RCC_APBENR2_COMP1EN_Msk          (0x1UL << RCC_APBENR2_COMP1EN_Pos)     /*!< 0x00200000 */
#define RCC_APBENR2_COMP1EN              RCC_APBENR2_COMP1EN_Msk                /*!< COMP1 clock enable */

#define RCC_APBENR2_COMP2EN_Pos          (22U)
#define RCC_APBENR2_COMP2EN_Msk          (0x1UL << RCC_APBENR2_COMP2EN_Pos)     /*!< 0x00400000 */
#define RCC_APBENR2_COMP2EN              RCC_APBENR2_COMP2EN_Msk                /*!< COMP2 clock enable */

#define RCC_APBENR2_TKEN_Pos             (25U)
#define RCC_APBENR2_TKEN_Msk             (0x1UL << RCC_APBENR2_TKEN_Pos)        /*!< 0x02000000 */
#define RCC_APBENR2_TKEN                 RCC_APBENR2_TKEN_Msk                   /*!< TK clock enable */

/********************  Bit definition for RCC_CCIPR register  ******************/
#define RCC_CCIPR_TIMCLK_CTRL_Pos        (0U)
#define RCC_CCIPR_TIMCLK_CTRL_Msk        (0x1UL << RCC_CCIPR_TIMCLK_CTRL_Pos)   /*!< 0x00000001 */
#define RCC_CCIPR_TIMCLK_CTRL            RCC_CCIPR_TIMCLK_CTRL_Msk              /*!< Timer PCLK control  */

#define RCC_CCIPR_COMP1SEL_Pos           (8U)
#define RCC_CCIPR_COMP1SEL_Msk           (0x1UL << RCC_CCIPR_COMP1SEL_Pos)      /*!< 0x00000100 */
#define RCC_CCIPR_COMP1SEL               RCC_CCIPR_COMP1SEL_Msk                 /*!< COMP1 clock select */

#define RCC_CCIPR_COMP2SEL_Pos           (9U)
#define RCC_CCIPR_COMP2SEL_Msk           (0x1UL << RCC_CCIPR_COMP2SEL_Pos)      /*!< 0x00000200 */
#define RCC_CCIPR_COMP2SEL               RCC_CCIPR_COMP2SEL_Msk                 /*!< COMP2 clock select */

#define RCC_CCIPR_IWDGSEL_Pos            (12U)
#define RCC_CCIPR_IWDGSEL_Msk            (0x1UL << RCC_CCIPR_IWDGSEL_Pos)       /*!< 0x00001000 */
#define RCC_CCIPR_IWDGSEL                RCC_CCIPR_IWDGSEL_Msk                  /*!< IWDG clock select */

#define RCC_CCIPR_ADC_CKMODE_Pos         (20U)
#define RCC_CCIPR_ADC_CKMODE_Msk         (0xFUL << RCC_CCIPR_ADC_CKMODE_Pos)    /*!< 0x000F0000 */
#define RCC_CCIPR_ADC_CKMODE             RCC_CCIPR_ADC_CKMODE_Msk               /*!< ADC clock mode select */
#define RCC_CCIPR_ADC_CKMODE_0           (0x1UL << RCC_CCIPR_ADC_CKMODE_Pos)    /*!< 0x00010000 */
#define RCC_CCIPR_ADC_CKMODE_1           (0x2UL << RCC_CCIPR_ADC_CKMODE_Pos)    /*!< 0x00020000 */
#define RCC_CCIPR_ADC_CKMODE_2           (0x4UL << RCC_CCIPR_ADC_CKMODE_Pos)    /*!< 0x00040000 */
#define RCC_CCIPR_ADC_CKMODE_3           (0x8UL << RCC_CCIPR_ADC_CKMODE_Pos)    /*!< 0x00080000 */

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos               (0U)
#define RCC_BDCR_LSEON_Msk               (0x1UL << RCC_BDCR_LSEON_Pos)          /*!< 0x00000001 */
#define RCC_BDCR_LSEON                   RCC_BDCR_LSEON_Msk                     /*!< LSE enable */

#define RCC_BDCR_LSERDY_Pos              (1U)
#define RCC_BDCR_LSERDY_Msk              (0x1UL << RCC_BDCR_LSERDY_Pos)         /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                  RCC_BDCR_LSERDY_Msk                    /*!< LSE ready */

#define RCC_BDCR_LSEBYP_Pos              (2U)
#define RCC_BDCR_LSEBYP_Msk              (0x1UL << RCC_BDCR_LSEBYP_Pos)         /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                  RCC_BDCR_LSEBYP_Msk                    /*!< LSE bypass */

#define RCC_BDCR_LSECSSON_Pos            (5U)
#define RCC_BDCR_LSECSSON_Msk            (0x1UL << RCC_BDCR_LSECSSON_Pos)       /*!< 0x00000020 */
#define RCC_BDCR_LSECSSON                RCC_BDCR_LSECSSON_Msk                  /*!< LSE CSS enable */

#define RCC_BDCR_LSECSSD_Pos             (6U)
#define RCC_BDCR_LSECSSD_Msk             (0x1UL << RCC_BDCR_LSECSSD_Pos)        /*!< 0x00000040 */
#define RCC_BDCR_LSECSSD                 RCC_BDCR_LSECSSD_Msk                   /*!< LSE CSS defeat */

#define RCC_BDCR_RTCSEL_Pos              (8U)
#define RCC_BDCR_RTCSEL_Msk              (0x3UL << RCC_BDCR_RTCSEL_Pos)         /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                  RCC_BDCR_RTCSEL_Msk                    /*!< RTC clock select */
#define RCC_BDCR_RTCSEL_0                (0x1UL << RCC_BDCR_RTCSEL_Pos)         /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                (0x2UL << RCC_BDCR_RTCSEL_Pos)         /*!< 0x00000200 */

#define RCC_BDCR_RTC_HSEDIV_SEL_Pos      (10U)
#define RCC_BDCR_RTC_HSEDIV_SEL_Msk      (0x3UL << RCC_BDCR_RTC_HSEDIV_SEL_Pos) /*!< 0x000000C0 */
#define RCC_BDCR_RTC_HSEDIV_SEL          RCC_BDCR_RTC_HSEDIV_SEL_Msk            /*!< RTC clock divider */
#define RCC_BDCR_RTC_HSEDIV_SEL_0        (0x1UL << RCC_BDCR_RTC_HSEDIV_SEL_Pos) /*!< 0x00000040 */
#define RCC_BDCR_RTC_HSEDIV_SEL_1        (0x2UL << RCC_BDCR_RTC_HSEDIV_SEL_Pos) /*!< 0x00000080 */

#define RCC_BDCR_RTCEN_Pos               (15U)
#define RCC_BDCR_RTCEN_Msk               (0x1UL << RCC_BDCR_RTCEN_Pos)          /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                   RCC_BDCR_RTCEN_Msk                     /*!< RTC clock enable */

#define RCC_BDCR_BDRST_Pos               (16U)
#define RCC_BDCR_BDRST_Msk               (0x1UL << RCC_BDCR_BDRST_Pos)          /*!< 0x00010000 */
#define RCC_BDCR_BDRST                   RCC_BDCR_BDRST_Msk                     /*!< RTC reset */

#define RCC_BDCR_LSCSEL_Pos              (25U)
#define RCC_BDCR_LSCSEL_Msk              (0x1UL << RCC_BDCR_LSCSEL_Pos)         /*!< 0x02000000 */
#define RCC_BDCR_LSCSEL                  RCC_BDCR_LSCSEL_Msk                    /*!< LOW speed clock select */

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                (0U)
#define RCC_CSR_LSION_Msk                (0x1UL << RCC_CSR_LSION_Pos)           /*!< 0x00000001 */
#define RCC_CSR_LSION                    RCC_CSR_LSION_Msk                      /*!< LSI enbale */

#define RCC_CSR_LSIRDY_Pos               (1U)
#define RCC_CSR_LSIRDY_Msk               (0x1UL << RCC_CSR_LSIRDY_Pos)          /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                   RCC_CSR_LSIRDY_Msk                     /*!< LSI ready flag */

#define RCC_CSR_NRST_FLTDIS_Pos          (8U)
#define RCC_CSR_NRST_FLTDIS_Msk          (0x1UL << RCC_CSR_NRST_FLTDIS_Pos)     /*!< 0x00000100 */
#define RCC_CSR_NRST_FLTDIS              RCC_CSR_NRST_FLTDIS_Msk                /*!< NRST filter diable */

#define RCC_CSR_BORRSTF_Pos              (9U)
#define RCC_CSR_BORRSTF_Msk              (0x1UL << RCC_CSR_BORRSTF_Pos)         /*!< 0x00000200 */
#define RCC_CSR_BORRSTF                  RCC_CSR_BORRSTF_Msk                    /*!< BOR reset flag */

#define RCC_CSR_RMVF_Pos                 (23U)
#define RCC_CSR_RMVF_Msk                 (0x1UL << RCC_CSR_RMVF_Pos)            /*!< 0x00800000 */
#define RCC_CSR_RMVF                     RCC_CSR_RMVF_Msk                       /*!< Remove reset flag */

#define RCC_CSR_OBLRSTF_Pos              (25U)
#define RCC_CSR_OBLRSTF_Msk              (0x1UL << RCC_CSR_OBLRSTF_Pos)         /*!< 0x02000000 */
#define RCC_CSR_OBLRSTF                  RCC_CSR_OBLRSTF_Msk                    /*!< Option byte reset flag */

#define RCC_CSR_PINRSTF_Pos              (26U)
#define RCC_CSR_PINRSTF_Msk              (0x1UL << RCC_CSR_PINRSTF_Pos)         /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                  RCC_CSR_PINRSTF_Msk                    /*!< NRST pin reset flag */

#define RCC_CSR_PORRSTF_Pos              (27U)
#define RCC_CSR_PORRSTF_Msk              (0x1UL << RCC_CSR_PORRSTF_Pos)         /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                  RCC_CSR_PORRSTF_Msk                    /*!< POR reset flag */

#define RCC_CSR_SFTRSTF_Pos              (28U)
#define RCC_CSR_SFTRSTF_Msk              (0x1UL << RCC_CSR_SFTRSTF_Pos)         /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                  RCC_CSR_SFTRSTF_Msk                    /*!< Soft reset flag */

#define RCC_CSR_IWDGRSTF_Pos             (29U)
#define RCC_CSR_IWDGRSTF_Msk             (0x1UL << RCC_CSR_IWDGRSTF_Pos)        /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                 RCC_CSR_IWDGRSTF_Msk                   /*!< IWDG reset flag */

/******************************************************************************/
/*                                                                            */
/*                             Real-Time Clock (RTC)                          */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for RTC_CRH register  ********************/
#define RTC_CRH_SECIE_Pos                   (0U)
#define RTC_CRH_SECIE_Msk                   (0x1UL << RTC_CRH_SECIE_Pos)        /*!< 0x00000001 */
#define RTC_CRH_SECIE                       RTC_CRH_SECIE_Msk                   /*!< Second Interrupt Enable */

#define RTC_CRH_ALRIE_Pos                   (1U)
#define RTC_CRH_ALRIE_Msk                   (0x1UL << RTC_CRH_ALRIE_Pos)        /*!< 0x00000002 */
#define RTC_CRH_ALRIE                       RTC_CRH_ALRIE_Msk                   /*!< Alarm Interrupt Enable */

#define RTC_CRH_OWIE_Pos                    (2U)
#define RTC_CRH_OWIE_Msk                    (0x1UL << RTC_CRH_OWIE_Pos)         /*!< 0x00000004 */
#define RTC_CRH_OWIE                        RTC_CRH_OWIE_Msk                    /*!< Overflow Interrupt Enable */

#define RTC_CRH_TK_SECIE_Pos                (3U)
#define RTC_CRH_TK_SECIE_Msk                (0x1UL << RTC_CRH_TK_SECIE_Pos)     /*!< 0x00000008 */
#define RTC_CRH_TK_SECIE                    RTC_CRH_TK_SECIE_Msk                /*!< TK Second Interrupt Enable */

#define RTC_CRH_TK_ALRIE_Pos                (4U)
#define RTC_CRH_TK_ALRIE_Msk                (0x1UL << RTC_CRH_TK_ALRIE_Pos)     /*!< 0x00000010 */
#define RTC_CRH_TK_ALRIE                    RTC_CRH_TK_ALRIE_Msk                /*!< TK Alarm Interrupt Enable */

#define RTC_CRH_TK_OWIE_Pos                 (5U)
#define RTC_CRH_TK_OWIE_Msk                 (0x1UL << RTC_CRH_TK_OWIE_Pos)      /*!< 0x00000020 */
#define RTC_CRH_TK_OWIE                     RTC_CRH_TK_OWIE_Msk                 /*!< TK Overflow Interrupt Enable */

/*******************  Bit definition for RTC_CRL register  ********************/
#define RTC_CRL_SECF_Pos                    (0U)
#define RTC_CRL_SECF_Msk                    (0x1UL << RTC_CRL_SECF_Pos)         /*!< 0x00000001 */
#define RTC_CRL_SECF                        RTC_CRL_SECF_Msk                    /*!< Second Flag */

#define RTC_CRL_ALRF_Pos                    (1U)
#define RTC_CRL_ALRF_Msk                    (0x1UL << RTC_CRL_ALRF_Pos)         /*!< 0x00000002 */
#define RTC_CRL_ALRF                        RTC_CRL_ALRF_Msk                    /*!< Alarm Flag */

#define RTC_CRL_OWF_Pos                     (2U)
#define RTC_CRL_OWF_Msk                     (0x1UL << RTC_CRL_OWF_Pos)          /*!< 0x00000004 */
#define RTC_CRL_OWF                         RTC_CRL_OWF_Msk                     /*!< OverfloW Flag */

#define RTC_CRL_RSF_Pos                     (3U)
#define RTC_CRL_RSF_Msk                     (0x1UL << RTC_CRL_RSF_Pos)          /*!< 0x00000008 */
#define RTC_CRL_RSF                         RTC_CRL_RSF_Msk                     /*!< Registers Synchronized Flag */

#define RTC_CRL_CNF_Pos                     (4U)
#define RTC_CRL_CNF_Msk                     (0x1UL << RTC_CRL_CNF_Pos)          /*!< 0x00000010 */
#define RTC_CRL_CNF                         RTC_CRL_CNF_Msk                     /*!< Configuration Flag */

#define RTC_CRL_RTOFF_Pos                   (5U)
#define RTC_CRL_RTOFF_Msk                   (0x1UL << RTC_CRL_RTOFF_Pos)        /*!< 0x00000020 */
#define RTC_CRL_RTOFF                       RTC_CRL_RTOFF_Msk                   /*!< RTC operation OFF */

/*******************  Bit definition for RTC_PRLH register  *******************/
#define RTC_PRLH_PRL_Pos                    (0U)
#define RTC_PRLH_PRL_Msk                    (0xFUL << RTC_PRLH_PRL_Pos)         /*!< 0x0000000F */
#define RTC_PRLH_PRL                        RTC_PRLH_PRL_Msk                    /*!< RTC Prescaler Reload Value High */

/*******************  Bit definition for RTC_PRLL register  *******************/
#define RTC_PRLL_PRL_Pos                    (0U)
#define RTC_PRLL_PRL_Msk                    (0xFFFFUL << RTC_PRLL_PRL_Pos)      /*!< 0x0000FFFF */
#define RTC_PRLL_PRL                        RTC_PRLL_PRL_Msk                    /*!< RTC Prescaler Reload Value Low */

/*******************  Bit definition for RTC_DIVH register  *******************/
#define RTC_DIVH_RTC_DIV_Pos                (0U)
#define RTC_DIVH_RTC_DIV_Msk                (0xFUL << RTC_DIVH_RTC_DIV_Pos)     /*!< 0x0000000F */
#define RTC_DIVH_RTC_DIV                    RTC_DIVH_RTC_DIV_Msk                /*!< RTC Clock Divider High */

/*******************  Bit definition for RTC_DIVL register  *******************/
#define RTC_DIVL_RTC_DIV_Pos                (0U)
#define RTC_DIVL_RTC_DIV_Msk                (0xFFFFUL << RTC_DIVL_RTC_DIV_Pos)  /*!< 0x0000FFFF */
#define RTC_DIVL_RTC_DIV                    RTC_DIVL_RTC_DIV_Msk                /*!< RTC Clock Divider Low */

/*******************  Bit definition for RTC_CNTH register  *******************/
#define RTC_CNTH_RTC_CNT_Pos                (0U)
#define RTC_CNTH_RTC_CNT_Msk                (0xFFFFUL << RTC_CNTH_RTC_CNT_Pos)  /*!< 0x0000FFFF */
#define RTC_CNTH_RTC_CNT                    RTC_CNTH_RTC_CNT_Msk                /*!< RTC Counter High */

/*******************  Bit definition for RTC_CNTL register  *******************/
#define RTC_CNTL_RTC_CNT_Pos                (0U)
#define RTC_CNTL_RTC_CNT_Msk                (0xFFFFUL << RTC_CNTL_RTC_CNT_Pos)  /*!< 0x0000FFFF */
#define RTC_CNTL_RTC_CNT                    RTC_CNTL_RTC_CNT_Msk                /*!< RTC Counter Low */

/*******************  Bit definition for RTC_ALRH register  *******************/
#define RTC_ALRH_RTC_ALR_Pos                (0U)
#define RTC_ALRH_RTC_ALR_Msk                (0xFFFFUL << RTC_ALRH_RTC_ALR_Pos)  /*!< 0x0000FFFF */
#define RTC_ALRH_RTC_ALR                    RTC_ALRH_RTC_ALR_Msk                /*!< RTC Alarm High */

/*******************  Bit definition for RTC_ALRL register  *******************/
#define RTC_ALRL_RTC_ALR_Pos                (0U)
#define RTC_ALRL_RTC_ALR_Msk                (0xFFFFUL << RTC_ALRL_RTC_ALR_Pos)  /*!< 0x0000FFFF */
#define RTC_ALRL_RTC_ALR                    RTC_ALRL_RTC_ALR_Msk                /*!< RTC Alarm Low */

/*******************  Bit definition for BKP_RTCCR register  ******************/
#define BKP_RTCCR_CAL_Pos                   (0U)
#define BKP_RTCCR_CAL_Msk                   (0x7FUL << BKP_RTCCR_CAL_Pos)       /*!< 0x0000007F */
#define BKP_RTCCR_CAL                       BKP_RTCCR_CAL_Msk                   /*!< Calibration value */
#define BKP_RTCCR_CAL_0                     (0x01UL << BKP_RTCCR_CAL_Pos)
#define BKP_RTCCR_CAL_1                     (0x02UL << BKP_RTCCR_CAL_Pos)
#define BKP_RTCCR_CAL_2                     (0x04UL << BKP_RTCCR_CAL_Pos)
#define BKP_RTCCR_CAL_3                     (0x08UL << BKP_RTCCR_CAL_Pos)
#define BKP_RTCCR_CAL_4                     (0x10UL << BKP_RTCCR_CAL_Pos)
#define BKP_RTCCR_CAL_5                     (0x20UL << BKP_RTCCR_CAL_Pos)
#define BKP_RTCCR_CAL_6                     (0x40UL << BKP_RTCCR_CAL_Pos)

#define BKP_RTCCR_CCO_Pos                   (7U)
#define BKP_RTCCR_CCO_Msk                   (0x1UL << BKP_RTCCR_CCO_Pos)        /*!< 0x00000080 */
#define BKP_RTCCR_CCO                       BKP_RTCCR_CCO_Msk                   /*!< Calibration Clock Output */

#define BKP_RTCCR_ASOE_Pos                  (8U)
#define BKP_RTCCR_ASOE_Msk                  (0x1UL << BKP_RTCCR_ASOE_Pos)       /*!< 0x00000100 */
#define BKP_RTCCR_ASOE                      BKP_RTCCR_ASOE_Msk                  /*!< Alarm or Second Output Enable */

#define BKP_RTCCR_ASOS_Pos                  (9U)
#define BKP_RTCCR_ASOS_Msk                  (0x1UL << BKP_RTCCR_ASOS_Pos)       /*!< 0x00000200 */
#define BKP_RTCCR_ASOS                      BKP_RTCCR_ASOS_Msk                  /*!< Alarm or Second Output Selection */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface (SPI)                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos                    (0U)
#define SPI_CR1_CPHA_Msk                    (0x1UL << SPI_CR1_CPHA_Pos)         /*!< 0x00000001 */
#define SPI_CR1_CPHA                        SPI_CR1_CPHA_Msk                    /*!< Clock Phase */

#define SPI_CR1_CPOL_Pos                    (1U)
#define SPI_CR1_CPOL_Msk                    (0x1UL << SPI_CR1_CPOL_Pos)         /*!< 0x00000002 */
#define SPI_CR1_CPOL                        SPI_CR1_CPOL_Msk                    /*!< Clock Polarity */

#define SPI_CR1_MSTR_Pos                    (2U)
#define SPI_CR1_MSTR_Msk                    (0x1UL << SPI_CR1_MSTR_Pos)         /*!< 0x00000004 */
#define SPI_CR1_MSTR                        SPI_CR1_MSTR_Msk                    /*!< Master Selection */

#define SPI_CR1_BR_Pos                      (3U)
#define SPI_CR1_BR_Msk                      (0x7UL << SPI_CR1_BR_Pos)           /*!< 0x00000038 */
#define SPI_CR1_BR                          SPI_CR1_BR_Msk                      /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                        (0x1UL << SPI_CR1_BR_Pos)           /*!< 0x00000008 */
#define SPI_CR1_BR_1                        (0x2UL << SPI_CR1_BR_Pos)           /*!< 0x00000010 */
#define SPI_CR1_BR_2                        (0x4UL << SPI_CR1_BR_Pos)           /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos                     (6U)
#define SPI_CR1_SPE_Msk                     (0x1UL << SPI_CR1_SPE_Pos)          /*!< 0x00000040 */
#define SPI_CR1_SPE                         SPI_CR1_SPE_Msk                     /*!< SPI Enable */

#define SPI_CR1_LSBFIRST_Pos                (7U)
#define SPI_CR1_LSBFIRST_Msk                (0x1UL << SPI_CR1_LSBFIRST_Pos)     /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST                    SPI_CR1_LSBFIRST_Msk                /*!< Frame Format */

#define SPI_CR1_SSI_Pos                     (8U)
#define SPI_CR1_SSI_Msk                     (0x1UL << SPI_CR1_SSI_Pos)          /*!< 0x00000100 */
#define SPI_CR1_SSI                         SPI_CR1_SSI_Msk                     /*!< Internal slave select */

#define SPI_CR1_SSM_Pos                     (9U)
#define SPI_CR1_SSM_Msk                     (0x1UL << SPI_CR1_SSM_Pos)          /*!< 0x00000200 */
#define SPI_CR1_SSM                         SPI_CR1_SSM_Msk                     /*!< Software slave management */

#define SPI_CR1_RXONLY_Pos                  (10U)
#define SPI_CR1_RXONLY_Msk                  (0x1UL << SPI_CR1_RXONLY_Pos)       /*!< 0x00000400 */
#define SPI_CR1_RXONLY                      SPI_CR1_RXONLY_Msk                  /*!< Receive only */

#define SPI_CR1_DFF_Pos                     (11U)
#define SPI_CR1_DFF_Msk                     (0x1UL << SPI_CR1_DFF_Pos)          /*!< 0x00000800 */
#define SPI_CR1_DFF                         SPI_CR1_DFF_Msk                     /*!< Data frame format */

#define SPI_CR1_BIDIOE_Pos                  (14U)
#define SPI_CR1_BIDIOE_Msk                  (0x1UL << SPI_CR1_BIDIOE_Pos)       /*!< 0x00004000 */
#define SPI_CR1_BIDIOE                      SPI_CR1_BIDIOE_Msk                  /*!< Output enable in bidirectional mode */

#define SPI_CR1_BIDIMODE_Pos                (15U)
#define SPI_CR1_BIDIMODE_Msk                (0x1UL << SPI_CR1_BIDIMODE_Pos)     /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE                    SPI_CR1_BIDIMODE_Msk                /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_SSOE_Pos                    (2U)
#define SPI_CR2_SSOE_Msk                    (0x1UL << SPI_CR2_SSOE_Pos)         /*!< 0x00000004 */
#define SPI_CR2_SSOE                        SPI_CR2_SSOE_Msk                    /*!< SS Output Enable */

#define SPI_CR2_CLRTXFIFO_Pos               (4U)
#define SPI_CR2_CLRTXFIFO_Msk               (0x1UL << SPI_CR2_CLRTXFIFO_Pos)    /*!< 0x00000010 */
#define SPI_CR2_CLRTXFIFO                   SPI_CR2_CLRTXFIFO_Msk

#define SPI_CR2_ERRIE_Pos                   (5U)
#define SPI_CR2_ERRIE_Msk                   (0x1UL << SPI_CR2_ERRIE_Pos)        /*!< 0x00000020 */
#define SPI_CR2_ERRIE                       SPI_CR2_ERRIE_Msk                   /*!< Error Interrupt Enable */

#define SPI_CR2_RXNEIE_Pos                  (6U)
#define SPI_CR2_RXNEIE_Msk                  (0x1UL << SPI_CR2_RXNEIE_Pos)       /*!< 0x00000040 */
#define SPI_CR2_RXNEIE                      SPI_CR2_RXNEIE_Msk                  /*!< RX buffer Not Empty Interrupt Enable */

#define SPI_CR2_TXEIE_Pos                   (7U)
#define SPI_CR2_TXEIE_Msk                   (0x1UL << SPI_CR2_TXEIE_Pos)        /*!< 0x00000080 */
#define SPI_CR2_TXEIE                       SPI_CR2_TXEIE_Msk                   /*!< Tx buffer Empty Interrupt Enable */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos                     (0U)
#define SPI_SR_RXNE_Msk                     (0x1UL << SPI_SR_RXNE_Pos)          /*!< 0x00000001 */
#define SPI_SR_RXNE                         SPI_SR_RXNE_Msk                     /*!< Receive buffer Not Empty */

#define SPI_SR_TXE_Pos                      (1U)
#define SPI_SR_TXE_Msk                      (0x1UL << SPI_SR_TXE_Pos)           /*!< 0x00000002 */
#define SPI_SR_TXE                          SPI_SR_TXE_Msk                      /*!< Transmit buffer Empty */

#define SPI_SR_MODF_Pos                     (5U)
#define SPI_SR_MODF_Msk                     (0x1UL << SPI_SR_MODF_Pos)          /*!< 0x00000020 */
#define SPI_SR_MODF                         SPI_SR_MODF_Msk                     /*!< Mode fault */

#define SPI_SR_OVR_Pos                      (6U)
#define SPI_SR_OVR_Msk                      (0x1UL << SPI_SR_OVR_Pos)           /*!< 0x00000040 */
#define SPI_SR_OVR                          SPI_SR_OVR_Msk                      /*!< Overrun flag */

#define SPI_SR_BSY_Pos                      (7U)
#define SPI_SR_BSY_Msk                      (0x1UL << SPI_SR_BSY_Pos)           /*!< 0x00000080 */
#define SPI_SR_BSY                          SPI_SR_BSY_Msk                      /*!< Busy flag */

#define SPI_SR_FRLVL_Pos                    (9U)
#define SPI_SR_FRLVL_Msk                    (0x3UL << SPI_SR_FRLVL_Pos)         /*!< 0x00000600 */
#define SPI_SR_FRLVL                        SPI_SR_FRLVL_Msk                    /*!< FIFO Reception Level */
#define SPI_SR_FRLVL_0                      (0x1UL << SPI_SR_FRLVL_Pos)         /*!< 0x00000200 */
#define SPI_SR_FRLVL_1                      (0x2UL << SPI_SR_FRLVL_Pos)         /*!< 0x00000400 */

#define SPI_SR_FTLVL_Pos                    (11U)
#define SPI_SR_FTLVL_Msk                    (0x3UL << SPI_SR_FTLVL_Pos)         /*!< 0x00001800 */
#define SPI_SR_FTLVL                        SPI_SR_FTLVL_Msk                    /*!< FIFO Transmission Level */
#define SPI_SR_FTLVL_0                      (0x1UL << SPI_SR_FTLVL_Pos)         /*!< 0x00000800 */
#define SPI_SR_FTLVL_1                      (0x2UL << SPI_SR_FTLVL_Pos)         /*!< 0x00001000 */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos                       (0U)
#define SPI_DR_DR_Msk                       (0xFFFFUL << SPI_DR_DR_Pos)         /*!< 0x0000FFFF */
#define SPI_DR_DR                           SPI_DR_DR_Msk                       /*!< Data Register */

/******************************************************************************/
/*                                                                            */
/*                       System Configuration (SYSCFG)                        */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for SYSCFG_CFGR1 register  ****************/
#define SYSCFG_CFGR1_MEM_MODE_Pos                 (0U)
#define SYSCFG_CFGR1_MEM_MODE_Msk                 (0x3UL << SYSCFG_CFGR1_MEM_MODE_Pos)     /*!< 0x00000003 */
#define SYSCFG_CFGR1_MEM_MODE                     SYSCFG_CFGR1_MEM_MODE_Msk                /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_CFGR1_MEM_MODE_0                   (0x1UL << SYSCFG_CFGR1_MEM_MODE_Pos)     /*!< 0x00000001 */
#define SYSCFG_CFGR1_MEM_MODE_1                   (0x2UL << SYSCFG_CFGR1_MEM_MODE_Pos)     /*!< 0x00000002 */

#define SYSCFG_CFGR1_TIM1_IC1_SRC_Pos             (2U)
#define SYSCFG_CFGR1_TIM1_IC1_SRC_Msk             (0x3UL << SYSCFG_CFGR1_TIM1_IC1_SRC_Pos) /*!< 0x0000000C */
#define SYSCFG_CFGR1_TIM1_IC1_SRC                 SYSCFG_CFGR1_TIM1_IC1_SRC_Msk            /*!< TIM1 CH1 input source */
#define SYSCFG_CFGR1_TIM1_IC1_SRC_0               (0x1UL << SYSCFG_CFGR1_TIM1_IC1_SRC_Pos) /*!< 0x00000004 */
#define SYSCFG_CFGR1_TIM1_IC1_SRC_1               (0x2UL << SYSCFG_CFGR1_TIM1_IC1_SRC_Pos) /*!< 0x00000008 */

#define SYSCFG_CFGR1_ETR_SRC_TIM1_Pos             (8U)
#define SYSCFG_CFGR1_ETR_SRC_TIM1_Msk             (0x3UL << SYSCFG_CFGR1_ETR_SRC_TIM1_Pos) /*!< 0x00000300 */
#define SYSCFG_CFGR1_ETR_SRC_TIM1                 SYSCFG_CFGR1_ETR_SRC_TIM1_Msk            /*!< TIM1 ETR input source */
#define SYSCFG_CFGR1_ETR_SRC_TIM1_0               (0x1UL << SYSCFG_CFGR1_ETR_SRC_TIM1_Pos) /*!< 0x00000100 */
#define SYSCFG_CFGR1_ETR_SRC_TIM1_1               (0x2UL << SYSCFG_CFGR1_ETR_SRC_TIM1_Pos) /*!< 0x00000200 */

/******************  Bit definition for SYSCFG_CFGR2 register  ****************/
#define SYSCFG_CFGR2_LOCKUP_LOCK_Pos              (0U)
#define SYSCFG_CFGR2_LOCKUP_LOCK_Msk              (0x1UL << SYSCFG_CFGR2_LOCKUP_LOCK_Pos)          /*!< 0x00000001 */
#define SYSCFG_CFGR2_LOCKUP_LOCK                  SYSCFG_CFGR2_LOCKUP_LOCK_Msk                     /*!< Enables and locks the LOCKUP_LOCK (Hardfault) output of CortexM0 with Break Input of TIMER1 */

#define SYSCFG_CFGR2_COMP1_BRK_TIM1_Pos           (3U)
#define SYSCFG_CFGR2_COMP1_BRK_TIM1_Msk           (0x1UL << SYSCFG_CFGR2_COMP1_BRK_TIM1_Pos)       /*!< 0x00000008 */
#define SYSCFG_CFGR2_COMP1_BRK_TIM1               SYSCFG_CFGR2_COMP1_BRK_TIM1_Msk                  /*!< COMP1_BRK_TIM1 */

#define SYSCFG_CFGR2_COMP2_BRK_TIM1_Pos           (4U)
#define SYSCFG_CFGR2_COMP2_BRK_TIM1_Msk           (0x1UL << SYSCFG_CFGR2_COMP2_BRK_TIM1_Pos)       /*!< 0x00000010 */
#define SYSCFG_CFGR2_COMP2_BRK_TIM1               SYSCFG_CFGR2_COMP2_BRK_TIM1_Msk                  /*!< COMP2_BRK_TIM1 */

#define SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1_Pos     (9U)
#define SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1_Msk     (0x1UL << SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1_Pos) /*!< 0x00000200 */
#define SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1         SYSCFG_CFGR2_COMP1_OCREF_CLR_TIM1_Msk            /*!< COMP1 output clear TIM1 ocref */

#define SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1_Pos     (11U)
#define SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1_Msk     (0x1UL << SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1_Pos) /*!< 0x00000800 */
#define SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1         SYSCFG_CFGR2_COMP2_OCREF_CLR_TIM1_Msk            /*!< COMP2 output clear TIM1 ocref */

/*****************  Bit definition for PA_ENS register  ************************/
#define SYSCFG_PAENS_Pos                          (0U)
#define SYSCFG_PAENS_Msk                          (0xFFFFUL << SYSCFG_PAENS_Pos)      /*!< 0x0000FFFF */
#define SYSCFG_PAENS                              SYSCFG_PAENS_Msk
#define SYSCFG_PAENS_0                            (0x1UL << SYSCFG_PAENS_Pos)         /*!< 0x00000001 */
#define SYSCFG_PAENS_1                            (0x2UL << SYSCFG_PAENS_Pos)         /*!< 0x00000002 */  
#define SYSCFG_PAENS_2                            (0x4UL << SYSCFG_PAENS_Pos)         /*!< 0x00000004 */  
#define SYSCFG_PAENS_3                            (0x8UL << SYSCFG_PAENS_Pos)         /*!< 0x00000008 */  
#define SYSCFG_PAENS_4                            (0x10UL << SYSCFG_PAENS_Pos)        /*!< 0x00000010 */  
#define SYSCFG_PAENS_5                            (0x20UL << SYSCFG_PAENS_Pos)        /*!< 0x00000020 */  
#define SYSCFG_PAENS_6                            (0x40UL << SYSCFG_PAENS_Pos)        /*!< 0x00000040 */  
#define SYSCFG_PAENS_7                            (0x80UL << SYSCFG_PAENS_Pos)        /*!< 0x00000080 */  
#define SYSCFG_PAENS_8                            (0x100UL << SYSCFG_PAENS_Pos)       /*!< 0x00000100 */  
#define SYSCFG_PAENS_9                            (0x200UL << SYSCFG_PAENS_Pos)       /*!< 0x00000200 */  
#define SYSCFG_PAENS_10                           (0x400UL << SYSCFG_PAENS_Pos)       /*!< 0x00000400 */  
#define SYSCFG_PAENS_11                           (0x800UL << SYSCFG_PAENS_Pos)       /*!< 0x00000800 */  
#define SYSCFG_PAENS_12                           (0x1000UL << SYSCFG_PAENS_Pos)      /*!< 0x00001000 */  
#define SYSCFG_PAENS_13                           (0x2000UL << SYSCFG_PAENS_Pos)      /*!< 0x00002000 */                             
#define SYSCFG_PAENS_14                           (0x4000UL << SYSCFG_PAENS_Pos)      /*!< 0x00004000 */  
#define SYSCFG_PAENS_15                           (0x8000UL << SYSCFG_PAENS_Pos)      /*!< 0x00008000 */  

/*****************  Bit definition for PB_ENS register  ************************/
#define SYSCFG_PBENS_Pos                          (0U)
#define SYSCFG_PBENS_Msk                          (0xFUL << SYSCFG_PBENS_Pos)         /*!< 0x0000000F */
#define SYSCFG_PBENS                              SYSCFG_PBENS_Msk
#define SYSCFG_PBENS_0                            (0x1UL << SYSCFG_PBENS_Pos)         /*!< 0x00000001 */
#define SYSCFG_PBENS_1                            (0x2UL << SYSCFG_PBENS_Pos)         /*!< 0x00000002 */  
#define SYSCFG_PBENS_2                            (0x4UL << SYSCFG_PBENS_Pos)         /*!< 0x00000004 */  
#define SYSCFG_PBENS_3                            (0x8UL << SYSCFG_PBENS_Pos)         /*!< 0x00000008 */  

/*****************  Bit definition for PF_ENS register  ************************/
#define SYSCFG_PFENS_Pos                          (0U)
#define SYSCFG_PFENS_Msk                          (0x3FUL << SYSCFG_PFENS_Pos)        /*!< 0x0000001F */
#define SYSCFG_PFENS                              SYSCFG_PFENS_Msk
#define SYSCFG_PFENS_0                            (0x1UL << SYSCFG_PFENS_Pos)         /*!< 0x00000001 */
#define SYSCFG_PFENS_1                            (0x2UL << SYSCFG_PFENS_Pos)         /*!< 0x00000002 */  
#define SYSCFG_PFENS_2                            (0x4UL << SYSCFG_PFENS_Pos)         /*!< 0x00000004 */  
#define SYSCFG_PFENS_3                            (0x8UL << SYSCFG_PFENS_Pos)         /*!< 0x00000008 */ 
#define SYSCFG_PFENS_4                            (0x10UL << SYSCFG_PFENS_Pos)        /*!< 0x00000010 */ 
#define SYSCFG_PFENS_5                            (0x20UL << SYSCFG_PFENS_Pos)        /*!< 0x00000020 */ 

/*****************  Bit definition for IOCFG register  ************************/
#define SYSCFG_IOCFG_PA_EIIC_Pos                  (0U)
#define SYSCFG_IOCFG_PA_EIIC_Msk                  (0x3UL << SYSCFG_IOCFG_PA_EIIC_Pos)         /*!< 0x0000003 */
#define SYSCFG_IOCFG_PA_EIIC                      SYSCFG_IOCFG_PA_EIIC_Msk                    /*!< PA_EIIC[1:0] bits */
#define SYSCFG_IOCFG_PA_EIIC_0                    (0x1UL << SYSCFG_IOCFG_PA_EIIC_Pos)         /*!< 0x00000001 */ 
#define SYSCFG_IOCFG_PA_EIIC_1                    (0x2UL << SYSCFG_IOCFG_PA_EIIC_Pos)         /*!< 0x00000002 */ 

#define SYSCFG_IOCFG_PF_EIIC_Pos                  (2U)
#define SYSCFG_IOCFG_PF_EIIC_Msk                  (0x7UL << SYSCFG_IOCFG_PF_EIIC_Pos)         /*!< 0x0000001C */
#define SYSCFG_IOCFG_PF_EIIC                      SYSCFG_IOCFG_PF_EIIC_Msk                    /*!< PF_EIIC[2:0] bits */
#define SYSCFG_IOCFG_PF_EIIC_0                    (0x1UL << SYSCFG_IOCFG_PF_EIIC_Pos)         /*!< 0x00000001 */ 
#define SYSCFG_IOCFG_PF_EIIC_1                    (0x2UL << SYSCFG_IOCFG_PF_EIIC_Pos)         /*!< 0x00000002 */ 
#define SYSCFG_IOCFG_PF_EIIC_2                    (0x4UL << SYSCFG_IOCFG_PF_EIIC_Pos)         /*!< 0x00000004 */

#define SYSCFG_IOCFG_PF_PU_IIC_Pos                (6U)
#define SYSCFG_IOCFG_PF_PU_IIC_Msk                (0x3UL << SYSCFG_IOCFG_PF_PU_IIC_Pos)       /*!< 0x000000C0 */
#define SYSCFG_IOCFG_PF_PU_IIC                    SYSCFG_IOCFG_PF_PU_IIC_Msk                  /*!< PF_PU_IIC[1:0] bits */
#define SYSCFG_IOCFG_PF_PU_IIC_0                  (0x1UL << SYSCFG_IOCFG_PF_PU_IIC_Pos)       /*!< 0x00000001 */ 
#define SYSCFG_IOCFG_PF_PU_IIC_1                  (0x2UL << SYSCFG_IOCFG_PF_PU_IIC_Pos)       /*!< 0x00000002 */ 

#define SYSCFG_IOCFG_PA_EHS_Pos                   (8U)
#define SYSCFG_IOCFG_PA_EHS_Msk                   (0x1FUL << SYSCFG_IOCFG_PA_EHS_Pos)         /*!< 0x00001F00 */
#define SYSCFG_IOCFG_PA_EHS                       SYSCFG_IOCFG_PA_EHS_Msk                     /*!< PA_EHS[4:0] bits */
#define SYSCFG_IOCFG_PA_EHS_0                     (0x1UL << SYSCFG_IOCFG_PA_EHS_Pos)          /*!< 0x00000001 */  
#define SYSCFG_IOCFG_PA_EHS_1                     (0x2UL << SYSCFG_IOCFG_PA_EHS_Pos)          /*!< 0x00000002 */  
#define SYSCFG_IOCFG_PA_EHS_2                     (0x4UL << SYSCFG_IOCFG_PA_EHS_Pos)          /*!< 0x00000004 */  
#define SYSCFG_IOCFG_PA_EHS_3                     (0x8UL << SYSCFG_IOCFG_PA_EHS_Pos)          /*!< 0x00000008 */  
#define SYSCFG_IOCFG_PA_EHS_4                     (0x10UL << SYSCFG_IOCFG_PA_EHS_Pos)         /*!< 0x00000010 */    

#define SYSCFG_IOCFG_PA_IODRVP_Pos                (13U)
#define SYSCFG_IOCFG_PA_IODRVP_Msk                (0x3UL << SYSCFG_IOCFG_PA_IODRVP_Pos)       /*!< 0x00006000 */
#define SYSCFG_IOCFG_PA_IODRVP                    SYSCFG_IOCFG_PA_IODRVP_Msk                  /*!< PA_IODRVP[1:0] bits */
#define SYSCFG_IOCFG_PA_IODRVP_0                  (0x1UL << SYSCFG_IOCFG_PA_IODRVP_Pos)       /*!< 0x00000001 */          
#define SYSCFG_IOCFG_PA_IODRVP_1                  (0x2UL << SYSCFG_IOCFG_PA_IODRVP_Pos)       /*!< 0x00000002 */ 

#define SYSCFG_IOCFG_PA_ENSEG_Pos                 (16U)
#define SYSCFG_IOCFG_PA_ENSEG_Msk                 (0x3FUL << SYSCFG_IOCFG_PA_ENSEG_Pos)       /*!< 0x003F0000 */
#define SYSCFG_IOCFG_PA_ENSEG                     SYSCFG_IOCFG_PA_ENSEG_Msk                   /*!< PA_ENSEG[5:0] bits */
#define SYSCFG_IOCFG_PA_ENSEG_0                   (0x1UL << SYSCFG_IOCFG_PA_ENSEG_Pos)        /*!< 0x00000001 */ 
#define SYSCFG_IOCFG_PA_ENSEG_1                   (0x2UL << SYSCFG_IOCFG_PA_ENSEG_Pos)        /*!< 0x00000002 */   
#define SYSCFG_IOCFG_PA_ENSEG_2                   (0x4UL << SYSCFG_IOCFG_PA_ENSEG_Pos)        /*!< 0x00000004 */   
#define SYSCFG_IOCFG_PA_ENSEG_3                   (0x8UL << SYSCFG_IOCFG_PA_ENSEG_Pos)        /*!< 0x00000008 */    
#define SYSCFG_IOCFG_PA_ENSEG_4                   (0x10UL << SYSCFG_IOCFG_PA_ENSEG_Pos)       /*!< 0x00000010 */   
#define SYSCFG_IOCFG_PA_ENSEG_5                   (0x20UL << SYSCFG_IOCFG_PA_ENSEG_Pos)       /*!< 0x00000020 */  

#define SYSCFG_IOCFG_PB_ENSEG_Pos                 (22U)
#define SYSCFG_IOCFG_PB_ENSEG_Msk                 (0x3UL << SYSCFG_IOCFG_PB_ENSEG_Pos)        /*!< 0x00C00000 */
#define SYSCFG_IOCFG_PB_ENSEG                     SYSCFG_IOCFG_PB_ENSEG_Msk                   /*!< PB_ENSEG[1:0] bits */
#define SYSCFG_IOCFG_PB_ENSEG_0                   (0x1UL << SYSCFG_IOCFG_PB_ENSEG_Pos)        /*!< 0x00000001 */ 
#define SYSCFG_IOCFG_PB_ENSEG_1                   (0x2UL << SYSCFG_IOCFG_PB_ENSEG_Pos)        /*!< 0x00000002 */ 

/*****************  Bit definition for PA_ANA2EN register  ************************/
#define SYSCFG_PA_ANA2EN_Pos                      (0U)
#define SYSCFG_PA_ANA2EN_Msk                      (0xFFFFUL << SYSCFG_PA_ANA2EN_Pos)          /*!< 0x0000FFFF */
#define SYSCFG_PA_ANA2EN                          SYSCFG_PA_ANA2EN_Msk                         

/*****************  Bit definition for PB_ANA2EN register  ************************/
#define SYSCFG_PB_ANA2EN_Pos                      (0U)
#define SYSCFG_PB_ANA2EN_Msk                      (0xFUL << SYSCFG_PB_ANA2EN_Pos)             /*!< 0x0000001F */
#define SYSCFG_PB_ANA2EN                          SYSCFG_PB_ANA2EN_Msk                         

/*****************  Bit definition for PF_ANA2EN register  ************************/
#define SYSCFG_PF_ANA2EN_Pos                      (0U)
#define SYSCFG_PF_ANA2EN_Msk                      (0x3FUL << SYSCFG_PF_ANA2EN_Pos)            /*!< 0x0000003F */
#define SYSCFG_PF_ANA2EN                          SYSCFG_PF_ANA2EN_Msk                         

/*****************  Bit definition for PA_IORP register  ************************/
#define SYSCFG_PA_IORP_PA0_IORP_Pos               (0U)
#define SYSCFG_PA_IORP_PA0_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA0_IORP_Pos)         /*!< 0x00000003 */
#define SYSCFG_PA_IORP_PA0_IORP                   SYSCFG_PA_IORP_PA0_IORP_Msk                    
#define SYSCFG_PA_IORP_PA0_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA0_IORP_Pos)         /*!< 0x00000001 */ 
#define SYSCFG_PA_IORP_PA0_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA0_IORP_Pos)         /*!< 0x00000002 */ 

#define SYSCFG_PA_IORP_PA1_IORP_Pos               (2U)
#define SYSCFG_PA_IORP_PA1_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA1_IORP_Pos)         /*!< 0x0000000C */
#define SYSCFG_PA_IORP_PA1_IORP                   SYSCFG_PA_IORP_PA1_IORP_Msk                    
#define SYSCFG_PA_IORP_PA1_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA1_IORP_Pos)         /*!< 0x00000004 */ 
#define SYSCFG_PA_IORP_PA1_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA1_IORP_Pos)         /*!< 0x00000008 */ 

#define SYSCFG_PA_IORP_PA2_IORP_Pos               (4U)
#define SYSCFG_PA_IORP_PA2_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA2_IORP_Pos)         /*!< 0x00000030 */
#define SYSCFG_PA_IORP_PA2_IORP                   SYSCFG_PA_IORP_PA2_IORP_Msk                    
#define SYSCFG_PA_IORP_PA2_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA2_IORP_Pos)         /*!< 0x00000010 */ 
#define SYSCFG_PA_IORP_PA2_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA2_IORP_Pos)         /*!< 0x00000020 */ 

#define SYSCFG_PA_IORP_PA3_IORP_Pos               (6U)
#define SYSCFG_PA_IORP_PA3_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA3_IORP_Pos)         /*!< 0x000000C0 */
#define SYSCFG_PA_IORP_PA3_IORP                   SYSCFG_PA_IORP_PA3_IORP_Msk                    
#define SYSCFG_PA_IORP_PA3_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA3_IORP_Pos)         /*!< 0x00000040 */ 
#define SYSCFG_PA_IORP_PA3_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA3_IORP_Pos)         /*!< 0x00000080 */ 

#define SYSCFG_PA_IORP_PA4_IORP_Pos               (8U)
#define SYSCFG_PA_IORP_PA4_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA4_IORP_Pos)         /*!< 0x00000300 */
#define SYSCFG_PA_IORP_PA4_IORP                   SYSCFG_PA_IORP_PA4_IORP_Msk                    
#define SYSCFG_PA_IORP_PA4_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA4_IORP_Pos)         /*!< 0x00000100 */ 
#define SYSCFG_PA_IORP_PA4_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA4_IORP_Pos)         /*!< 0x00000200 */ 

#define SYSCFG_PA_IORP_PA5_IORP_Pos               (10U)
#define SYSCFG_PA_IORP_PA5_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA5_IORP_Pos)         /*!< 0x00000C00 */
#define SYSCFG_PA_IORP_PA5_IORP                   SYSCFG_PA_IORP_PA5_IORP_Msk                    
#define SYSCFG_PA_IORP_PA5_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA5_IORP_Pos)         /*!< 0x00000400 */ 
#define SYSCFG_PA_IORP_PA5_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA5_IORP_Pos)         /*!< 0x00000800 */ 

#define SYSCFG_PA_IORP_PA6_IORP_Pos               (12U)
#define SYSCFG_PA_IORP_PA6_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA6_IORP_Pos)         /*!< 0x00003000 */
#define SYSCFG_PA_IORP_PA6_IORP                   SYSCFG_PA_IORP_PA6_IORP_Msk                    
#define SYSCFG_PA_IORP_PA6_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA6_IORP_Pos)         /*!< 0x00001000 */ 
#define SYSCFG_PA_IORP_PA6_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA6_IORP_Pos)         /*!< 0x00002000 */ 

#define SYSCFG_PA_IORP_PA7_IORP_Pos               (14U)
#define SYSCFG_PA_IORP_PA7_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA7_IORP_Pos)         /*!< 0x0000C000 */
#define SYSCFG_PA_IORP_PA7_IORP                   SYSCFG_PA_IORP_PA7_IORP_Msk                    
#define SYSCFG_PA_IORP_PA7_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA7_IORP_Pos)         /*!< 0x00004000 */ 
#define SYSCFG_PA_IORP_PA7_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA7_IORP_Pos)         /*!< 0x00008000 */ 

#define SYSCFG_PA_IORP_PA8_IORP_Pos               (16U)
#define SYSCFG_PA_IORP_PA8_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA8_IORP_Pos)         /*!< 0x00030000 */
#define SYSCFG_PA_IORP_PA8_IORP                   SYSCFG_PA_IORP_PA8_IORP_Msk                    
#define SYSCFG_PA_IORP_PA8_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA8_IORP_Pos)         /*!< 0x00010000 */ 
#define SYSCFG_PA_IORP_PA8_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA8_IORP_Pos)         /*!< 0x00020000 */ 

#define SYSCFG_PA_IORP_PA9_IORP_Pos               (18U)
#define SYSCFG_PA_IORP_PA9_IORP_Msk               (0x3UL << SYSCFG_PA_IORP_PA9_IORP_Pos)         /*!< 0x000C0000 */
#define SYSCFG_PA_IORP_PA9_IORP                   SYSCFG_PA_IORP_PA9_IORP_Msk                    
#define SYSCFG_PA_IORP_PA9_IORP_0                 (0x1UL << SYSCFG_PA_IORP_PA9_IORP_Pos)         /*!< 0x00040000 */ 
#define SYSCFG_PA_IORP_PA9_IORP_1                 (0x2UL << SYSCFG_PA_IORP_PA9_IORP_Pos)         /*!< 0x00080000 */ 

#define SYSCFG_PA_IORP_PA10_IORP_Pos              (20U)
#define SYSCFG_PA_IORP_PA10_IORP_Msk              (0x3UL << SYSCFG_PA_IORP_PA10_IORP_Pos)        /*!< 0x00300000 */
#define SYSCFG_PA_IORP_PA10_IORP                  SYSCFG_PA_IORP_PA10_IORP_Msk                   
#define SYSCFG_PA_IORP_PA10_IORP_0                (0x1UL << SYSCFG_PA_IORP_PA10_IORP_Pos)        /*!< 0x00100000 */ 
#define SYSCFG_PA_IORP_PA10_IORP_1                (0x2UL << SYSCFG_PA_IORP_PA10_IORP_Pos)        /*!< 0x00200000 */ 

#define SYSCFG_PA_IORP_PA11_IORP_Pos              (22U)
#define SYSCFG_PA_IORP_PA11_IORP_Msk              (0x3UL << SYSCFG_PA_IORP_PA11_IORP_Pos)        /*!< 0x00C00000 */
#define SYSCFG_PA_IORP_PA11_IORP                  SYSCFG_PA_IORP_PA11_IORP_Msk                   
#define SYSCFG_PA_IORP_PA11_IORP_0                (0x1UL << SYSCFG_PA_IORP_PA11_IORP_Pos)        /*!< 0x00400000 */ 
#define SYSCFG_PA_IORP_PA11_IORP_1                (0x2UL << SYSCFG_PA_IORP_PA11_IORP_Pos)        /*!< 0x00800000 */ 

#define SYSCFG_PA_IORP_PA12_IORP_Pos              (24U)
#define SYSCFG_PA_IORP_PA12_IORP_Msk              (0x3UL << SYSCFG_PA_IORP_PA12_IORP_Pos)        /*!< 0x03000000 */
#define SYSCFG_PA_IORP_PA12_IORP                  SYSCFG_PA_IORP_PA12_IORP_Msk                   
#define SYSCFG_PA_IORP_PA12_IORP_0                (0x1UL << SYSCFG_PA_IORP_PA12_IORP_Pos)        /*!< 0x01000000 */ 
#define SYSCFG_PA_IORP_PA12_IORP_1                (0x2UL << SYSCFG_PA_IORP_PA12_IORP_Pos)        /*!< 0x02000000 */ 

#define SYSCFG_PA_IORP_PA13_IORP_Pos              (26U)
#define SYSCFG_PA_IORP_PA13_IORP_Msk              (0x3UL << SYSCFG_PA_IORP_PA13_IORP_Pos)        /*!< 0x0C000000 */
#define SYSCFG_PA_IORP_PA13_IORP                  SYSCFG_PA_IORP_PA13_IORP_Msk                   
#define SYSCFG_PA_IORP_PA13_IORP_0                (0x1UL << SYSCFG_PA_IORP_PA13_IORP_Pos)        /*!< 0x04000000 */ 
#define SYSCFG_PA_IORP_PA13_IORP_1                (0x2UL << SYSCFG_PA_IORP_PA13_IORP_Pos)        /*!< 0x08000000 */ 

#define SYSCFG_PA_IORP_PA14_IORP_Pos              (28U)
#define SYSCFG_PA_IORP_PA14_IORP_Msk              (0x3UL << SYSCFG_PA_IORP_PA14_IORP_Pos)        /*!< 0x30000000 */
#define SYSCFG_PA_IORP_PA14_IORP                  SYSCFG_PA_IORP_PA14_IORP_Msk                   
#define SYSCFG_PA_IORP_PA14_IORP_0                (0x1UL << SYSCFG_PA_IORP_PA14_IORP_Pos)        /*!< 0x10000000 */ 
#define SYSCFG_PA_IORP_PA14_IORP_1                (0x2UL << SYSCFG_PA_IORP_PA14_IORP_Pos)        /*!< 0x20000000 */ 

#define SYSCFG_PA_IORP_PA15_IORP_Pos              (30U)
#define SYSCFG_PA_IORP_PA15_IORP_Msk              (0x3UL << SYSCFG_PA_IORP_PA15_IORP_Pos)        /*!< 0xC0000000 */
#define SYSCFG_PA_IORP_PA15_IORP                  SYSCFG_PA_IORP_PA15_IORP_Msk                   
#define SYSCFG_PA_IORP_PA15_IORP_0                (0x1UL << SYSCFG_PA_IORP_PA15_IORP_Pos)        /*!< 0x40000000 */ 
#define SYSCFG_PA_IORP_PA15_IORP_1                (0x2UL << SYSCFG_PA_IORP_PA15_IORP_Pos)        /*!< 0x80000000 */ 

/*****************  Bit definition for PB_IORP register  ************************/
#define SYSCFG_PB_IORP_PB0_IORP_Pos               (0U)
#define SYSCFG_PB_IORP_PB0_IORP_Msk               (0x3UL << SYSCFG_PB_IORP_PB0_IORP_Pos)         /*!< 0x00000003 */
#define SYSCFG_PB_IORP_PB0_IORP                   SYSCFG_PB_IORP_PB0_IORP_Msk                    
#define SYSCFG_PB_IORP_PB0_IORP_0                 (0x1UL << SYSCFG_PB_IORP_PB0_IORP_Pos)         /*!< 0x00000001 */ 
#define SYSCFG_PB_IORP_PB0_IORP_1                 (0x2UL << SYSCFG_PB_IORP_PB0_IORP_Pos)         /*!< 0x00000002 */ 

#define SYSCFG_PB_IORP_PB1_IORP_Pos               (2U)
#define SYSCFG_PB_IORP_PB1_IORP_Msk               (0x3UL << SYSCFG_PB_IORP_PB1_IORP_Pos)         /*!< 0x0000000C */
#define SYSCFG_PB_IORP_PB1_IORP                   SYSCFG_PB_IORP_PB1_IORP_Msk                    
#define SYSCFG_PB_IORP_PB1_IORP_0                 (0x1UL << SYSCFG_PB_IORP_PB1_IORP_Pos)         /*!< 0x00000004 */ 
#define SYSCFG_PB_IORP_PB1_IORP_1                 (0x2UL << SYSCFG_PB_IORP_PB1_IORP_Pos)         /*!< 0x00000008 */ 

#define SYSCFG_PB_IORP_PB2_IORP_Pos               (4U)
#define SYSCFG_PB_IORP_PB2_IORP_Msk               (0x3UL << SYSCFG_PB_IORP_PB2_IORP_Pos)         /*!< 0x00000030 */
#define SYSCFG_PB_IORP_PB2_IORP                   SYSCFG_PB_IORP_PB2_IORP_Msk                    
#define SYSCFG_PB_IORP_PB2_IORP_0                 (0x1UL << SYSCFG_PB_IORP_PB2_IORP_Pos)         /*!< 0x00000010 */ 
#define SYSCFG_PB_IORP_PB2_IORP_1                 (0x2UL << SYSCFG_PB_IORP_PB2_IORP_Pos)         /*!< 0x00000020 */ 

#define SYSCFG_PB_IORP_PB3_IORP_Pos               (6U)
#define SYSCFG_PB_IORP_PB3_IORP_Msk               (0x3UL << SYSCFG_PB_IORP_PB3_IORP_Pos)         /*!< 0x000000C0 */
#define SYSCFG_PB_IORP_PB3_IORP                   SYSCFG_PB_IORP_PB3_IORP_Msk                    
#define SYSCFG_PB_IORP_PB3_IORP_0                 (0x1UL << SYSCFG_PB_IORP_PB3_IORP_Pos)         /*!< 0x00000040 */ 
#define SYSCFG_PB_IORP_PB3_IORP_1                 (0x2UL << SYSCFG_PB_IORP_PB3_IORP_Pos)         /*!< 0x00000080 */ 

/*****************  Bit definition for PF_IORP register  ************************/
#define SYSCFG_PF_IORP_PF0_IORP_Pos               (0U)
#define SYSCFG_PF_IORP_PF0_IORP_Msk               (0x3UL << SYSCFG_PF_IORP_PF0_IORP_Pos)         /*!< 0x00000003 */
#define SYSCFG_PF_IORP_PF0_IORP                   SYSCFG_PF_IORP_PF0_IORP_Msk                    
#define SYSCFG_PF_IORP_PF0_IORP_0                 (0x1UL << SYSCFG_PF_IORP_PF0_IORP_Pos)         /*!< 0x00000001 */ 
#define SYSCFG_PF_IORP_PF0_IORP_1                 (0x2UL << SYSCFG_PF_IORP_PF0_IORP_Pos)         /*!< 0x00000002 */ 

#define SYSCFG_PF_IORP_PF1_IORP_Pos               (2U)
#define SYSCFG_PF_IORP_PF1_IORP_Msk               (0x3UL << SYSCFG_PF_IORP_PF1_IORP_Pos)         /*!< 0x0000000C */
#define SYSCFG_PF_IORP_PF1_IORP                   SYSCFG_PF_IORP_PF1_IORP_Msk                    
#define SYSCFG_PF_IORP_PF1_IORP_0                 (0x1UL << SYSCFG_PF_IORP_PF1_IORP_Pos)         /*!< 0x00000004 */ 
#define SYSCFG_PF_IORP_PF1_IORP_1                 (0x2UL << SYSCFG_PF_IORP_PF1_IORP_Pos)         /*!< 0x00000008 */ 

#define SYSCFG_PF_IORP_PF2_IORP_Pos               (4U)
#define SYSCFG_PF_IORP_PF2_IORP_Msk               (0x3UL << SYSCFG_PF_IORP_PF2_IORP_Pos)         /*!< 0x00000030 */
#define SYSCFG_PF_IORP_PF2_IORP                   SYSCFG_PF_IORP_PF2_IORP_Msk                    
#define SYSCFG_PF_IORP_PF2_IORP_0                 (0x1UL << SYSCFG_PF_IORP_PF2_IORP_Pos)         /*!< 0x00000010 */ 
#define SYSCFG_PF_IORP_PF2_IORP_1                 (0x2UL << SYSCFG_PF_IORP_PF2_IORP_Pos)         /*!< 0x00000020 */ 

#define SYSCFG_PF_IORP_PF3_IORP_Pos               (6U)
#define SYSCFG_PF_IORP_PF3_IORP_Msk               (0x3UL << SYSCFG_PF_IORP_PF3_IORP_Pos)         /*!< 0x000000C0 */
#define SYSCFG_PF_IORP_PF3_IORP                   SYSCFG_PF_IORP_PF3_IORP_Msk                    
#define SYSCFG_PF_IORP_PF3_IORP_0                 (0x1UL << SYSCFG_PF_IORP_PF3_IORP_Pos)         /*!< 0x00000040 */ 
#define SYSCFG_PF_IORP_PF3_IORP_1                 (0x2UL << SYSCFG_PF_IORP_PF3_IORP_Pos)         /*!< 0x00000080 */ 

#define SYSCFG_PF_IORP_PF4_IORP_Pos               (8U)
#define SYSCFG_PF_IORP_PF4_IORP_Msk               (0x3UL << SYSCFG_PF_IORP_PF4_IORP_Pos)         /*!< 0x00000300 */
#define SYSCFG_PF_IORP_PF4_IORP                   SYSCFG_PF_IORP_PF4_IORP_Msk                    
#define SYSCFG_PF_IORP_PF4_IORP_0                 (0x1UL << SYSCFG_PF_IORP_PF4_IORP_Pos)         /*!< 0x00000100 */ 
#define SYSCFG_PF_IORP_PF4_IORP_1                 (0x2UL << SYSCFG_PF_IORP_PF4_IORP_Pos)         /*!< 0x00000200 */ 

#define SYSCFG_PF_IORP_PF5_IORP_Pos               (10U)
#define SYSCFG_PF_IORP_PF5_IORP_Msk               (0x3UL << SYSCFG_PF_IORP_PF5_IORP_Pos)         /*!< 0x00000C00 */
#define SYSCFG_PF_IORP_PF5_IORP                   SYSCFG_PF_IORP_PF5_IORP_Msk                    
#define SYSCFG_PF_IORP_PF5_IORP_0                 (0x1UL << SYSCFG_PF_IORP_PF5_IORP_Pos)         /*!< 0x00000400 */ 
#define SYSCFG_PF_IORP_PF5_IORP_1                 (0x2UL << SYSCFG_PF_IORP_PF5_IORP_Pos)         /*!< 0x00000800 */ 

/*****************************************************************************/
/*                                                                           */
/*                               Timers (TIM)                                */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for TIM_CR1 register  *******************/
#define TIM_CR1_CEN_Pos           (0U)
#define TIM_CR1_CEN_Msk           (0x1UL << TIM_CR1_CEN_Pos)                    /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                               /*!<Counter enable */

#define TIM_CR1_UDIS_Pos          (1U)
#define TIM_CR1_UDIS_Msk          (0x1UL << TIM_CR1_UDIS_Pos)                   /*!< 0x00000002 */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk                              /*!<Update disable */

#define TIM_CR1_URS_Pos           (2U)
#define TIM_CR1_URS_Msk           (0x1UL << TIM_CR1_URS_Pos)                    /*!< 0x00000004 */
#define TIM_CR1_URS               TIM_CR1_URS_Msk                               /*!<Update request source */

#define TIM_CR1_OPM_Pos           (3U)
#define TIM_CR1_OPM_Msk           (0x1UL << TIM_CR1_OPM_Pos)                    /*!< 0x00000008 */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk                               /*!<One pulse mode */

#define TIM_CR1_DIR_Pos           (4U)
#define TIM_CR1_DIR_Msk           (0x1UL << TIM_CR1_DIR_Pos)                    /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                               /*!<Direction */

#define TIM_CR1_CMS_Pos           (5U)
#define TIM_CR1_CMS_Msk           (0x3UL << TIM_CR1_CMS_Pos)                    /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                               /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x1UL << TIM_CR1_CMS_Pos)                    /*!< 0x00000020 */
#define TIM_CR1_CMS_1             (0x2UL << TIM_CR1_CMS_Pos)                    /*!< 0x00000040 */

#define TIM_CR1_ARPE_Pos          (7U)
#define TIM_CR1_ARPE_Msk          (0x1UL << TIM_CR1_ARPE_Pos)                   /*!< 0x00000080 */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk                              /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos           (8U)
#define TIM_CR1_CKD_Msk           (0x3UL << TIM_CR1_CKD_Pos)                    /*!< 0x00000300 */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk                               /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0             (0x1UL << TIM_CR1_CKD_Pos)                    /*!< 0x00000100 */
#define TIM_CR1_CKD_1             (0x2UL << TIM_CR1_CKD_Pos)                    /*!< 0x00000200 */

/*******************  Bit definition for TIM_CR2 register  *******************/
#define TIM_CR2_CCPC_Pos          (0U)
#define TIM_CR2_CCPC_Msk          (0x1UL << TIM_CR2_CCPC_Pos)                   /*!< 0x00000001 */
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk                              /*!<Capture/Compare Preloaded Control */

#define TIM_CR2_CCUS_Pos          (2U)
#define TIM_CR2_CCUS_Msk          (0x1UL << TIM_CR2_CCUS_Pos)                   /*!< 0x00000004 */
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk                              /*!<Capture/Compare Control Update Selection */

#define TIM_CR2_MMS_Pos           (4U)
#define TIM_CR2_MMS_Msk           (0x7UL << TIM_CR2_MMS_Pos)                    /*!< 0x00000070 */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk                               /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0             (0x1UL << TIM_CR2_MMS_Pos)                    /*!< 0x00000010 */
#define TIM_CR2_MMS_1             (0x2UL << TIM_CR2_MMS_Pos)                    /*!< 0x00000020 */
#define TIM_CR2_MMS_2             (0x4UL << TIM_CR2_MMS_Pos)                    /*!< 0x00000040 */

#define TIM_CR2_TI1S_Pos          (7U)
#define TIM_CR2_TI1S_Msk          (0x1UL << TIM_CR2_TI1S_Pos)                   /*!< 0x00000080 */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk                              /*!<TI1 Selection */

#define TIM_CR2_OIS1_Pos          (8U)
#define TIM_CR2_OIS1_Msk          (0x1UL << TIM_CR2_OIS1_Pos)                   /*!< 0x00000100 */
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk                              /*!<Output Idle state 1 (OC1 output) */

#define TIM_CR2_OIS1N_Pos         (9U)
#define TIM_CR2_OIS1N_Msk         (0x1UL << TIM_CR2_OIS1N_Pos)                  /*!< 0x00000200 */
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk                             /*!<Output Idle state 1 (OC1N output) */

#define TIM_CR2_OIS2_Pos          (10U)
#define TIM_CR2_OIS2_Msk          (0x1UL << TIM_CR2_OIS2_Pos)                   /*!< 0x00000400 */
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk                              /*!<Output Idle state 2 (OC2 output) */

#define TIM_CR2_OIS2N_Pos         (11U)
#define TIM_CR2_OIS2N_Msk         (0x1UL << TIM_CR2_OIS2N_Pos)                  /*!< 0x00000800 */
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk                             /*!<Output Idle state 2 (OC2N output) */

#define TIM_CR2_OIS3_Pos          (12U)
#define TIM_CR2_OIS3_Msk          (0x1UL << TIM_CR2_OIS3_Pos)                   /*!< 0x00001000 */
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk                              /*!<Output Idle state 3 (OC3 output) */

#define TIM_CR2_OIS3N_Pos         (13U)
#define TIM_CR2_OIS3N_Msk         (0x1UL << TIM_CR2_OIS3N_Pos)                  /*!< 0x00002000 */
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk                             /*!<Output Idle state 3 (OC3N output) */

#define TIM_CR2_OIS4_Pos          (14U)
#define TIM_CR2_OIS4_Msk          (0x1UL << TIM_CR2_OIS4_Pos)                   /*!< 0x00004000 */
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk                              /*!<Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  ******************/
#define TIM_SMCR_SMS_Pos          (0U)
#define TIM_SMCR_SMS_Pos1         (16U)
#define TIM_SMCR_SMS_Msk          ((0x7UL << TIM_SMCR_SMS_Pos) | (0x1UL<<TIM_SMCR_SMS_Pos1))  /*!< 0x00010007 */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk                                            /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0            (0x1UL << TIM_SMCR_SMS_Pos)                                 /*!< 0x00000001 */
#define TIM_SMCR_SMS_1            (0x2UL << TIM_SMCR_SMS_Pos)                                 /*!< 0x00000002 */
#define TIM_SMCR_SMS_2            (0x4UL << TIM_SMCR_SMS_Pos)                                 /*!< 0x00000004 */
#define TIM_SMCR_SMS_3            (0x1UL << TIM_SMCR_SMS_Pos1)                                /*!< 0x00000004 */

#define TIM_SMCR_OCCS_Pos         (3U)
#define TIM_SMCR_OCCS_Msk         (0x1UL << TIM_SMCR_OCCS_Pos)                  /*!< 0x00000008 */
#define TIM_SMCR_OCCS             TIM_SMCR_OCCS_Msk                             /*!< OCREF clear selection */

#define TIM_SMCR_TS_Pos           (4U)
#define TIM_SMCR_TS_Msk           (0x7UL << TIM_SMCR_TS_Pos)                    /*!< 0x00000070 */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk                               /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0             (0x1UL << TIM_SMCR_TS_Pos)                    /*!< 0x00000010 */
#define TIM_SMCR_TS_1             (0x2UL << TIM_SMCR_TS_Pos)                    /*!< 0x00000020 */
#define TIM_SMCR_TS_2             (0x4UL << TIM_SMCR_TS_Pos)                    /*!< 0x00000040 */

#define TIM_SMCR_MSM_Pos          (7U)
#define TIM_SMCR_MSM_Msk          (0x1UL << TIM_SMCR_MSM_Pos)                   /*!< 0x00000080 */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk                              /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos          (8U)
#define TIM_SMCR_ETF_Msk          (0xFUL << TIM_SMCR_ETF_Pos)                   /*!< 0x00000F00 */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk                              /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0            (0x1UL << TIM_SMCR_ETF_Pos)                   /*!< 0x00000100 */
#define TIM_SMCR_ETF_1            (0x2UL << TIM_SMCR_ETF_Pos)                   /*!< 0x00000200 */
#define TIM_SMCR_ETF_2            (0x4UL << TIM_SMCR_ETF_Pos)                   /*!< 0x00000400 */
#define TIM_SMCR_ETF_3            (0x8UL << TIM_SMCR_ETF_Pos)                   /*!< 0x00000800 */

#define TIM_SMCR_ETPS_Pos         (12U)
#define TIM_SMCR_ETPS_Msk         (0x3UL << TIM_SMCR_ETPS_Pos)                  /*!< 0x00003000 */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk                             /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0           (0x1UL << TIM_SMCR_ETPS_Pos)                  /*!< 0x00001000 */
#define TIM_SMCR_ETPS_1           (0x2UL << TIM_SMCR_ETPS_Pos)                  /*!< 0x00002000 */

#define TIM_SMCR_ECE_Pos          (14U)
#define TIM_SMCR_ECE_Msk          (0x1UL << TIM_SMCR_ECE_Pos)                   /*!< 0x00004000 */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk                              /*!<External clock enable */

#define TIM_SMCR_ETP_Pos          (15U)
#define TIM_SMCR_ETP_Msk          (0x1UL << TIM_SMCR_ETP_Pos)                   /*!< 0x00008000 */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk                              /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  ******************/
#define TIM_DIER_UIE_Pos          (0U)
#define TIM_DIER_UIE_Msk          (0x1UL << TIM_DIER_UIE_Pos)                   /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk                              /*!<Update interrupt enable */

#define TIM_DIER_CC1IE_Pos        (1U)
#define TIM_DIER_CC1IE_Msk        (0x1UL << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                            /*!<Capture/Compare 1 interrupt enable */

#define TIM_DIER_CC2IE_Pos        (2U)
#define TIM_DIER_CC2IE_Msk        (0x1UL << TIM_DIER_CC2IE_Pos)                 /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk                            /*!<Capture/Compare 2 interrupt enable */

#define TIM_DIER_CC3IE_Pos        (3U)
#define TIM_DIER_CC3IE_Msk        (0x1UL << TIM_DIER_CC3IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk                            /*!<Capture/Compare 3 interrupt enable */

#define TIM_DIER_CC4IE_Pos        (4U)
#define TIM_DIER_CC4IE_Msk        (0x1UL << TIM_DIER_CC4IE_Pos)                 /*!< 0x00000010 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk                            /*!<Capture/Compare 4 interrupt enable */

#define TIM_DIER_COMIE_Pos        (5U)
#define TIM_DIER_COMIE_Msk        (0x1UL << TIM_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                            /*!<COM interrupt enable */

#define TIM_DIER_TIE_Pos          (6U)
#define TIM_DIER_TIE_Msk          (0x1UL << TIM_DIER_TIE_Pos)                   /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                              /*!<Trigger interrupt enable */

#define TIM_DIER_BIE_Pos          (7U)
#define TIM_DIER_BIE_Msk          (0x1UL << TIM_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk                              /*!<Break interrupt enable */

/********************  Bit definition for TIM_SR register  *******************/
#define TIM_SR_UIF_Pos            (0U)
#define TIM_SR_UIF_Msk            (0x1UL << TIM_SR_UIF_Pos)                     /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                                /*!<Update interrupt Flag */

#define TIM_SR_CC1IF_Pos          (1U)
#define TIM_SR_CC1IF_Msk          (0x1UL << TIM_SR_CC1IF_Pos)                   /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk                              /*!<Capture/Compare 1 interrupt Flag */

#define TIM_SR_CC2IF_Pos          (2U)
#define TIM_SR_CC2IF_Msk          (0x1UL << TIM_SR_CC2IF_Pos)                   /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk                              /*!<Capture/Compare 2 interrupt Flag */

#define TIM_SR_CC3IF_Pos          (3U)
#define TIM_SR_CC3IF_Msk          (0x1UL << TIM_SR_CC3IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk                              /*!<Capture/Compare 3 interrupt Flag */

#define TIM_SR_CC4IF_Pos          (4U)
#define TIM_SR_CC4IF_Msk          (0x1UL << TIM_SR_CC4IF_Pos)                   /*!< 0x00000010 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk                              /*!<Capture/Compare 4 interrupt Flag */

#define TIM_SR_COMIF_Pos          (5U)
#define TIM_SR_COMIF_Msk          (0x1UL << TIM_SR_COMIF_Pos)                   /*!< 0x00000020 */
#define TIM_SR_COMIF              TIM_SR_COMIF_Msk                              /*!<COM interrupt Flag */

#define TIM_SR_TIF_Pos            (6U)
#define TIM_SR_TIF_Msk            (0x1UL << TIM_SR_TIF_Pos)                     /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk                                /*!<Trigger interrupt Flag */

#define TIM_SR_BIF_Pos            (7U)
#define TIM_SR_BIF_Msk            (0x1UL << TIM_SR_BIF_Pos)                     /*!< 0x00000080 */
#define TIM_SR_BIF                TIM_SR_BIF_Msk                                /*!<Break interrupt Flag */

#define TIM_SR_CC1OF_Pos          (9U)
#define TIM_SR_CC1OF_Msk          (0x1UL << TIM_SR_CC1OF_Pos)                   /*!< 0x00000200 */
#define TIM_SR_CC1OF              TIM_SR_CC1OF_Msk                              /*!<Capture/Compare 1 Overcapture Flag */

#define TIM_SR_CC2OF_Pos          (10U)
#define TIM_SR_CC2OF_Msk          (0x1UL << TIM_SR_CC2OF_Pos)                   /*!< 0x00000400 */
#define TIM_SR_CC2OF              TIM_SR_CC2OF_Msk                              /*!<Capture/Compare 2 Overcapture Flag */

#define TIM_SR_CC3OF_Pos          (11U)
#define TIM_SR_CC3OF_Msk          (0x1UL << TIM_SR_CC3OF_Pos)                   /*!< 0x00000800 */
#define TIM_SR_CC3OF              TIM_SR_CC3OF_Msk                              /*!<Capture/Compare 3 Overcapture Flag */

#define TIM_SR_CC4OF_Pos          (12U)
#define TIM_SR_CC4OF_Msk          (0x1UL << TIM_SR_CC4OF_Pos)                   /*!< 0x00001000 */
#define TIM_SR_CC4OF              TIM_SR_CC4OF_Msk                              /*!<Capture/Compare 4 Overcapture Flag */

#define TIM_SR_IC1IR_Pos          (16U)
#define TIM_SR_IC1IR_Msk          (0x1UL << TIM_SR_IC1IR_Pos)                   /*!< 0x00010000 */
#define TIM_SR_IC1IR              TIM_SR_IC1IR_Msk                              /*!< desc IC1IR */

#define TIM_SR_IC2IR_Pos          (17U)
#define TIM_SR_IC2IR_Msk          (0x1UL << TIM_SR_IC2IR_Pos)                   /*!< 0x00020000 */
#define TIM_SR_IC2IR              TIM_SR_IC2IR_Msk                              /*!< desc IC2IR */

#define TIM_SR_IC3IR_Pos          (18U)
#define TIM_SR_IC3IR_Msk          (0x1UL << TIM_SR_IC3IR_Pos)                   /*!< 0x00040000 */
#define TIM_SR_IC3IR              TIM_SR_IC3IR_Msk                              /*!< desc IC3IR */

#define TIM_SR_IC4IR_Pos          (19U)
#define TIM_SR_IC4IR_Msk          (0x1UL << TIM_SR_IC4IR_Pos)                   /*!< 0x00080000 */
#define TIM_SR_IC4IR              TIM_SR_IC4IR_Msk                              /*!< desc IC4IR */

#define TIM_SR_IC1IF_Pos          (20U)
#define TIM_SR_IC1IF_Msk          (0x1UL << TIM_SR_IC1IF_Pos)                   /*!< 0x00100000 */
#define TIM_SR_IC1IF              TIM_SR_IC1IF_Msk                              /*!< desc IC1IF */

#define TIM_SR_IC2IF_Pos          (21U)
#define TIM_SR_IC2IF_Msk          (0x1UL << TIM_SR_IC2IF_Pos)                   /*!< 0x00200000 */
#define TIM_SR_IC2IF              TIM_SR_IC2IF_Msk                              /*!< desc IC2IF */

#define TIM_SR_IC3IF_Pos          (22U)
#define TIM_SR_IC3IF_Msk          (0x1UL << TIM_SR_IC3IF_Pos)                   /*!< 0x00400000 */
#define TIM_SR_IC3IF              TIM_SR_IC3IF_Msk                              /*!< desc IC3IF */

#define TIM_SR_IC4IF_Pos          (23U)
#define TIM_SR_IC4IF_Msk          (0x1UL << TIM_SR_IC4IF_Pos)                   /*!< 0x00800000 */
#define TIM_SR_IC4IF              TIM_SR_IC4IF_Msk                              /*!< desc IC4IF */

/*******************  Bit definition for TIM_EGR register  *******************/
#define TIM_EGR_UG_Pos            (0U)
#define TIM_EGR_UG_Msk            (0x1UL << TIM_EGR_UG_Pos)                     /*!< 0x00000001 */
#define TIM_EGR_UG                TIM_EGR_UG_Msk                                /*!<Update Generation */

#define TIM_EGR_CC1G_Pos          (1U)
#define TIM_EGR_CC1G_Msk          (0x1UL << TIM_EGR_CC1G_Pos)                   /*!< 0x00000002 */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk                              /*!<Capture/Compare 1 Generation */

#define TIM_EGR_CC2G_Pos          (2U)
#define TIM_EGR_CC2G_Msk          (0x1UL << TIM_EGR_CC2G_Pos)                   /*!< 0x00000004 */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk                              /*!<Capture/Compare 2 Generation */

#define TIM_EGR_CC3G_Pos          (3U)
#define TIM_EGR_CC3G_Msk          (0x1UL << TIM_EGR_CC3G_Pos)                   /*!< 0x00000008 */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk                              /*!<Capture/Compare 3 Generation */

#define TIM_EGR_CC4G_Pos          (4U)
#define TIM_EGR_CC4G_Msk          (0x1UL << TIM_EGR_CC4G_Pos)                   /*!< 0x00000010 */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk                              /*!<Capture/Compare 4 Generation */

#define TIM_EGR_COMG_Pos          (5U)
#define TIM_EGR_COMG_Msk          (0x1UL << TIM_EGR_COMG_Pos)                   /*!< 0x00000020 */
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk                              /*!<Capture/Compare Control Update Generation */

#define TIM_EGR_TG_Pos            (6U)
#define TIM_EGR_TG_Msk            (0x1UL << TIM_EGR_TG_Pos)                     /*!< 0x00000040 */
#define TIM_EGR_TG                TIM_EGR_TG_Msk                                /*!<Trigger Generation */

#define TIM_EGR_BG_Pos            (7U)
#define TIM_EGR_BG_Msk            (0x1UL << TIM_EGR_BG_Pos)                     /*!< 0x00000080 */
#define TIM_EGR_BG                TIM_EGR_BG_Msk                                /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  ******************/
#define TIM_CCMR1_CC1S_Pos        (0U)
#define TIM_CCMR1_CC1S_Msk        (0x3UL << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk                            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0          (0x1UL << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_1          (0x2UL << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000002 */

#define TIM_CCMR1_OC1FE_Pos       (2U)
#define TIM_CCMR1_OC1FE_Msk       (0x1UL << TIM_CCMR1_OC1FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk                           /*!<Output Compare 1 Fast enable */

#define TIM_CCMR1_OC1PE_Pos       (3U)
#define TIM_CCMR1_OC1PE_Msk       (0x1UL << TIM_CCMR1_OC1PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk                           /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos        (4U)
#define TIM_CCMR1_OC1M_Pos1       (16U)
#define TIM_CCMR1_OC1M_Msk        ((0x7UL << TIM_CCMR1_OC1M_Pos) | (0x1UL<<TIM_CCMR1_OC1M_Pos1))                 /*!< 0x00000070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                            /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0          (0x1UL << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR1_OC1M_2          (0x4UL << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR1_OC1M_3          (0x1UL << TIM_CCMR1_OC1M_Pos1)                /*!< 0x00000040 */

#define TIM_CCMR1_OC1CE_Pos       (7U)
#define TIM_CCMR1_OC1CE_Msk       (0x1UL << TIM_CCMR1_OC1CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE           TIM_CCMR1_OC1CE_Msk                           /*!<Output Compare 1Clear Enable */

#define TIM_CCMR1_CC2S_Pos        (8U)
#define TIM_CCMR1_CC2S_Msk        (0x3UL << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk                            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0          (0x1UL << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_1          (0x2UL << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000200 */

#define TIM_CCMR1_OC2FE_Pos       (10U)
#define TIM_CCMR1_OC2FE_Msk       (0x1UL << TIM_CCMR1_OC2FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk                           /*!<Output Compare 2 Fast enable */

#define TIM_CCMR1_OC2PE_Pos       (11U)
#define TIM_CCMR1_OC2PE_Msk       (0x1UL << TIM_CCMR1_OC2PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk                           /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos        (12U)
#define TIM_CCMR1_OC2M_Pos1       (24U)
#define TIM_CCMR1_OC2M_Msk        ((0x7UL << TIM_CCMR1_OC2M_Pos) | (0x1UL<<TIM_CCMR1_OC2M_Pos1))                 /*!< 0x00007000 */
#define TIM_CCMR1_OC2M            TIM_CCMR1_OC2M_Msk                            /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0          (0x1UL << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_1          (0x2UL << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_2          (0x4UL << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR1_OC2M_3          (0x1UL << TIM_CCMR1_OC2M_Pos1)                                 /*!< 0x00004000 */

#define TIM_CCMR1_OC2CE_Pos       (15U)
#define TIM_CCMR1_OC2CE_Msk       (0x1UL << TIM_CCMR1_OC2CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE           TIM_CCMR1_OC2CE_Msk                           /*!<Output Compare 2 Clear Enable */

/*---------------------------------------------------------------------------*/
#define TIM_CCMR1_IC1PSC_Pos      (2U)
#define TIM_CCMR1_IC1PSC_Msk      (0x3UL << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC          TIM_CCMR1_IC1PSC_Msk                          /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0        (0x1UL << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1        (0x2UL << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000008 */

#define TIM_CCMR1_IC1F_Pos        (4U)
#define TIM_CCMR1_IC1F_Msk        (0xFUL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F            TIM_CCMR1_IC1F_Msk                            /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0          (0x1UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_1          (0x2UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_2          (0x4UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_3          (0x8UL << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000080 */

#define TIM_CCMR1_IC2PSC_Pos      (10U)
#define TIM_CCMR1_IC2PSC_Msk      (0x3UL << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC          TIM_CCMR1_IC2PSC_Msk                          /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0        (0x1UL << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_1        (0x2UL << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000800 */

#define TIM_CCMR1_IC2F_Pos        (12U)
#define TIM_CCMR1_IC2F_Msk        (0xFUL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F            TIM_CCMR1_IC2F_Msk                            /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0          (0x1UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_1          (0x2UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_2          (0x4UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR1_IC2F_3          (0x8UL << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00008000 */

/******************  Bit definition for TIM_CCMR2 register  ******************/
#define TIM_CCMR2_CC3S_Pos        (0U)
#define TIM_CCMR2_CC3S_Msk        (0x3UL << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk                            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0          (0x1UL << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_1          (0x2UL << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000002 */

#define TIM_CCMR2_OC3FE_Pos       (2U)
#define TIM_CCMR2_OC3FE_Msk       (0x1UL << TIM_CCMR2_OC3FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk                           /*!<Output Compare 3 Fast enable */

#define TIM_CCMR2_OC3PE_Pos       (3U)
#define TIM_CCMR2_OC3PE_Msk       (0x1UL << TIM_CCMR2_OC3PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk                           /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos        (4U)
#define TIM_CCMR2_OC3M_Pos1       (16U)
#define TIM_CCMR2_OC3M_Msk        ((0x7UL << TIM_CCMR2_OC3M_Pos) | (0x1UL<<TIM_CCMR2_OC3M_Pos1))                 /*!< 0x00000070 */
#define TIM_CCMR2_OC3M            TIM_CCMR2_OC3M_Msk                            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0          (0x1UL << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR2_OC3M_1          (0x2UL << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR2_OC3M_2          (0x4UL << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR2_OC3M_3          (0x1UL << TIM_CCMR2_OC3M_Pos1)                 /*!< 0x00000040 */

#define TIM_CCMR2_OC3CE_Pos       (7U)
#define TIM_CCMR2_OC3CE_Msk       (0x1UL << TIM_CCMR2_OC3CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE           TIM_CCMR2_OC3CE_Msk                           /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos        (8U)
#define TIM_CCMR2_CC4S_Msk        (0x3UL << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk                            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0          (0x1UL << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_1          (0x2UL << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000200 */

#define TIM_CCMR2_OC4FE_Pos       (10U)
#define TIM_CCMR2_OC4FE_Msk       (0x1UL << TIM_CCMR2_OC4FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk                           /*!<Output Compare 4 Fast enable */

#define TIM_CCMR2_OC4PE_Pos       (11U)
#define TIM_CCMR2_OC4PE_Msk       (0x1UL << TIM_CCMR2_OC4PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk                           /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos        (12U)
#define TIM_CCMR2_OC4M_Pos1       (24U)
#define TIM_CCMR2_OC4M_Msk        ((0x7UL << TIM_CCMR2_OC4M_Pos) | (0x1UL<< TIM_CCMR2_OC4M_Pos1))                /*!< 0x00007000 */
#define TIM_CCMR2_OC4M            TIM_CCMR2_OC4M_Msk                            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0          (0x1UL << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR2_OC4M_1          (0x2UL << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR2_OC4M_2          (0x4UL << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR2_OC4M_3          (0x1UL << TIM_CCMR2_OC4M_Pos1)                /*!< 0x00004000 */

#define TIM_CCMR2_OC4CE_Pos       (15U)
#define TIM_CCMR2_OC4CE_Msk       (0x1UL << TIM_CCMR2_OC4CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE           TIM_CCMR2_OC4CE_Msk                           /*!<Output Compare 4 Clear Enable */

/*---------------------------------------------------------------------------*/
#define TIM_CCMR2_IC3PSC_Pos      (2U)
#define TIM_CCMR2_IC3PSC_Msk      (0x3UL << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC          TIM_CCMR2_IC3PSC_Msk                          /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0        (0x1UL << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_1        (0x2UL << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000008 */

#define TIM_CCMR2_IC3F_Pos        (4U)
#define TIM_CCMR2_IC3F_Msk        (0xFUL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F            TIM_CCMR2_IC3F_Msk                            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0          (0x1UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_1          (0x2UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_2          (0x4UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_3          (0x8UL << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000080 */

#define TIM_CCMR2_IC4PSC_Pos      (10U)
#define TIM_CCMR2_IC4PSC_Msk      (0x3UL << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC          TIM_CCMR2_IC4PSC_Msk                          /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0        (0x1UL << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_1        (0x2UL << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos        (12U)
#define TIM_CCMR2_IC4F_Msk        (0xFUL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F            TIM_CCMR2_IC4F_Msk                            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0          (0x1UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_1          (0x2UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_2          (0x4UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR2_IC4F_3          (0x8UL << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00008000 */

/*******************  Bit definition for TIM_CCER register  ******************/
#define TIM_CCER_CC1E_Pos         (0U)
#define TIM_CCER_CC1E_Msk         (0x1UL << TIM_CCER_CC1E_Pos)                  /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                             /*!<Capture/Compare 1 output enable */

#define TIM_CCER_CC1P_Pos         (1U)
#define TIM_CCER_CC1P_Msk         (0x1UL << TIM_CCER_CC1P_Pos)                  /*!< 0x00000002 */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk                             /*!<Capture/Compare 1 output Polarity */

#define TIM_CCER_CC1NE_Pos        (2U)
#define TIM_CCER_CC1NE_Msk        (0x1UL << TIM_CCER_CC1NE_Pos)                 /*!< 0x00000004 */
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk                            /*!<Capture/Compare 1 Complementary output enable */

#define TIM_CCER_CC1NP_Pos        (3U)
#define TIM_CCER_CC1NP_Msk        (0x1UL << TIM_CCER_CC1NP_Pos)                 /*!< 0x00000008 */
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk                            /*!<Capture/Compare 1 Complementary output Polarity */

#define TIM_CCER_CC2E_Pos         (4U)
#define TIM_CCER_CC2E_Msk         (0x1UL << TIM_CCER_CC2E_Pos)                  /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                             /*!<Capture/Compare 2 output enable */

#define TIM_CCER_CC2P_Pos         (5U)
#define TIM_CCER_CC2P_Msk         (0x1UL << TIM_CCER_CC2P_Pos)                  /*!< 0x00000020 */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk                             /*!<Capture/Compare 2 output Polarity */

#define TIM_CCER_CC2NE_Pos        (6U)
#define TIM_CCER_CC2NE_Msk        (0x1UL << TIM_CCER_CC2NE_Pos)                 /*!< 0x00000040 */
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk                            /*!<Capture/Compare 2 Complementary output enable */

#define TIM_CCER_CC2NP_Pos        (7U)
#define TIM_CCER_CC2NP_Msk        (0x1UL << TIM_CCER_CC2NP_Pos)                 /*!< 0x00000080 */
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk                            /*!<Capture/Compare 2 Complementary output Polarity */

#define TIM_CCER_CC3E_Pos         (8U)
#define TIM_CCER_CC3E_Msk         (0x1UL << TIM_CCER_CC3E_Pos)                  /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                             /*!<Capture/Compare 3 output enable */

#define TIM_CCER_CC3P_Pos         (9U)
#define TIM_CCER_CC3P_Msk         (0x1UL << TIM_CCER_CC3P_Pos)                  /*!< 0x00000200 */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk                             /*!<Capture/Compare 3 output Polarity */

#define TIM_CCER_CC3NE_Pos        (10U)
#define TIM_CCER_CC3NE_Msk        (0x1UL << TIM_CCER_CC3NE_Pos)                 /*!< 0x00000400 */
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk                            /*!<Capture/Compare 3 Complementary output enable */

#define TIM_CCER_CC3NP_Pos        (11U)
#define TIM_CCER_CC3NP_Msk        (0x1UL << TIM_CCER_CC3NP_Pos)                 /*!< 0x00000800 */
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk                            /*!<Capture/Compare 3 Complementary output Polarity */

#define TIM_CCER_CC4E_Pos         (12U)
#define TIM_CCER_CC4E_Msk         (0x1UL << TIM_CCER_CC4E_Pos)                  /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                             /*!<Capture/Compare 4 output enable */

#define TIM_CCER_CC4P_Pos         (13U)
#define TIM_CCER_CC4P_Msk         (0x1UL << TIM_CCER_CC4P_Pos)                  /*!< 0x00002000 */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk                             /*!<Capture/Compare 4 output Polarity */

/*******************  Bit definition for TIM_CNT register  *******************/
#define TIM_CNT_CNT_Pos           (0U)
#define TIM_CNT_CNT_Msk           (0xFFFFUL << TIM_CNT_CNT_Pos)                /*!< 0x0000FFFF */
#define TIM_CNT_CNT               TIM_CNT_CNT_Msk                              /*!<Counter Value */

/*******************  Bit definition for TIM_PSC register  *******************/
#define TIM_PSC_PSC_Pos           (0U)
#define TIM_PSC_PSC_Msk           (0xFFFFUL << TIM_PSC_PSC_Pos)                /*!< 0x0000FFFF */
#define TIM_PSC_PSC               TIM_PSC_PSC_Msk                              /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  *******************/
#define TIM_ARR_ARR_Pos           (0U)
#define TIM_ARR_ARR_Msk           (0xFFFFUL << TIM_ARR_ARR_Pos)                /*!< 0x0000FFFF */
#define TIM_ARR_ARR               TIM_ARR_ARR_Msk                              /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  *******************/
#define TIM_RCR_REP_Pos           (0U)
#define TIM_RCR_REP_Msk           (0xFFUL << TIM_RCR_REP_Pos)                  /*!< 0x000000FF */
#define TIM_RCR_REP               TIM_RCR_REP_Msk                              /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  ******************/
#define TIM_CCR1_CCR1_Pos         (0U)
#define TIM_CCR1_CCR1_Msk         (0xFFFFUL << TIM_CCR1_CCR1_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1             TIM_CCR1_CCR1_Msk                            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  ******************/
#define TIM_CCR2_CCR2_Pos         (0U)
#define TIM_CCR2_CCR2_Msk         (0xFFFFUL << TIM_CCR2_CCR2_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2             TIM_CCR2_CCR2_Msk                            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  ******************/
#define TIM_CCR3_CCR3_Pos         (0U)
#define TIM_CCR3_CCR3_Msk         (0xFFFFUL << TIM_CCR3_CCR3_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3             TIM_CCR3_CCR3_Msk                            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  ******************/
#define TIM_CCR4_CCR4_Pos         (0U)
#define TIM_CCR4_CCR4_Msk         (0xFFFFUL << TIM_CCR4_CCR4_Pos)              /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4             TIM_CCR4_CCR4_Msk                            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  ******************/
#define TIM_BDTR_DTG_Pos          (0U)
#define TIM_BDTR_DTG_Msk          (0xFFUL << TIM_BDTR_DTG_Pos)                  /*!< 0x000000FF */
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk                              /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0            (0x01UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000001 */
#define TIM_BDTR_DTG_1            (0x02UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000002 */
#define TIM_BDTR_DTG_2            (0x04UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000004 */
#define TIM_BDTR_DTG_3            (0x08UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000008 */
#define TIM_BDTR_DTG_4            (0x10UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000010 */
#define TIM_BDTR_DTG_5            (0x20UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000020 */
#define TIM_BDTR_DTG_6            (0x40UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000040 */
#define TIM_BDTR_DTG_7            (0x80UL << TIM_BDTR_DTG_Pos)                  /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos         (8U)
#define TIM_BDTR_LOCK_Msk         (0x3UL << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000300 */
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk                             /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0           (0x1UL << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1           (0x2UL << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos         (10U)
#define TIM_BDTR_OSSI_Msk         (0x1UL << TIM_BDTR_OSSI_Pos)                  /*!< 0x00000400 */
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk                             /*!<Off-State Selection for Idle mode */

#define TIM_BDTR_OSSR_Pos         (11U)
#define TIM_BDTR_OSSR_Msk         (0x1UL << TIM_BDTR_OSSR_Pos)                  /*!< 0x00000800 */
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk                             /*!<Off-State Selection for Run mode */

#define TIM_BDTR_BKE_Pos          (12U)
#define TIM_BDTR_BKE_Msk          (0x1UL << TIM_BDTR_BKE_Pos)                   /*!< 0x00001000 */
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk                              /*!<Break enable */

#define TIM_BDTR_BKP_Pos          (13U)
#define TIM_BDTR_BKP_Msk          (0x1UL << TIM_BDTR_BKP_Pos)                   /*!< 0x00002000 */
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk                              /*!<Break Polarity */

#define TIM_BDTR_AOE_Pos          (14U)
#define TIM_BDTR_AOE_Msk          (0x1UL << TIM_BDTR_AOE_Pos)                   /*!< 0x00004000 */
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk                              /*!<Automatic Output enable */

#define TIM_BDTR_MOE_Pos          (15U)
#define TIM_BDTR_MOE_Msk          (0x1UL << TIM_BDTR_MOE_Pos)                   /*!< 0x00008000 */
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk                              /*!<Main Output enable */

/*******************  Bit definition for TIM14_OR register  *******************/
#define TIM14_OR_TI1_RMP_Pos      (0U)
#define TIM14_OR_TI1_RMP_Msk      (0x3UL << TIM14_OR_TI1_RMP_Pos)               /*!< 0x00000003 */
#define TIM14_OR_TI1_RMP          TIM14_OR_TI1_RMP_Msk                          /*!<TI1_RMP[1:0] bits (TIM14 Input 4 remap) */
#define TIM14_OR_TI1_RMP_0        (0x1UL << TIM14_OR_TI1_RMP_Pos)               /*!< 0x00000001 */
#define TIM14_OR_TI1_RMP_1        (0x2UL << TIM14_OR_TI1_RMP_Pos)               /*!< 0x00000002 */


/******************************************************************************/
/*                                                                            */
/*                             Touch Key Clock (TK)                          */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TK_CR1 register  ********************/
#define TK_CR1_EN_Pos                       (0U)
#define TK_CR1_EN_Msk                       (0x1UL << TK_CR1_EN_Pos)        /*!< 0x00000001 */
#define TK_CR1_EN                           TK_CR1_EN_Msk                   /*!< TK Enable */

#define TK_CR1_MODE_Pos                     (1U)
#define TK_CR1_MODE_Msk                     (0x3UL << TK_CR1_MODE_Pos)      /*!< 0x00000006 */
#define TK_CR1_MODE                         TK_CR1_MODE_Msk                 /*!< TK Enable */
#define TK_CR1_MODE_0                       (0x1UL << TK_CR1_MODE_Pos)      /*!< 0x00000002 */
#define TK_CR1_MODE_1                       (0x2UL << TK_CR1_MODE_Pos)      /*!< 0x00000004 */

#define TK_CR1_EXTCMODEN_Pos                (3U)
#define TK_CR1_EXTCMODEN_Msk                (0x1UL << TK_CR1_EXTCMODEN_Pos) /*!< 0x00000008 */
#define TK_CR1_EXTCMODEN                    TK_CR1_EXTCMODEN_Msk            /*!< EXTCMOD Enable */

#define TK_CR1_ICOPA_Pos                    (4U)
#define TK_CR1_ICOPA_Msk                    (0x1UL << TK_CR1_ICOPA_Pos)     /*!< 0x00000010 */
#define TK_CR1_ICOPA                        TK_CR1_ICOPA_Msk                /*!< Sidac current select */

#define TK_CR1_COPAMODE_Pos                 (5U)
#define TK_CR1_COPAMODE_Msk                 (0x1UL << TK_CR1_COPAMODE_Pos)  /*!< 0x00000020 */
#define TK_CR1_COPAMODE                     TK_CR1_COPAMODE_Msk            

#define TK_CR1_COPADATA_Pos                 (6U)
#define TK_CR1_COPADATA_Msk                 (0x3UL << TK_CR1_COPADATA_Pos)  /*!< 0x000000C0 */
#define TK_CR1_COPADATA                     TK_CR1_COPADATA_Msk             
#define TK_CR1_COPADATA_0                   (0x1UL << TK_CR1_COPADATA_Pos)  /*!< 0x00000040 */
#define TK_CR1_COPADATA_1                   (0x2UL << TK_CR1_COPADATA_Pos)  /*!< 0x00000080 */

#define TK_CR1_VCCTKCR_Pos                  (8U)
#define TK_CR1_VCCTKCR_Msk                  (0x7UL << TK_CR1_VCCTKCR_Pos)   /*!< 0x00000700 */
#define TK_CR1_VCCTKCR                      TK_CR1_VCCTKCR_Msk              /*!< VCCTK voltage selection */
#define TK_CR1_VCCTKCR_0                    (0x1UL << TK_CR1_VCCTKCR_Pos)   /*!< 0x00000100 */
#define TK_CR1_VCCTKCR_1                    (0x2UL << TK_CR1_VCCTKCR_Pos)   /*!< 0x00000200 */
#define TK_CR1_VCCTKCR_2                    (0x4UL << TK_CR1_VCCTKCR_Pos)   /*!< 0x00000400 */

#define TK_CR1_VCC1CR_Pos                   (11U)
#define TK_CR1_VCC1CR_Msk                   (0x7UL << TK_CR1_VCC1CR_Pos)    /*!< 0x00003800 */
#define TK_CR1_VCC1CR                       TK_CR1_VCC1CR_Msk               /*!< VCC1 voltage selection */
#define TK_CR1_VCC1CR_0                     (0x1UL << TK_CR1_VCC1CR_Pos)    /*!< 0x00000800 */
#define TK_CR1_VCC1CR_1                     (0x2UL << TK_CR1_VCC1CR_Pos)    /*!< 0x00001000 */
#define TK_CR1_VCC1CR_2                     (0x4UL << TK_CR1_VCC1CR_Pos)    /*!< 0x00002000 */

#define TK_CR1_MIDACSTEP_Pos                (14U)
#define TK_CR1_MIDACSTEP_Msk                (0x1UL << TK_CR1_MIDACSTEP_Pos) /*!< 0x00004000 */
#define TK_CR1_MIDACSTEP                    TK_CR1_MIDACSTEP_Msk            /*!< MIDAC Step Select */

#define TK_CR1_SIDACSTEP_Pos                (15U)
#define TK_CR1_SIDACSTEP_Msk                (0x1UL << TK_CR1_SIDACSTEP_Pos) /*!< 0x00008000 */
#define TK_CR1_SIDACSTEP                    TK_CR1_SIDACSTEP_Msk            /*!< SIDAC Step Select */

#define TK_CR1_CFT_Pos                      (16U)
#define TK_CR1_CFT_Msk                      (0x7UL << TK_CR1_CFT_Pos)       /*!< 0x00070000 */
#define TK_CR1_CFT                          TK_CR1_CFT_Msk                   
#define TK_CR1_CFT_0                        (0x1UL << TK_CR1_CFT_Pos)       /*!< 0x00010000 */
#define TK_CR1_CFT_1                        (0x2UL << TK_CR1_CFT_Pos)       /*!< 0x00020000 */
#define TK_CR1_CFT_2                        (0x4UL << TK_CR1_CFT_Pos)       /*!< 0x00040000 */

#define TK_CR1_VCCTKEN_Pos                  (19U)
#define TK_CR1_VCCTKEN_Msk                  (0x1UL << TK_CR1_VCCTKEN_Pos)   /*!< 0x00080000 */
#define TK_CR1_VCCTKEN                      TK_CR1_VCCTKEN_Msk   

#define TK_CR1_INTCMODEN_Pos                (20U)
#define TK_CR1_INTCMODEN_Msk                (0x1UL << TK_CR1_INTCMODEN_Pos) /*!< 0x00100000 */
#define TK_CR1_INTCMODEN                    TK_CR1_INTCMODEN_Msk   

#define TK_CR1_CMPHYSEN_Pos                 (21U)
#define TK_CR1_CMPHYSEN_Msk                 (0x1UL << TK_CR1_CMPHYSEN_Pos)   /*!< 0x00200000 */
#define TK_CR1_CMPHYSEN                     TK_CR1_CMPHYSEN_Msk   

#define TK_CR1_BUFFEREN_Pos                 (22U)
#define TK_CR1_BUFFEREN_Msk                 (0x1UL << TK_CR1_BUFFEREN_Pos)   /*!< 0x00400000 */
#define TK_CR1_BUFFEREN                     TK_CR1_BUFFEREN_Msk   

#define TK_CR1_DIV_Pos                      (24U)
#define TK_CR1_DIV_Msk                      (0x7UL << TK_CR1_DIV_Pos)       /*!< 0x07000000 */
#define TK_CR1_DIV                          TK_CR1_DIV_Msk                  /*!< TK Clock div */
#define TK_CR1_DIV_0                        (0x1UL << TK_CR1_DIV_Pos)       /*!< 0x01000000 */
#define TK_CR1_DIV_1                        (0x2UL << TK_CR1_DIV_Pos)       /*!< 0x02000000 */
#define TK_CR1_DIV_2                        (0x4UL << TK_CR1_DIV_Pos)       /*!< 0x04000000 */

#define TK_CR1_STDIV_Pos                    (27U)
#define TK_CR1_STDIV_Msk                    (0x7UL << TK_CR1_STDIV_Pos)     /*!< 0x38000000 */
#define TK_CR1_STDIV                        TK_CR1_STDIV_Msk                /*!< ST Clock div */
#define TK_CR1_STDIV_0                      (0x1UL << TK_CR1_STDIV_Pos)     /*!< 0x08000000 */
#define TK_CR1_STDIV_1                      (0x2UL << TK_CR1_STDIV_Pos)     /*!< 0x10000000 */
#define TK_CR1_STDIV_2                      (0x4UL << TK_CR1_STDIV_Pos)     /*!< 0x20000000 */

/*******************  Bit definition for TK_CR2 register  ********************/
#define TK_CR2_START_Pos                    (0U)
#define TK_CR2_START_Msk                    (0x1UL << TK_CR2_START_Pos)      /*!< 0x00000001 */
#define TK_CR2_START                        TK_CR2_START_Msk                 

#define TK_CR2_RSTB_Pos                     (1U)
#define TK_CR2_RSTB_Msk                     (0x1UL << TK_CR2_RSTB_Pos)       /*!< 0x00000002 */
#define TK_CR2_RSTB                         TK_CR2_RSTB_Msk                 

#define TK_CR2_SWCTRL_Pos                   (2U)
#define TK_CR2_SWCTRL_Msk                   (0x1UL << TK_CR2_SWCTRL_Pos)     /*!< 0x00000004 */
#define TK_CR2_SWCTRL                       TK_CR2_SWCTRL_Msk                

#define TK_CR2_CMODCHRG_Pos                 (3U)
#define TK_CR2_CMODCHRG_Msk                 (0x1UL << TK_CR2_CMODCHRG_Pos)   /*!< 0x00000008 */
#define TK_CR2_CMODCHRG                     TK_CR2_CMODCHRG_Msk              

#define TK_CR2_CMODDISCH_Pos                (4U)
#define TK_CR2_CMODDISCH_Msk                (0x1UL << TK_CR2_CMODDISCH_Pos)  /*!< 0x00000010 */
#define TK_CR2_CMODDISCH                    TK_CR2_CMODDISCH_Msk                 

#define TK_CR2_MIDACEN_Pos                  (5U)
#define TK_CR2_MIDACEN_Msk                  (0x1UL << TK_CR2_MIDACEN_Pos)    /*!< 0x00000020 */
#define TK_CR2_MIDACEN                      TK_CR2_MIDACEN_Msk                   

#define TK_CR2_SIDACEN_Pos                  (6U)
#define TK_CR2_SIDACEN_Msk                  (0x1UL << TK_CR2_SIDACEN_Pos)    /*!< 0x00000040 */
#define TK_CR2_SIDACEN                      TK_CR2_SIDACEN_Msk                   

#define TK_CR2_CMODVTINIT_Pos               (7U)
#define TK_CR2_CMODVTINIT_Msk               (0x1UL << TK_CR2_CMODVTINIT_Pos)  /*!< 0x00000080 */
#define TK_CR2_CMODVTINIT                   TK_CR2_CMODVTINIT_Msk             

#define TK_CR2_PWMM_Pos                     (8U)
#define TK_CR2_PWMM_Msk                     (0x1UL << TK_CR2_PWMM_Pos)           /*!< 0x00000100 */
#define TK_CR2_PWMM                         TK_CR2_PWMM_Msk                      

#define TK_CR2_CTRLSOURCE_Pos               (9U)
#define TK_CR2_CTRLSOURCE_Msk               (0x1UL << TK_CR2_CTRLSOURCE_Pos)     /*!< 0x00000200 */
#define TK_CR2_CTRLSOURCE                   TK_CR2_CTRLSOURCE_Msk                

#define TK_CR2_LPABNORMAL_Pos               (10U)
#define TK_CR2_LPABNORMAL_Msk               (0x1UL << TK_CR2_LPABNORMAL_Pos)     /*!< 0x00000400 */
#define TK_CR2_LPABNORMAL                   TK_CR2_LPABNORMAL_Msk     

#define TK_CR2_CSA_DISCH_SEL_Pos            (11U)
#define TK_CR2_CSA_DISCH_SEL_Msk            (0x1UL << TK_CR2_CSA_DISCH_SEL_Pos)  /*!< 0x00000800 */
#define TK_CR2_CSA_DISCH_SEL                TK_CR2_CSA_DISCH_SEL_Msk     

#define TK_CR2_SHORTSW_NUM_Pos              (12U)
#define TK_CR2_SHORTSW_NUM_Msk              (0x3UL << TK_CR2_SHORTSW_NUM_Pos)    /*!< 0x00003000 */
#define TK_CR2_SHORTSW_NUM                  TK_CR2_SHORTSW_NUM_Msk                     
#define TK_CR2_SHORTSW_NUM_0                (0x1UL << TK_CR2_SHORTSW_NUM_Pos)    /*!< 0x00001000 */
#define TK_CR2_SHORTSW_NUM_1                (0x2UL << TK_CR2_SHORTSW_NUM_Pos)    /*!< 0x00002000 */

#define TK_CR2_LPEN_Pos                     (14U)
#define TK_CR2_LPEN_Msk                     (0x1UL << TK_CR2_LPEN_Pos)            /*!< 0x00004000 */
#define TK_CR2_LPEN                         TK_CR2_LPEN_Msk     

#define TK_CR2_KVREF_Pos                    (16U)
#define TK_CR2_KVREF_Msk                    (0x3UL << TK_CR2_KVREF_Pos)          /*!< 0x00030000 */
#define TK_CR2_KVREF                        TK_CR2_KVREF_Msk                     
#define TK_CR2_KVREF_0                      (0x1UL << TK_CR2_KVREF_Pos)          /*!< 0x00010000 */
#define TK_CR2_KVREF_1                      (0x2UL << TK_CR2_KVREF_Pos)          /*!< 0x00020000 */

#define TK_CR2_KVREF_CSA_TARGET_Pos         (18U)
#define TK_CR2_KVREF_CSA_TARGET_Msk         (0x3UL << TK_CR2_KVREF_CSA_TARGET_Pos)        /*!< 0x000C0000 */
#define TK_CR2_KVREF_CSA_TARGET             TK_CR2_KVREF_CSA_TARGET_Msk                   
#define TK_CR2_KVREF_CSA_TARGET_0           (0x1UL << TK_CR2_KVREF_CSA_TARGET_Pos)        /*!< 0x00040000 */
#define TK_CR2_KVREF_CSA_TARGET_1           (0x2UL << TK_CR2_KVREF_CSA_TARGET_Pos)        /*!< 0x00080000 */

#define TK_CR2_LFSRW_Pos                    (20U)
#define TK_CR2_LFSRW_Msk                    (0xFUL << TK_CR2_LFSRW_Pos)          /*!< 0x00F00000 */
#define TK_CR2_LFSRW                        TK_CR2_LFSRW_Msk                     
#define TK_CR2_LFSRW_0                      (0x1UL << TK_CR2_LFSRW_Pos)          /*!< 0x00100000 */
#define TK_CR2_LFSRW_1                      (0x2UL << TK_CR2_LFSRW_Pos)          /*!< 0x00200000 */
#define TK_CR2_LFSRW_2                      (0x4UL << TK_CR2_LFSRW_Pos)          /*!< 0x00400000 */
#define TK_CR2_LFSRW_3                      (0x8UL << TK_CR2_LFSRW_Pos)          /*!< 0x00800000 */

/*******************  Bit definition for TK_IDACR register  ********************/
#define TK_IDACR_MIDAC_Pos                  (0U)
#define TK_IDACR_MIDAC_Msk                  (0xFFUL << TK_IDACR_MIDAC_Pos)                /*!< 0x000000FF */
#define TK_IDACR_MIDAC                      TK_IDACR_MIDAC_Msk                   
#define TK_IDACR_MIDAC_0                    (0x1UL << TK_IDACR_MIDAC_Pos)                 /*!< 0x00000001 */
#define TK_IDACR_MIDAC_1                    (0x2UL << TK_IDACR_MIDAC_Pos)                 /*!< 0x00000002 */
#define TK_IDACR_MIDAC_2                    (0x4UL << TK_IDACR_MIDAC_Pos)                 /*!< 0x00000004 */
#define TK_IDACR_MIDAC_3                    (0x8UL << TK_IDACR_MIDAC_Pos)                 /*!< 0x00000008 */
#define TK_IDACR_MIDAC_4                    (0x10UL << TK_IDACR_MIDAC_Pos)                /*!< 0x00000010 */
#define TK_IDACR_MIDAC_5                    (0x20UL << TK_IDACR_MIDAC_Pos)                /*!< 0x00000020 */
#define TK_IDACR_MIDAC_6                    (0x40UL << TK_IDACR_MIDAC_Pos)                /*!< 0x00000040 */
#define TK_IDACR_MIDAC_7                    (0x80UL << TK_IDACR_MIDAC_Pos)                /*!< 0x00000080 */

#define TK_IDACR_SIDAC_Pos                  (16U)
#define TK_IDACR_SIDAC_Msk                  (0xFFUL << TK_IDACR_SIDAC_Pos)                /*!< 0x00FF0000 */
#define TK_IDACR_SIDAC                      TK_IDACR_SIDAC_Msk                   
#define TK_IDACR_SIDAC_0                    (0x1UL << TK_IDACR_SIDAC_Pos)                 /*!< 0x00010000 */
#define TK_IDACR_SIDAC_1                    (0x2UL << TK_IDACR_SIDAC_Pos)                 /*!< 0x00020000 */
#define TK_IDACR_SIDAC_2                    (0x4UL << TK_IDACR_SIDAC_Pos)                 /*!< 0x00040000 */
#define TK_IDACR_SIDAC_3                    (0x8UL << TK_IDACR_SIDAC_Pos)                 /*!< 0x00080000 */
#define TK_IDACR_SIDAC_4                    (0x10UL << TK_IDACR_SIDAC_Pos)                /*!< 0x00100000 */
#define TK_IDACR_SIDAC_5                    (0x20UL << TK_IDACR_SIDAC_Pos)                /*!< 0x00200000 */
#define TK_IDACR_SIDAC_6                    (0x40UL << TK_IDACR_SIDAC_Pos)                /*!< 0x00400000 */
#define TK_IDACR_SIDAC_7                    (0x80UL << TK_IDACR_SIDAC_Pos)                /*!< 0x00800000 */

#define TK_IDACR_SIDAC_CSA_TARGET_Pos       (24U)
#define TK_IDACR_SIDAC_CSA_TARGET_Msk       (0xFFUL << TK_IDACR_SIDAC_CSA_TARGET_Pos)     /*!< 0xFF000000 */
#define TK_IDACR_SIDAC_CSA_TARGET           TK_IDACR_SIDAC_CSA_TARGET_Msk                   
#define TK_IDACR_SIDAC_CSA_TARGET_0         (0x1UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)      /*!< 0x01000000 */
#define TK_IDACR_SIDAC_CSA_TARGET_1         (0x2UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)      /*!< 0x02000000 */
#define TK_IDACR_SIDAC_CSA_TARGET_2         (0x4UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)      /*!< 0x04000000 */
#define TK_IDACR_SIDAC_CSA_TARGET_3         (0x8UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)      /*!< 0x08000000 */
#define TK_IDACR_SIDAC_CSA_TARGET_4         (0x10UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)     /*!< 0x10000000 */
#define TK_IDACR_SIDAC_CSA_TARGET_5         (0x20UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)     /*!< 0x20000000 */
#define TK_IDACR_SIDAC_CSA_TARGET_6         (0x40UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)     /*!< 0x40000000 */
#define TK_IDACR_SIDAC_CSA_TARGET_7         (0x80UL << TK_IDACR_SIDAC_CSA_TARGET_Pos)     /*!< 0x80000000 */

/*******************  Bit definition for TK_KR register  ********************/
#define TK_KR_KEN_Pos                       (0U)
#define TK_KR_KEN_Msk                       (0x3FFFFFFUL << TK_KR_KEN_Pos)           /*!< 0x03FFFFFF */
#define TK_KR_KEN                           TK_KR_KEN_Msk         
#define TK_KR_KEN_0                         (0x1UL << TK_KR_KEN_Pos)           /*!< 0x00000001 */
#define TK_KR_KEN_1                         (0x2UL << TK_KR_KEN_Pos)           /*!< 0x00000002 */
#define TK_KR_KEN_2                         (0x4UL << TK_KR_KEN_Pos)           /*!< 0x00000004 */
#define TK_KR_KEN_3                         (0x8UL << TK_KR_KEN_Pos)           /*!< 0x00000008 */
#define TK_KR_KEN_4                         (0x10UL << TK_KR_KEN_Pos)          /*!< 0x00000010 */
#define TK_KR_KEN_5                         (0x20UL << TK_KR_KEN_Pos)          /*!< 0x00000020 */
#define TK_KR_KEN_6                         (0x40UL << TK_KR_KEN_Pos)          /*!< 0x00000040 */
#define TK_KR_KEN_7                         (0x80UL << TK_KR_KEN_Pos)          /*!< 0x00000080 */
#define TK_KR_KEN_8                         (0x100UL << TK_KR_KEN_Pos)         /*!< 0x00000100 */
#define TK_KR_KEN_9                         (0x200UL << TK_KR_KEN_Pos)         /*!< 0x00000200 */
#define TK_KR_KEN_10                        (0x400UL << TK_KR_KEN_Pos)         /*!< 0x00000400 */
#define TK_KR_KEN_11                        (0x800UL << TK_KR_KEN_Pos)         /*!< 0x00000800 */
#define TK_KR_KEN_12                        (0x1000UL << TK_KR_KEN_Pos)        /*!< 0x00001000 */
#define TK_KR_KEN_13                        (0x2000UL << TK_KR_KEN_Pos)        /*!< 0x00002000 */
#define TK_KR_KEN_14                        (0x4000UL << TK_KR_KEN_Pos)        /*!< 0x00004000 */
#define TK_KR_KEN_15                        (0x8000UL << TK_KR_KEN_Pos)        /*!< 0x00008000 */
#define TK_KR_KEN_16                        (0x10000UL << TK_KR_KEN_Pos)       /*!< 0x00010000 */
#define TK_KR_KEN_17                        (0x20000UL << TK_KR_KEN_Pos)       /*!< 0x00020000 */
#define TK_KR_KEN_18                        (0x40000UL << TK_KR_KEN_Pos)       /*!< 0x00040000 */
#define TK_KR_KEN_19                        (0x80000UL << TK_KR_KEN_Pos)       /*!< 0x00080000 */
#define TK_KR_KEN_20                        (0x100000UL << TK_KR_KEN_Pos)      /*!< 0x00100000 */
#define TK_KR_KEN_21                        (0x200000UL << TK_KR_KEN_Pos)      /*!< 0x00200000 */
#define TK_KR_KEN_22                        (0x400000UL << TK_KR_KEN_Pos)      /*!< 0x00400000 */
#define TK_KR_KEN_23                        (0x800000UL << TK_KR_KEN_Pos)      /*!< 0x00800000 */
#define TK_KR_KEN_24                        (0x1000000UL << TK_KR_KEN_Pos)     /*!< 0x01000000 */
#define TK_KR_KEN_25                        (0x2000000UL << TK_KR_KEN_Pos)     /*!< 0x02000000 */

/*******************  Bit definition for TK_CSR register  ********************/
#define TK_CSR_CHS_Pos                       (0U)
#define TK_CSR_CHS_Msk                       (0x3FFFFFFUL << TK_CSR_CHS_Pos)           /*!< 0x03FFFFFF */
#define TK_CSR_CHS                           TK_CSR_CHS_Msk         
#define TK_CSR_CHS_0                         (0x1UL << TK_CSR_CHS_Pos)           /*!< 0x00000001 */
#define TK_CSR_CHS_1                         (0x2UL << TK_CSR_CHS_Pos)           /*!< 0x00000002 */
#define TK_CSR_CHS_2                         (0x4UL << TK_CSR_CHS_Pos)           /*!< 0x00000004 */
#define TK_CSR_CHS_3                         (0x8UL << TK_CSR_CHS_Pos)           /*!< 0x00000008 */
#define TK_CSR_CHS_4                         (0x10UL << TK_CSR_CHS_Pos)          /*!< 0x00000010 */
#define TK_CSR_CHS_5                         (0x20UL << TK_CSR_CHS_Pos)          /*!< 0x00000020 */
#define TK_CSR_CHS_6                         (0x40UL << TK_CSR_CHS_Pos)          /*!< 0x00000040 */
#define TK_CSR_CHS_7                         (0x80UL << TK_CSR_CHS_Pos)          /*!< 0x00000080 */
#define TK_CSR_CHS_8                         (0x100UL << TK_CSR_CHS_Pos)         /*!< 0x00000100 */
#define TK_CSR_CHS_9                         (0x200UL << TK_CSR_CHS_Pos)         /*!< 0x00000200 */
#define TK_CSR_CHS_10                        (0x400UL << TK_CSR_CHS_Pos)         /*!< 0x00000400 */
#define TK_CSR_CHS_11                        (0x800UL << TK_CSR_CHS_Pos)         /*!< 0x00000800 */
#define TK_CSR_CHS_12                        (0x1000UL << TK_CSR_CHS_Pos)        /*!< 0x00001000 */
#define TK_CSR_CHS_13                        (0x2000UL << TK_CSR_CHS_Pos)        /*!< 0x00002000 */
#define TK_CSR_CHS_14                        (0x4000UL << TK_CSR_CHS_Pos)        /*!< 0x00004000 */
#define TK_CSR_CHS_15                        (0x8000UL << TK_CSR_CHS_Pos)        /*!< 0x00008000 */
#define TK_CSR_CHS_16                        (0x10000UL << TK_CSR_CHS_Pos)       /*!< 0x00010000 */
#define TK_CSR_CHS_17                        (0x20000UL << TK_CSR_CHS_Pos)       /*!< 0x00020000 */
#define TK_CSR_CHS_18                        (0x40000UL << TK_CSR_CHS_Pos)       /*!< 0x00040000 */
#define TK_CSR_CHS_19                        (0x80000UL << TK_CSR_CHS_Pos)       /*!< 0x00080000 */
#define TK_CSR_CHS_20                        (0x100000UL << TK_CSR_CHS_Pos)      /*!< 0x00100000 */
#define TK_CSR_CHS_21                        (0x200000UL << TK_CSR_CHS_Pos)      /*!< 0x00200000 */
#define TK_CSR_CHS_22                        (0x400000UL << TK_CSR_CHS_Pos)      /*!< 0x00400000 */
#define TK_CSR_CHS_23                        (0x800000UL << TK_CSR_CHS_Pos)      /*!< 0x00800000 */
#define TK_CSR_CHS_24                        (0x1000000UL << TK_CSR_CHS_Pos)     /*!< 0x01000000 */
#define TK_CSR_CHS_25                        (0x2000000UL << TK_CSR_CHS_Pos)     /*!< 0x02000000 */

/*******************  Bit definition for TK_UCR register  ********************/
#define TK_UCR_COPAMODEIO_Pos                       (0U)
#define TK_UCR_COPAMODEIO_Msk                       (0x3FFFFFFUL << TK_UCR_COPAMODEIO_Pos)           /*!< 0x03FFFFFF */
#define TK_UCR_COPAMODEIO                           TK_UCR_COPAMODEIO_Msk         
#define TK_UCR_COPAMODEIO_0                         (0x1UL << TK_UCR_COPAMODEIO_Pos)           /*!< 0x00000001 */
#define TK_UCR_COPAMODEIO_1                         (0x2UL << TK_UCR_COPAMODEIO_Pos)           /*!< 0x00000002 */
#define TK_UCR_COPAMODEIO_2                         (0x4UL << TK_UCR_COPAMODEIO_Pos)           /*!< 0x00000004 */
#define TK_UCR_COPAMODEIO_3                         (0x8UL << TK_UCR_COPAMODEIO_Pos)           /*!< 0x00000008 */
#define TK_UCR_COPAMODEIO_4                         (0x10UL << TK_UCR_COPAMODEIO_Pos)          /*!< 0x00000010 */
#define TK_UCR_COPAMODEIO_5                         (0x20UL << TK_UCR_COPAMODEIO_Pos)          /*!< 0x00000020 */
#define TK_UCR_COPAMODEIO_6                         (0x40UL << TK_UCR_COPAMODEIO_Pos)          /*!< 0x00000040 */
#define TK_UCR_COPAMODEIO_7                         (0x80UL << TK_UCR_COPAMODEIO_Pos)          /*!< 0x00000080 */
#define TK_UCR_COPAMODEIO_8                         (0x100UL << TK_UCR_COPAMODEIO_Pos)         /*!< 0x00000100 */
#define TK_UCR_COPAMODEIO_9                         (0x200UL << TK_UCR_COPAMODEIO_Pos)         /*!< 0x00000200 */
#define TK_UCR_COPAMODEIO_10                        (0x400UL << TK_UCR_COPAMODEIO_Pos)         /*!< 0x00000400 */
#define TK_UCR_COPAMODEIO_11                        (0x800UL << TK_UCR_COPAMODEIO_Pos)         /*!< 0x00000800 */
#define TK_UCR_COPAMODEIO_12                        (0x1000UL << TK_UCR_COPAMODEIO_Pos)        /*!< 0x00001000 */
#define TK_UCR_COPAMODEIO_13                        (0x2000UL << TK_UCR_COPAMODEIO_Pos)        /*!< 0x00002000 */
#define TK_UCR_COPAMODEIO_14                        (0x4000UL << TK_UCR_COPAMODEIO_Pos)        /*!< 0x00004000 */
#define TK_UCR_COPAMODEIO_15                        (0x8000UL << TK_UCR_COPAMODEIO_Pos)        /*!< 0x00008000 */
#define TK_UCR_COPAMODEIO_16                        (0x10000UL << TK_UCR_COPAMODEIO_Pos)       /*!< 0x00010000 */
#define TK_UCR_COPAMODEIO_17                        (0x20000UL << TK_UCR_COPAMODEIO_Pos)       /*!< 0x00020000 */
#define TK_UCR_COPAMODEIO_18                        (0x40000UL << TK_UCR_COPAMODEIO_Pos)       /*!< 0x00040000 */
#define TK_UCR_COPAMODEIO_19                        (0x80000UL << TK_UCR_COPAMODEIO_Pos)       /*!< 0x00080000 */
#define TK_UCR_COPAMODEIO_20                        (0x100000UL << TK_UCR_COPAMODEIO_Pos)      /*!< 0x00100000 */
#define TK_UCR_COPAMODEIO_21                        (0x200000UL << TK_UCR_COPAMODEIO_Pos)      /*!< 0x00200000 */
#define TK_UCR_COPAMODEIO_22                        (0x400000UL << TK_UCR_COPAMODEIO_Pos)      /*!< 0x00400000 */
#define TK_UCR_COPAMODEIO_23                        (0x800000UL << TK_UCR_COPAMODEIO_Pos)      /*!< 0x00800000 */
#define TK_UCR_COPAMODEIO_24                        (0x1000000UL << TK_UCR_COPAMODEIO_Pos)     /*!< 0x01000000 */
#define TK_UCR_COPAMODEIO_25                        (0x2000000UL << TK_UCR_COPAMODEIO_Pos)     /*!< 0x02000000 */

/*******************  Bit definition for TK_STR1 register  ********************/
#define TK_STR1_SETTIME_Pos                 (0U)
#define TK_STR1_SETTIME_Msk                 (0x3FFUL << TK_STR1_SETTIME_Pos)        /*!< 0x000003FF */
#define TK_STR1_SETTIME                     TK_STR1_SETTIME_Msk     
#define TK_STR1_SETTIME_0                   (0x1UL << TK_STR1_SETTIME_Pos)          /*!< 0x00000001 */
#define TK_STR1_SETTIME_1                   (0x2UL << TK_STR1_SETTIME_Pos)          /*!< 0x00000002 */
#define TK_STR1_SETTIME_2                   (0x4UL << TK_STR1_SETTIME_Pos)          /*!< 0x00000004 */
#define TK_STR1_SETTIME_3                   (0x8UL << TK_STR1_SETTIME_Pos)          /*!< 0x00000008 */
#define TK_STR1_SETTIME_4                   (0x10UL << TK_STR1_SETTIME_Pos)         /*!< 0x00000010 */
#define TK_STR1_SETTIME_5                   (0x20UL << TK_STR1_SETTIME_Pos)         /*!< 0x00000020 */
#define TK_STR1_SETTIME_6                   (0x40UL << TK_STR1_SETTIME_Pos)         /*!< 0x00000040 */
#define TK_STR1_SETTIME_7                   (0x80UL << TK_STR1_SETTIME_Pos)         /*!< 0x00000080 */
#define TK_STR1_SETTIME_8                   (0x100UL << TK_STR1_SETTIME_Pos)        /*!< 0x00000100 */
#define TK_STR1_SETTIME_9                   (0x200UL << TK_STR1_SETTIME_Pos)        /*!< 0x00000200 */

#define TK_STR1_VTINITTIME_Pos              (16U)
#define TK_STR1_VTINITTIME_Msk              (0xFFUL << TK_STR1_VTINITTIME_Pos)      /*!< 0x00FF0000 */
#define TK_STR1_VTINITTIME                  TK_STR1_VTINITTIME_Msk     
#define TK_STR1_VTINITTIME_0                (0x1UL << TK_STR1_VTINITTIME_Pos)       /*!< 0x00010000 */
#define TK_STR1_VTINITTIME_1                (0x2UL << TK_STR1_VTINITTIME_Pos)       /*!< 0x00020000 */
#define TK_STR1_VTINITTIME_2                (0x4UL << TK_STR1_VTINITTIME_Pos)       /*!< 0x00040000 */
#define TK_STR1_VTINITTIME_3                (0x8UL << TK_STR1_VTINITTIME_Pos)       /*!< 0x00080000 */
#define TK_STR1_VTINITTIME_4                (0x10UL << TK_STR1_VTINITTIME_Pos)      /*!< 0x00100000 */
#define TK_STR1_VTINITTIME_5                (0x20UL << TK_STR1_VTINITTIME_Pos)      /*!< 0x00200000 */
#define TK_STR1_VTINITTIME_6                (0x40UL << TK_STR1_VTINITTIME_Pos)      /*!< 0x00400000 */
#define TK_STR1_VTINITTIME_7                (0x80UL << TK_STR1_VTINITTIME_Pos)      /*!< 0x00800000 */

/*******************  Bit definition for TK_STR2 register  ********************/
#define TK_STR2_VREF_MIDAC_TIME_Pos         (0U)
#define TK_STR2_VREF_MIDAC_TIME_Msk         (0x1FUL << TK_STR2_VREF_MIDAC_TIME_Pos) /*!< 0x0000001F */
#define TK_STR2_VREF_MIDAC_TIME             TK_STR2_VREF_MIDAC_TIME_Msk                      
#define TK_STR2_VREF_MIDAC_TIME_0           (0x1UL << TK_STR2_VREF_MIDAC_TIME_Pos)  /*!< 0x00000001 */
#define TK_STR2_VREF_MIDAC_TIME_1           (0x2UL << TK_STR2_VREF_MIDAC_TIME_Pos)  /*!< 0x00000002 */
#define TK_STR2_VREF_MIDAC_TIME_2           (0x4UL << TK_STR2_VREF_MIDAC_TIME_Pos)  /*!< 0x00000004 */
#define TK_STR2_VREF_MIDAC_TIME_3           (0x8UL << TK_STR2_VREF_MIDAC_TIME_Pos)  /*!< 0x00000008 */
#define TK_STR2_VREF_MIDAC_TIME_4           (0x10UL << TK_STR2_VREF_MIDAC_TIME_Pos) /*!< 0x00000010 */

#define TK_STR2_DISCH_CHRG_TIME_Pos         (8U)
#define TK_STR2_DISCH_CHRG_TIME_Msk         (0x7FUL << TK_STR2_DISCH_CHRG_TIME_Pos) /*!< 0x00007F00 */
#define TK_STR2_DISCH_CHRG_TIME             TK_STR2_DISCH_CHRG_TIME_Msk                     
#define TK_STR2_DISCH_CHRG_TIME_0           (0x1UL << TK_STR2_DISCH_CHRG_TIME_Pos)  /*!< 0x00000100 */
#define TK_STR2_DISCH_CHRG_TIME_1           (0x2UL << TK_STR2_DISCH_CHRG_TIME_Pos)  /*!< 0x00000200 */
#define TK_STR2_DISCH_CHRG_TIME_2           (0x4UL << TK_STR2_DISCH_CHRG_TIME_Pos)  /*!< 0x00000400 */
#define TK_STR2_DISCH_CHRG_TIME_3           (0x8UL << TK_STR2_DISCH_CHRG_TIME_Pos)  /*!< 0x00000800 */
#define TK_STR2_DISCH_CHRG_TIME_4           (0x10UL << TK_STR2_DISCH_CHRG_TIME_Pos) /*!< 0x00001000 */
#define TK_STR2_DISCH_CHRG_TIME_5           (0x20UL << TK_STR2_DISCH_CHRG_TIME_Pos) /*!< 0x00002000 */
#define TK_STR2_DISCH_CHRG_TIME_6           (0x40UL << TK_STR2_DISCH_CHRG_TIME_Pos) /*!< 0x00004000 */

#define TK_STR2_SIDACTIME_Pos               (16U)
#define TK_STR2_SIDACTIME_Msk               (0x1FUL << TK_STR2_SIDACTIME_Pos)       /*!< 0x001F0000 */
#define TK_STR2_SIDACTIME                   TK_STR2_SIDACTIME_Msk  
#define TK_STR2_SIDACTIME_0                 (0x1UL << TK_STR2_SIDACTIME_Pos)        /*!< 0x00010000 */
#define TK_STR2_SIDACTIME_1                 (0x2UL << TK_STR2_SIDACTIME_Pos)        /*!< 0x00020000 */
#define TK_STR2_SIDACTIME_2                 (0x4UL << TK_STR2_SIDACTIME_Pos)        /*!< 0x00040000 */
#define TK_STR2_SIDACTIME_3                 (0x8UL << TK_STR2_SIDACTIME_Pos)        /*!< 0x00080000 */
#define TK_STR2_SIDACTIME_4                 (0x10UL << TK_STR2_SIDACTIME_Pos)       /*!< 0x00100000 */

#define TK_STR2_BEFORE_SW_TIME_Pos          (24U)
#define TK_STR2_BEFORE_SW_TIME_Msk          (0xFUL << TK_STR2_BEFORE_SW_TIME_Pos)   /*!< 0x0F000000 */
#define TK_STR2_BEFORE_SW_TIME              TK_STR2_BEFORE_SW_TIME_Msk     
#define TK_STR2_BEFORE_SW_TIME_0            (0x1UL << TK_STR2_BEFORE_SW_TIME_Pos)   /*!< 0x01000000 */
#define TK_STR2_BEFORE_SW_TIME_1            (0x2UL << TK_STR2_BEFORE_SW_TIME_Pos)   /*!< 0x02000000 */
#define TK_STR2_BEFORE_SW_TIME_2            (0x4UL << TK_STR2_BEFORE_SW_TIME_Pos)   /*!< 0x04000000 */
#define TK_STR2_BEFORE_SW_TIME_3            (0x8UL << TK_STR2_BEFORE_SW_TIME_Pos)   /*!< 0x08000000 */

#define TK_STR2_CHRG_SIDAEN_TIME_Pos        (28U)
#define TK_STR2_CHRG_SIDAEN_TIME_Msk        (0x3UL << TK_STR2_CHRG_SIDAEN_TIME_Pos) /*!< 0x30000000 */
#define TK_STR2_CHRG_SIDAEN_TIME            TK_STR2_CHRG_SIDAEN_TIME_Msk   
#define TK_STR2_CHRG_SIDAEN_TIME_0          (0x1UL << TK_STR2_CHRG_SIDAEN_TIME_Pos) /*!< 0x10000000 */
#define TK_STR2_CHRG_SIDAEN_TIME_1          (0x2UL << TK_STR2_CHRG_SIDAEN_TIME_Pos) /*!< 0x20000000 */

/*******************  Bit definition for TK_WR register  ********************/
#define TK_WR_WIN_Pos                       (0U)
#define TK_WR_WIN_Msk                       (0xFFFFUL << TK_WR_WIN_Pos)             /*!< 0x0000FFFF */
#define TK_WR_WIN                           TK_WR_WIN_Msk                           

/*******************  Bit definition for TK_SDR register  ********************/
#define TK_SDR_SWDIV_Pos                    (0U)
#define TK_SDR_SWDIV_Msk                    (0x1FFUL << TK_SDR_SWDIV_Pos)           /*!< 0x000001FF */
#define TK_SDR_SWDIV                        TK_SDR_SWDIV_Msk                        
#define TK_SDR_SWDIV_0                      (0x1UL << TK_SDR_SWDIV_Pos)             /*!< 0x00000001 */
#define TK_SDR_SWDIV_1                      (0x2UL << TK_SDR_SWDIV_Pos)             /*!< 0x00000002 */
#define TK_SDR_SWDIV_2                      (0x4UL << TK_SDR_SWDIV_Pos)             /*!< 0x00000004 */
#define TK_SDR_SWDIV_3                      (0x8UL << TK_SDR_SWDIV_Pos)             /*!< 0x00000008 */
#define TK_SDR_SWDIV_4                      (0x10UL << TK_SDR_SWDIV_Pos)            /*!< 0x00000010 */
#define TK_SDR_SWDIV_5                      (0x20UL << TK_SDR_SWDIV_Pos)            /*!< 0x00000020 */
#define TK_SDR_SWDIV_6                      (0x40UL << TK_SDR_SWDIV_Pos)            /*!< 0x00000040 */
#define TK_SDR_SWDIV_7                      (0x80UL << TK_SDR_SWDIV_Pos)            /*!< 0x00000080 */
#define TK_SDR_SWDIV_8                      (0x100UL << TK_SDR_SWDIV_Pos)           /*!< 0x00000100 */

/*******************  Bit definition for TK_PRSR register  ********************/
#define TK_PRSR_LFSRP_Pos                   (0U)
#define TK_PRSR_LFSRP_Msk                   (0xFFFFUL << TK_PRSR_LFSRP_Pos)         /*!< 0x0000FFFF */
#define TK_PRSR_LFSRP                       TK_PRSR_LFSRP_Msk                       

#define TK_PRSR_SEED_Pos                    (16U)
#define TK_PRSR_SEED_Msk                    (0xFFFFUL << TK_PRSR_SEED_Pos)          /*!< 0xFFFF0000 */
#define TK_PRSR_SEED                        TK_PRSR_SEED_Msk                        

/*******************  Bit definition for TK_DR register  ********************/
#define TK_DR_DATA_Pos                      (0U)
#define TK_DR_DATA_Msk                      (0xFFFFUL << TK_DR_DATA_Pos)            /*!< 0x0000FFFF */
#define TK_DR_DATA                          TK_DR_DATA_Msk                          

/*******************  Bit definition for TK_SR register  ********************/
#define TK_SR_SENSEDONE_Pos                 (0U)
#define TK_SR_SENSEDONE_Msk                 (0x1UL << TK_SR_SENSEDONE_Pos)          /*!< 0x00000001 */
#define TK_SR_SENSEDONE                     TK_SR_SENSEDONE_Msk                    

#define TK_SR_CMPOUT_Pos                    (1U)
#define TK_SR_CMPOUT_Msk                    (0x1UL << TK_SR_CMPOUT_Pos)             /*!< 0x00000002 */
#define TK_SR_CMPOUT                        TK_SR_CMPOUT_Msk                        

#define TK_SR_SHORTSWERR_Pos                (2U)
#define TK_SR_SHORTSWERR_Msk                (0x1UL << TK_SR_SHORTSWERR_Pos)         /*!< 0x00000004 */
#define TK_SR_SHORTSWERR                    TK_SR_SHORTSWERR_Msk                        

/*******************  Bit definition for TK_IER register  ********************/
#define TK_IER_EVTINTEN_Pos                 (0U)
#define TK_IER_EVTINTEN_Msk                 (0x1UL << TK_IER_EVTINTEN_Pos)          /*!< 0x00000001 */
#define TK_IER_EVTINTEN                     TK_IER_EVTINTEN_Msk                        

#define TK_IER_ERRINTEN_Pos                 (1U)
#define TK_IER_ERRINTEN_Msk                 (0x1UL << TK_IER_ERRINTEN_Pos)          /*!< 0x00000002 */
#define TK_IER_ERRINTEN                     TK_IER_ERRINTEN_Msk         

/*******************  Bit definition for TK_ICR register  ********************/
#define TK_ICR_EVTINTCLR_Pos                (0U)
#define TK_ICR_EVTINTCLR_Msk                (0x1UL << TK_ICR_EVTINTCLR_Pos)         /*!< 0x00000001 */
#define TK_ICR_EVTINTCLR                    TK_ICR_EVTINTCLR_Msk                       

#define TK_ICR_ERRINTCLR_Pos                (1U)
#define TK_ICR_ERRINTCLR_Msk                (0x1UL << TK_ICR_ERRINTCLR_Pos)         /*!< 0x00000002 */
#define TK_ICR_ERRINTCLR                    TK_ICR_ERRINTCLR_Msk

/*******************  Bit definition for TK_CDR register  ********************/
#define TK_CDR_CDATA_Pos                    (0U)
#define TK_CDR_CDATA_Msk                    (0xFFFFUL << TK_CDR_CDATA_Pos)          /*!< 0x0000FFFF */
#define TK_CDR_CDATA                        TK_CDR_CDATA_Msk                        

/*******************  Bit definition for TK_LPDR register  ********************/
#define TK_LPDR_NORMALDELTA_Pos             (0U)
#define TK_LPDR_NORMALDELTA_Msk             (0x3FFUL << TK_LPDR_NORMALDELTA_Pos)    /*!< 0x000003FF */
#define TK_LPDR_NORMALDELTA                 TK_LPDR_NORMALDELTA_Msk                 

#define TK_LPDR_ABNORMALDELTA_Pos           (16U)
#define TK_LPDR_ABNORMALDELTA_Msk           (0x3FFUL << TK_LPDR_ABNORMALDELTA_Pos)  /*!< 0x03FF0000 */
#define TK_LPDR_ABNORMALDELTA               TK_LPDR_ABNORMALDELTA_Msk               

/*******************  Bit definition for TK_LPTR register  ********************/
#define TK_LPTR_LPWAIT_Pos                  (0U)
#define TK_LPTR_LPWAIT_Msk                  (0x7FUL << TK_LPTR_LPWAIT_Pos)          /*!< 0x0000007F */
#define TK_LPTR_LPWAIT                      TK_LPTR_LPWAIT_Msk                      

/*******************  Bit definition for TK_OPR register  ********************/
#define TK_OPR_OP_Pos                       (0U)
#define TK_OPR_OP_Msk                       (0x3FUL << TK_OPR_OP_Pos)               /*!< 0x0000003F */
#define TK_OPR_OP                           TK_OPR_OP_Msk                   

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (UART)       */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for UART_DR register  *******************/
#define UART_DR_DR_Pos                     (0U)
#define UART_DR_DR_Msk                     (0x1FFUL << UART_DR_DR_Pos)           /*!< 0x000001FF */
#define UART_DR_DR                         UART_DR_DR_Msk                        /*!< Data value */
#define UART_DR_DR_0                       (0x1UL<<UART_DR_DR_Pos)               /*!< 0x00000001 */
#define UART_DR_DR_1                       (0x2UL<<UART_DR_DR_Pos)               /*!< 0x00000002 */
#define UART_DR_DR_2                       (0x4UL<<UART_DR_DR_Pos)               /*!< 0x00000004 */
#define UART_DR_DR_3                       (0x8UL<<UART_DR_DR_Pos)               /*!< 0x00000008 */
#define UART_DR_DR_4                       (0x10UL<<UART_DR_DR_Pos)              /*!< 0x00000010 */
#define UART_DR_DR_5                       (0x20UL<<UART_DR_DR_Pos)              /*!< 0x00000020 */
#define UART_DR_DR_6                       (0x40UL<<UART_DR_DR_Pos)              /*!< 0x00000040 */
#define UART_DR_DR_7                       (0x80UL<<UART_DR_DR_Pos)              /*!< 0x00000080 */
#define UART_DR_DR_8                       (0x100UL<<UART_DR_DR_Pos)             /*!< 0x00000100 */

/******************  Bit definition for UART_BRR register  *******************/
#define UART_BRR_BRR_Pos                   (0U)
#define UART_BRR_BRR_Msk                   (0xFFFFUL << UART_BRR_BRR_Pos)        /*!< 0x0000FFFF */
#define UART_BRR_BRR                       UART_BRR_BRR_Msk                      /*!< UART BRR */
#define UART_BRR_BRR_0                     (0x1UL<<UART_BRR_BRR_Pos)             /*!< 0x00000001 */
#define UART_BRR_BRR_1                     (0x2UL<<UART_BRR_BRR_Pos)             /*!< 0x00000002 */
#define UART_BRR_BRR_2                     (0x4UL<<UART_BRR_BRR_Pos)             /*!< 0x00000004 */
#define UART_BRR_BRR_3                     (0x8UL<<UART_BRR_BRR_Pos)             /*!< 0x00000008 */
#define UART_BRR_BRR_4                     (0x10UL<<UART_BRR_BRR_Pos)            /*!< 0x00000010 */
#define UART_BRR_BRR_5                     (0x20UL<<UART_BRR_BRR_Pos)            /*!< 0x00000020 */
#define UART_BRR_BRR_6                     (0x40UL<<UART_BRR_BRR_Pos)            /*!< 0x00000040 */
#define UART_BRR_BRR_7                     (0x80UL<<UART_BRR_BRR_Pos)            /*!< 0x00000080 */
#define UART_BRR_BRR_8                     (0x100UL<<UART_BRR_BRR_Pos)           /*!< 0x00000100 */
#define UART_BRR_BRR_9                     (0x200UL<<UART_BRR_BRR_Pos)           /*!< 0x00000200 */
#define UART_BRR_BRR_10                    (0x400UL<<UART_BRR_BRR_Pos)           /*!< 0x00000400 */
#define UART_BRR_BRR_11                    (0x800UL<<UART_BRR_BRR_Pos)           /*!< 0x00000800 */
#define UART_BRR_BRR_12                    (0x1000UL<<UART_BRR_BRR_Pos)          /*!< 0x00001000 */
#define UART_BRR_BRR_13                    (0x2000UL<<UART_BRR_BRR_Pos)          /*!< 0x00002000 */
#define UART_BRR_BRR_14                    (0x4000UL<<UART_BRR_BRR_Pos)          /*!< 0x00004000 */
#define UART_BRR_BRR_15                    (0x8000UL<<UART_BRR_BRR_Pos)          /*!< 0x00008000 */

/*******************  Bit definition for UART_SR register  *******************/
#define UART_SR_RXNE_Pos                   (0U)
#define UART_SR_RXNE_Msk                   (0x1UL << UART_SR_RXNE_Pos)           /*!< 0x00000001 */
#define UART_SR_RXNE                       UART_SR_RXNE_Msk                      /*!< Read Data Register Not Empty */

#define UART_SR_ORE_Pos                    (1U)
#define UART_SR_ORE_Msk                    (0x1UL << UART_SR_ORE_Pos)            /*!< 0x00000002 */
#define UART_SR_ORE                        UART_SR_ORE_Msk                       /*!< OverRun Error */

#define UART_SR_PE_Pos                     (2U)
#define UART_SR_PE_Msk                     (0x1UL << UART_SR_PE_Pos)             /*!< 0x00000004 */
#define UART_SR_PE                         UART_SR_PE_Msk                        /*!< Parity Error */

#define UART_SR_FE_Pos                     (3U)
#define UART_SR_FE_Msk                     (0x1UL << UART_SR_FE_Pos)             /*!< 0x00000008 */
#define UART_SR_FE                         UART_SR_FE_Msk                        /*!< Framing Error */

#define UART_SR_BRI_Pos                    (4U)
#define UART_SR_BRI_Msk                    (0x1UL << UART_SR_BRI_Pos)            /*!< 0x00000010 */
#define UART_SR_BRI                        UART_SR_BRI_Msk                       

#define UART_SR_TDRE_Pos                   (5U)
#define UART_SR_TDRE_Msk                   (0x1UL << UART_SR_TDRE_Pos)           /*!< 0x00000020 */
#define UART_SR_TDRE                       UART_SR_TDRE_Msk                      

#define UART_SR_TXE_Pos                    (6U)
#define UART_SR_TXE_Msk                    (0x1UL << UART_SR_TXE_Pos)            /*!< 0x00000040 */
#define UART_SR_TXE                        UART_SR_TXE_Msk                       /*!< Transmit Data Register Empty */

#define UART_SR_ADDR_RCVD_Pos              (8U)
#define UART_SR_ADDR_RCVD_Msk              (0x1UL << UART_SR_ADDR_RCVD_Pos)      /*!< 0x00000100 */
#define UART_SR_ADDR_RCVD                  UART_SR_ADDR_RCVD_Msk                 

#define UART_SR_BUSY_Pos                   (9U)
#define UART_SR_BUSY_Msk                   (0x1UL << UART_SR_BUSY_Pos)           /*!< 0x00000200 */
#define UART_SR_BUSY                       UART_SR_BUSY_Msk                      

#define UART_SR_BUSY_ERR_Pos               (10U)
#define UART_SR_BUSY_ERR_Msk               (0x1UL << UART_SR_BUSY_ERR_Pos)       /*!< 0x00000400 */
#define UART_SR_BUSY_ERR                   UART_SR_BUSY_ERR_Msk                

/******************  Bit definition for UART_CR1 register  *******************/
#define UART_CR1_M_Pos                     (0U)
#define UART_CR1_M_Msk                     (0x3UL << UART_CR1_M_Pos)             /*!< 0x00000003 */
#define UART_CR1_M                         UART_CR1_M_Msk                        
#define UART_CR1_M_0                       (0x1UL << UART_CR1_M_Pos)             /*!< 0x00000001 */
#define UART_CR1_M_1                       (0x2UL << UART_CR1_M_Pos)             /*!< 0x00000002 */

#define UART_CR1_STOP_Pos                  (2U)
#define UART_CR1_STOP_Msk                  (0x1UL << UART_CR1_STOP_Pos)          /*!< 0x00000004 */
#define UART_CR1_STOP                      UART_CR1_STOP_Msk                     

#define UART_CR1_PCE_Pos                   (3U)
#define UART_CR1_PCE_Msk                   (0x1UL << UART_CR1_PCE_Pos)           /*!< 0x00000008 */
#define UART_CR1_PCE                       UART_CR1_PCE_Msk                      

#define UART_CR1_PS_Pos                    (4U)
#define UART_CR1_PS_Msk                    (0x1UL << UART_CR1_PS_Pos)            /*!< 0x00000010 */
#define UART_CR1_PS                        UART_CR1_PS_Msk                       /*!< Parity Selection */

#define UART_CR1_SP_Pos                    (5U)
#define UART_CR1_SP_Msk                    (0x1UL << UART_CR1_SP_Pos)            /*!< 0x00000020 */
#define UART_CR1_SP                        UART_CR1_SP_Msk                       

#define UART_CR1_SBK_Pos                   (6U)
#define UART_CR1_SBK_Msk                   (0x1UL << UART_CR1_SBK_Pos)           /*!< 0x00000040 */
#define UART_CR1_SBK                       UART_CR1_SBK_Msk                      /*!< Send Break */

#define UART_CR1_SWAP_Pos                  (8U)
#define UART_CR1_SWAP_Msk                  (0x1UL << UART_CR1_SWAP_Pos)          /*!< 0x00000100 */
#define UART_CR1_SWAP                      UART_CR1_SWAP_Msk                     

#define UART_CR1_MSBFIRST_Pos              (9U)
#define UART_CR1_MSBFIRST_Msk              (0x1UL << UART_CR1_MSBFIRST_Pos)      /*!< 0x00000200 */
#define UART_CR1_MSBFIRST                  UART_CR1_MSBFIRST_Msk                 /*!< MSB First */

/******************  Bit definition for UART_CR2 register  *******************/
#define UART_CR2_RXNEIE_Pos                (0U)
#define UART_CR2_RXNEIE_Msk                (0x1UL << UART_CR2_RXNEIE_Pos)        /*!< 0x00000001 */
#define UART_CR2_RXNEIE                    UART_CR2_RXNEIE_Msk                   /*!< RXNE Interrupt Enable */

#define UART_CR2_TDREIE_Pos                (1U)
#define UART_CR2_TDREIE_Msk                (0x1UL << UART_CR2_TDREIE_Pos)        /*!< 0x00000002 */
#define UART_CR2_TDREIE                    UART_CR2_TDREIE_Msk                   /*!< TDRE Interrupt Enable */

#define UART_CR2_LSIE_Pos                  (2U)
#define UART_CR2_LSIE_Msk                  (0x1UL << UART_CR2_LSIE_Pos)          /*!< 0x00000004 */
#define UART_CR2_LSIE                      UART_CR2_LSIE_Msk                     /*!< LS Interrupt Enable */ 

#define UART_CR2_BUSYERRIE_Pos             (3U)
#define UART_CR2_BUSYERRIE_Msk             (0x1UL << UART_CR2_BUSYERRIE_Pos)     /*!< 0x00000008 */
#define UART_CR2_BUSYERRIE                 UART_CR2_BUSYERRIE_Msk                /*!< BUSYERR Interrupt Enable */

/******************  Bit definition for UART_CR3 register  *******************/
#define UART_CR3_M_E_Pos                   (0U)
#define UART_CR3_M_E_Msk                   (0x1UL << UART_CR3_M_E_Pos)           /*!< 0x00000001 */
#define UART_CR3_M_E                       UART_CR3_M_E_Msk                      

#define UART_CR3_ADDR_MATCH_Pos            (1U)
#define UART_CR3_ADDR_MATCH_Msk            (0x1UL << UART_CR3_ADDR_MATCH_Pos)    /*!< 0x00000002 */
#define UART_CR3_ADDR_MATCH                UART_CR3_ADDR_MATCH_Msk               

#define UART_CR3_SEND_ADDR_Pos             (2U)
#define UART_CR3_SEND_ADDR_Msk             (0x1UL << UART_CR3_SEND_ADDR_Pos)     /*!< 0x00000004 */
#define UART_CR3_SEND_ADDR                 UART_CR3_SEND_ADDR_Msk                

#define UART_CR3_TX_MODE_Pos               (3U)
#define UART_CR3_TX_MODE_Msk               (0x1UL << UART_CR3_TX_MODE_Pos)       /*!< 0x00000008 */
#define UART_CR3_TX_MODE                   UART_CR3_TX_MODE_Msk                  

/******************  Bit definition for UART_RAR register  *******************/
#define UART_RAR_RAR_Pos                   (0U)
#define UART_RAR_RAR_Msk                   (0xFFUL << UART_RAR_RAR_Pos)          /*!< 0x000000FF */
#define UART_RAR_RAR                       UART_RAR_RAR_Msk                     
#define UART_RAR_RAR_0                     (0x1UL<<UART_RAR_RAR_Pos)             /*!< 0x00000001 */
#define UART_RAR_RAR_1                     (0x2UL<<UART_RAR_RAR_Pos)             /*!< 0x00000002 */
#define UART_RAR_RAR_2                     (0x4UL<<UART_RAR_RAR_Pos)             /*!< 0x00000004 */
#define UART_RAR_RAR_3                     (0x8UL<<UART_RAR_RAR_Pos)             /*!< 0x00000008 */
#define UART_RAR_RAR_4                     (0x10UL<<UART_RAR_RAR_Pos)            /*!< 0x00000010 */
#define UART_RAR_RAR_5                     (0x20UL<<UART_RAR_RAR_Pos)            /*!< 0x00000020 */
#define UART_RAR_RAR_6                     (0x40UL<<UART_RAR_RAR_Pos)            /*!< 0x00000040 */
#define UART_RAR_RAR_7                     (0x80UL<<UART_RAR_RAR_Pos)            /*!< 0x00000080 */

/******************  Bit definition for UART_TAR register  *******************/
#define UART_TAR_TAR_Pos                   (0U)
#define UART_TAR_TAR_Msk                   (0xFFUL << UART_TAR_TAR_Pos)          /*!< 0x000000FF */
#define UART_TAR_TAR                       UART_TAR_TAR_Msk                      
#define UART_TAR_TAR_0                     (0x1UL<<UART_TAR_TAR_Pos)             /*!< 0x00000001 */
#define UART_TAR_TAR_1                     (0x2UL<<UART_TAR_TAR_Pos)             /*!< 0x00000002 */
#define UART_TAR_TAR_2                     (0x4UL<<UART_TAR_TAR_Pos)             /*!< 0x00000004 */
#define UART_TAR_TAR_3                     (0x8UL<<UART_TAR_TAR_Pos)             /*!< 0x00000008 */
#define UART_TAR_TAR_4                     (0x10UL<<UART_TAR_TAR_Pos)            /*!< 0x00000010 */
#define UART_TAR_TAR_5                     (0x20UL<<UART_TAR_TAR_Pos)            /*!< 0x00000020 */
#define UART_TAR_TAR_6                     (0x40UL<<UART_TAR_TAR_Pos)            /*!< 0x00000040 */
#define UART_TAR_TAR_7                     (0x80UL<<UART_TAR_TAR_Pos)            /*!< 0x00000080 */


/** @addtogroup Exported_macros
  * @{
  */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC)

/****************************** COMP Instances ********************************/
#define IS_COMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == COMP1) || \
                                        ((INSTANCE) == COMP2))

/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOF))

/********************** GPIO Alternate Function Instances *********************/
#define IS_GPIO_AF_INSTANCE(INSTANCE)   IS_GPIO_ALL_INSTANCE(INSTANCE)

/**************************** GPIO Lock Instances *****************************/
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) ((INSTANCE) == I2C)

/************************ I2C WAKEUP FROMSTOP Instances ***********************/
#define IS_I2C_WAKEUP_FROMSTOP_INSTANCE(INSTANCE) (((INSTANCE) == I2C))

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** RTC Instances ********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SPI1)

/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE)       (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM14))

/****************** TIM Instances : supporting the break function *************/
#define IS_TIM_BREAK_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/************** TIM Instances : supporting Break source selection *************/
#define IS_TIM_BREAKSOURCE_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM14))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/******************* TIM Instances : output(s) available **********************/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM14) &&                 \
     (((CHANNEL) == TIM_CHANNEL_1))))

/****************** TIM Instances : supporting complementary output(s) ********/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3))))           

/****************** TIM Instances : supporting clock division *****************/
#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1))   
                                                 
/****** TIM Instances : supporting external clock mode 1 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/****** TIM Instances : supporting external clock mode 2 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting external clock mode 1 for TIX inputs*/
#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)      ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting internal trigger inputs(ITRX) *******/
#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)     ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting commutation event generation ***/
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE) (((INSTANCE) == TIM1))

/****************** TIM Instances : supporting counting mode selection ********/
#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting encoder interface **************/
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)  ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting Hall sensor interface **********/
#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/**************** TIM Instances : external trigger input available ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)      ((INSTANCE) == TIM1)

/************* TIM Instances : supporting ETR source selection ***************/
#define IS_TIM_ETRSEL_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/****** TIM Instances : Master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting OCxREF clear *******************/
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)        ((INSTANCE) == TIM1)

/****************** TIM Instances : remapping capability **********************/
#define IS_TIM_REMAP_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting repetition counter *************/
#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1))

/****************** TIM Instances : supporting synchronization ****************/
#define IS_TIM_SYNCHRO_INSTANCE(INSTANCE)  IS_TIM_MASTER_INSTANCE(INSTANCE)

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)      ((INSTANCE) == TIM1)

/******************* TIM Instances : Timer input selection ********************/
#define IS_TIM_TISEL_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM14))

/************ TIM Instances : Advanced timers  ********************************/
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1))

/******************** UART Instances **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == UART1) || \
                                    ((INSTANCE) == UART2) || \
                                    ((INSTANCE) == UART3))


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
#endif /* __cplusplus */

#endif /* __PY32T020X5_H */

/**
  * @}
  */

/**
* @}
*/

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/


/**
  ******************************************************************************
  * @file    py32t020_hal_flash.h
  * @author  MCU Application Team
  * @brief   Header file of FLASH HAL module.
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
#ifndef __PY32T020_FLASH_H
#define __PY32T020_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32t0xx.h"

/**
  * @brief  FLASH Option Bytes PROGRAM structure definition
  */
typedef struct
{
  uint32_t  OptionType;       /*!< OptionType: Option byte to be configured.
                                   This parameter can be a value of @ref FLASH_Option_Type */

  uint32_t  WRPSector;        /*!< WRPSector: This bitfield specifies the sector (s) which are write protected.
                                   This parameter can be a combination of @ref FLASH_Option_Bytes_Write_Protection */

  uint32_t  SDKStartAddr;     /*!< SDK Start address (used for FLASH_SDKR). It represents first address of start block
                                   to protect. Make sure this parameter is multiple of SDK granularity: 2048 Bytes.*/

  uint32_t  SDKEndAddr;       /*!< SDK End address (used for FLASH_SDKR). It represents first address of end block
                                   to protect. Make sure this parameter is multiple of SDK granularity: 2048 Bytes.*/

  uint32_t  RDPLevel;         /*!< RDPLevel: Set the read protection level.
                                   This parameter can be a value of @ref FLASH_OB_Read_Protection */

  uint32_t  USERType;         /*!< User option byte(s) to be configured (used for OPTIONBYTE_USER).
                                   This parameter can be a combination of @ref FLASH_OB_USER_Type */

  uint32_t  USERConfig;       /*!< Value of the user option byte (used for OPTIONBYTE_USER).
                                   This parameter can be a combination of
                                   @ref FLASH_OB_USER_BOR_ENABLE,
                                   @ref FLASH_OB_USER_BOR_LEVEL,
                                   @ref FLASH_OB_USER_IWDG_SW,
                                   @ref FLASH_OB_USER_IWDG_STOP,
                                   @ref FLASH_OB_USER_NRST,
                                   @ref FLASH_OB_USER_SWD 
                                   */
} FLASH_OBProgramInitTypeDef;
/** @defgroup FLASH_OB_USER_Type FLASH User Option Type
  * @{
  */
#define OB_USER_BOR_EN          FLASH_OPTR_BOR_EN
#define OB_USER_BOR_LEV         FLASH_OPTR_BOR_LEV
#define OB_USER_IWDG_SW         FLASH_OPTR_IWDG_SW
#define OB_USER_NRST_MODE       FLASH_OPTR_NRST_MODE
#define OB_USER_SWD_MODE        FLASH_BTCR_SWD_MODE << 16 
#define OB_USER_IWDG_STOP       FLASH_OPTR_IWDG_STOP
#define OB_USER_ALL             (OB_USER_BOR_EN  | OB_USER_BOR_LEV   | OB_USER_IWDG_SW | \
                                 OB_USER_NRST_MODE | OB_USER_IWDG_STOP | OB_USER_SWD_MODE) 
/** @defgroup FLASH_Option_Type FLASH Option Type
  * @{
  */
#define OPTIONBYTE_WRP            ((uint32_t)0x01U)  /*!<WRP option byte configuration*/
#define OPTIONBYTE_SDK            ((uint32_t)0x02U)  /*!<SDK option byte configuration*/
#define OPTIONBYTE_RDP            ((uint32_t)0x04U)  /*!<RDP option byte configuration*/
#define OPTIONBYTE_USER           ((uint32_t)0x08U)  /*!<USER option byte configuration*/
#define OPTIONBYTE_ALL            (OPTIONBYTE_WRP  | \
                                   OPTIONBYTE_SDK  | \
                                   OPTIONBYTE_USER | \
                                   OPTIONBYTE_RDP) 
								   
/** @defgroup FLASH_OB_Read_Protection FLASH Option Bytes Read Protection
  * @{
  */
#define OB_RDP_LEVEL_0         	  ((uint8_t)0xAAU) 
#define OB_RDP_LEVEL_1            ((uint8_t)0x55U) 

void FLASH_Unlock(void);
void FLASH_Lock(void);
void FLASH_OB_Unlock(void);
void FLASH_OB_Lock(void);
void FLASH_OB_Launch(void);

void Erase_UserData(uint32_t UserDataAddress);
void Program_UserData(uint32_t Address, uint32_t *DataAddress);
void FLASH_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);

#ifdef __cplusplus
}
#endif

#endif /* __PY32T020_HAL_FLASH_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

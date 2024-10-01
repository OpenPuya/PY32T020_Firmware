/**
 ******************************************************************************
 * @file    py32t020_flash.c
 * @author  MCU Application Team
 * @brief   FLASH HAL module driver.
 *          This file provides firmware functions to manage the following
 *          functionalities of the internal FLASH memory:
 *           + Program operations functions
 *           + Memory Control functions
 *           + Peripheral Errors functions
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

#if defined(USE_FULL_LL_DRIVER)
/* Includes ------------------------------------------------------------------*/
#include "py32t020_flash.h"
#ifdef USE_FULL_ASSERT
#include "py32_assert.h"
#else
#define assert_param(expr) ((void)0U)
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
 * @}
 */
const uint32_t _TimmingParam[8] = 	   {0x1FFF011C, 0x1FFF011C, 0x1FFF011C, 0x1FFF011C,
                                        0x1FFF011C, 0x1FFF0130, 0x1FFF011C, 0x1FFF011C};
#define FLASH_GET_FLAG(__FLAG__)        (READ_BIT(FLASH->SR,   (__FLAG__)) == (__FLAG__))
#define FLASH_FLAG_SR_ERROR             (FLASH_SR_OPTVERR  | FLASH_SR_WRPERR)     /*!< All SR error flags */

#define FLASH_FLAG_SR_CLEAR             (FLASH_FLAG_SR_ERROR | FLASH_SR_EOP)

#define _FLASH_TIME_REG_SET(__EPPARA0__,__EPPARA1__,__EPPARA2__,__EPPARA3__,__EPPARA4__)           \
                                                        do {                                            \
                                                            FLASH->TS0  = (__EPPARA0__)&0x1FF;           \
                                                            FLASH->TS1  = ((__EPPARA0__)>>18)&0x3FF;    \
                                                            FLASH->TS3  = ((__EPPARA0__)>>9)&0x1FF;      \
                                                            FLASH->TS2P = (__EPPARA1__)&0x1FF;           \
                                                            FLASH->TPS3 = ((__EPPARA1__)>>16)&0xFFF;    \
                                                            FLASH->PERTPE = (__EPPARA2__)&0x3FFFF;      \
                                                            FLASH->SMERTPE = (__EPPARA3__)&0x3FFFF;     \
                                                            FLASH->PRGTPE = (__EPPARA4__)&0xFFFF;       \
                                                            FLASH->PRETPE = ((__EPPARA4__)>>16)&0x3FFF; \
                                                         } while(0U)

#define _FLASH_IS_INVALID_TIMMING_SEQUENCE(_INDEX_)  (((FLASH->TS0)     !=  ((*(uint32_t *)(_TimmingParam[_INDEX_]))&0x1FF))          ||  \
                                                           ((FLASH->TS1)     != (((*(uint32_t *)(_TimmingParam[_INDEX_]))>>18)&0x3FF))    ||  \
                                                           ((FLASH->TS3)     != (((*(uint32_t *)(_TimmingParam[_INDEX_]))>>9)&0x1FF))      ||  \
                                                           ((FLASH->TS2P)    !=  ((*(uint32_t *)(_TimmingParam[_INDEX_]+4))&0x1FF))        ||  \
                                                           ((FLASH->TPS3)    != (((*(uint32_t *)(_TimmingParam[_INDEX_]+4))>>16)&0xFFF))  ||  \
                                                           ((FLASH->PERTPE)  !=  ((*(uint32_t *)(_TimmingParam[_INDEX_]+8))&0x3FFFF))     ||  \
                                                           ((FLASH->SMERTPE) !=  ((*(uint32_t *)(_TimmingParam[_INDEX_]+12))&0x3FFFF))    ||  \
                                                           ((FLASH->PRGTPE)  !=  ((*(uint32_t *)(_TimmingParam[_INDEX_]+16))&0xFFFF))     ||  \
                                                           ((FLASH->PRETPE)  != (((*(uint32_t *)(_TimmingParam[_INDEX_]+16))>>16)&0x3FFF)))

#define _FLASH_TIMMING_SEQUENCE_CONFIG() do{                                                                            \
                                                uint32_t tmpreg = (RCC->ICSCR & RCC_ICSCR_HSI_FS) >> RCC_ICSCR_HSI_FS_Pos;   \
                                                if (_FLASH_IS_INVALID_TIMMING_SEQUENCE(tmpreg))                         \
                                                {                                                                            \
                                                  _FLASH_TIME_REG_SET((*(uint32_t *)(_TimmingParam[tmpreg])),      \
                                                                           (*(uint32_t *)(_TimmingParam[tmpreg]+4)),    \
                                                                           (*(uint32_t *)(_TimmingParam[tmpreg]+8)),    \
                                                                           (*(uint32_t *)(_TimmingParam[tmpreg]+12)),   \
                                                                           (*(uint32_t *)(_TimmingParam[tmpreg]+16)));  \
                                                }                                                                            \
                                              }while(0U)
/**
 * @brief  Unlock the FLASH control register access.
 * @retval HAL Status
 */
void FLASH_Unlock(void)
{
    if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0x00U)
    {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);
    }
}
/**
 * @brief  Lock the FLASH control register access.
 * @retval HAL Status
 */
void FLASH_Lock(void)
{
    /* Set the LOCK Bit to lock the FLASH Registers access */
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}
/**
 * @brief  Unlock the FLASH Option Bytes Registers access.
 * @retval HAL Status
 */
void FLASH_OB_Unlock(void)
{
    if (READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) != 0x00U)
    {
        /* Authorizes the Option Byte register programming */
        WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
        WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);
    }
}
/**
 * @brief  Lock the FLASH Option Bytes Registers access.
 * @retval HAL Status
 */
void FLASH_OB_Lock(void)
{
    /* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access */
    SET_BIT(FLASH->CR, FLASH_CR_OPTLOCK);
}
/**
 * @brief  Launch the option byte loading.
 * @retval HAL Status
 */
void FLASH_OB_Launch(void)
{
    /* Set the bit to force the option byte reloading */
    SET_BIT(FLASH->CR, FLASH_CR_OBL_LAUNCH);
}
/**
 * @brief  Wait for a FLASH operation to complete.
 * @param  Timeout maximum flash operation timeout
 * @retval HAL_StatusTypeDef HAL Status
 */
void FLASH_WaitForLastOperation(void)
{
    /* Wait if any operation is ongoing */
    while (FLASH_GET_FLAG(FLASH_SR_BSY) != 0x00U)
    {
        ;
    }
    /* Clear SR register */
    FLASH->SR = FLASH_FLAG_SR_CLEAR;
}
/**
 * @brief  UserData erase
 * @retval None
 */
void Erase_UserData(uint32_t UserDataAddress)
{
	/* Config flash timming */
	_FLASH_TIMMING_SEQUENCE_CONFIG();
	 /* Wait for last operation to be completed */
    FLASH_WaitForLastOperation();
    /* Clean the error context */
    SET_BIT(FLASH->CR, FLASH_CR_PER);
    *(__IO uint32_t *)(UserDataAddress) = 0xFF;
	 /* Wait for last operation to be completed */
    FLASH_WaitForLastOperation();
	/* If the erase operation is completed, disable the PER Bit */
	CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
}

/**
 * @brief  Program the UserData
 * @retval None
 */
void Program_UserData(uint32_t Address, uint32_t *DataAddress)
{
    uint8_t index = 0;
    uint32_t dest = Address;
    uint32_t *src = DataAddress;
    uint32_t primask_bit;
	/* Config flash timming */
	_FLASH_TIMMING_SEQUENCE_CONFIG();
	 /* Wait for last operation to be completed */
    FLASH_WaitForLastOperation();
	
    SET_BIT(FLASH->CR, FLASH_CR_PG);
    /* Enter critical section */
    primask_bit = __get_PRIMASK();
    __disable_irq();
    /* 32 words*/
    while (index < 32U)
    {
        *(uint32_t *)dest = *src;
        src += 1U;
        dest += 4U;
        index++;
        if (index == 31)
        {
            SET_BIT(FLASH->CR, FLASH_CR_PGSTRT);
        }
    }
    /* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);
	 /* Wait for last operation to be completed */
	FLASH_WaitForLastOperation();
	/* If the program operation is completed, disable the UPG Bit */
	CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
}
/**
  * @brief  Set User configuration
  * @param  UserType  The FLASH User Option Bytes to be modified.
  *         This parameter can be a combination of @ref FLASH_OB_USER_Type
  * @param  UserConfig  The FLASH User Option Bytes values.
  *         This parameter can be a combination of:
  *           @arg @ref OB_USER_BOR_EN
  *           @arg @ref OB_USER_BOR_LEV
  *           @arg @ref OB_USER_IWDG_SW
  *           @arg @ref OB_USER_NRST_MODE
  *           @arg @ref OB_USER_IWDG_STOP
  * @param  RDPLevel  specifies the read protection level.
  *         This parameter can be one of the following values:
  *           @arg @ref OB_RDP_LEVEL_0 No protection
  *           @arg @ref OB_RDP_LEVEL_1 Memory Read protection
  * @note  (*) availability depends on devices
  * @retval None
  */
void FLASH_OB_OptrConfig(uint32_t UserType, uint32_t UserConfig, uint32_t RDPLevel)
{
	uint32_t optr;
	/* Check the parameters */
	assert_param(IS_OB_USER_TYPE(UserType));
	assert_param(IS_OB_USER_CONFIG(UserType, UserConfig));
	assert_param(IS_OB_RDP_LEVEL(RDPLevel));
	/* Configure the RDP level in the option bytes register */
	optr = FLASH->OPTR;
	optr &= ~(UserType | FLASH_OPTR_RDP);
	FLASH->OPTR = (optr | UserConfig | RDPLevel);
}

///**
//  * @brief  Return the FLASH User Option Byte value.
//  * @retval The FLASH User Option Bytes values. It will be a combination of all the following values:
//  *           @arg @ref OB_USER_BOR_EN
//  *           @arg @ref OB_USER_BOR_LEV
//  *           @arg @ref OB_USER_IWDG_SW
//  *           @arg @ref OB_USER_NRST_MODE
//  *           @arg @ref OB_USER_SWD_MODE
//  *           @arg @ref OB_USER_IWDG_STOP
//  */
//uint32_t FLASH_OB_GetUser(void)
//{
//	uint32_t user = ((FLASH->OPTR & ~FLASH_OPTR_RDP) & OB_USER_ALL);
//	user |= (FLASH->BTCR&0x300)<<16; 
//	return user;
//}


/**
  * @brief  Return the FLASH Read Protection level.
  * @retval FLASH ReadOut Protection Status:
  *         This return value can be one of the following values:
  *           @arg @ref OB_RDP_LEVEL_0 No protection
  *           @arg @ref OB_RDP_LEVEL_1 Read protection of the memory
  */
uint32_t FLASH_OB_GetRDP(void)
{
	uint32_t rdplvl = READ_BIT(FLASH->OPTR, FLASH_OPTR_RDP);
	if (rdplvl == OB_RDP_LEVEL_0)
	{
		return (OB_RDP_LEVEL_0);
	}
	else
	{
		return rdplvl;
	}
}
/**
  * @brief  Program option bytes
  * @note   The function @ref HAL_FLASH_Unlock() should be called before to unlock the FLASH interface
  *         The function @ref HAL_FLASH_OB_Unlock() should be called before to unlock the options bytes
  *         The function @ref HAL_FLASH_OB_Launch() should be called after to force the reload of the options bytes
  *         (system reset will occur)
  *
  * @param  pOBInit pointer to an FLASH_OBInitStruct structure that
  *         contains the configuration information for the programming.
  *
  * @retval HAL_StatusTypeDef HAL Status
  */
void FLASH_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit)
{
	uint32_t primask_bit;
	uint32_t optr;
	/* Config flash timming */
	_FLASH_TIMMING_SEQUENCE_CONFIG();
	/* Enter critical section */
    primask_bit = __get_PRIMASK();
    __disable_irq();
	/* Check the parameters */
//	assert_param(IS_OPTIONBYTE(pOBInit->OptionType));
//	/* WRP register */
//	if ((pOBInit->OptionType & OPTIONBYTE_WRP) != 0x00U)
//	{
//		/* Write protection configuration */
//		FLASH->WRPR = (uint16_t)(~(pOBInit->WRPSector));
//	}
//	/* SDK register */
//	if ((pOBInit->OptionType & OPTIONBYTE_SDK) != 0x00U)
//	{
//		/* SDK protection configuration */
//		FLASH->SDKR = (pOBInit->SDKStartAddr) | (pOBInit->SDKEndAddr<<8);
//	}
//	/* BTCR register */
//	if (((pOBInit->OptionType & OPTIONBYTE_USER) == OPTIONBYTE_USER) && ((pOBInit->USERType & OB_USER_SWD_MODE) == OB_USER_SWD_MODE))
//	{
//		/* BTCR SWD mode configuration */
//		MODIFY_REG(FLASH->BTCR,FLASH_BTCR_SWD_MODE,pOBInit->USERConfig>>16);
//	}
	/* Option register */
//	if ((pOBInit->OptionType & (OPTIONBYTE_RDP | OPTIONBYTE_USER)) == (OPTIONBYTE_RDP | OPTIONBYTE_USER))
//	{
//		/* Fully modify OPTR register with RDP & user data */
//		FLASH_OB_OptrConfig(pOBInit->USERType & 0xFFFF, pOBInit->USERConfig & 0xFFFF, pOBInit->RDPLevel);
//	}
//	else if((pOBInit->OptionType & OPTIONBYTE_RDP) != 0x00U)
//	{
//		/* Only modify RDP so get current user data */
//		optr = FLASH_OB_GetUser();
//		FLASH_OB_OptrConfig(optr & 0xFFFF, optr & 0xFFFF, pOBInit->RDPLevel);
//	}
//	else if ((pOBInit->OptionType & OPTIONBYTE_USER) != 0x00U)
	if ((pOBInit->OptionType & OPTIONBYTE_USER) != 0x00U)
	{
		/* Only modify user so get current RDP level */
		optr = FLASH_OB_GetRDP();
		FLASH_OB_OptrConfig(pOBInit->USERType & 0xFFFF, pOBInit->USERConfig & 0xFFFF, optr);
	}
	else
	{
		/* nothing to do */
	}
	/* starts to modify Flash Option bytes */
	FLASH->CR |= FLASH_CR_OPTSTRT;
	/* set bit EOPIE */
	FLASH->CR |= FLASH_CR_EOPIE;
	/* trigger program */
	*((__IO uint32_t *)(0x40022080))=0xff;
	/* Wait for last operation to be completed */
	FLASH_WaitForLastOperation();
	/* Exit critical section: restore previous priority mask */
    __set_PRIMASK(primask_bit);
}

#endif
/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 - 2019 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        2021-7-1
 * $Revision:    V1.0.0
 *
 * Project:      Flash Programming Functions for Puya PY32F030xx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "FlashOS.h"        // FlashOS Structures
#include "py32t0xx.h"

#define HSI_24M_TRIM_ADDR     (0x1FFF1E10UL)
#define FLASH_24M_TRIM_ADDR   (0x1FFF1E98UL)

void InitRccAndFlashParam(void)
{
  //SET SYSCLOCK 24MHz
  RCC->CR |= RCC_CR_HSION;
  RCC->ICSCR = M32(HSI_24M_TRIM_ADDR);
  while (RCC_CR_HSIRDY != READ_BIT(RCC->CR, RCC_CR_HSIRDY));

  //SET TRIM
  FLASH->TS0     = M16(FLASH_24M_TRIM_ADDR + 0x00);
  FLASH->TS1     = M16(FLASH_24M_TRIM_ADDR + 0x02);
  FLASH->TS3     = M32(FLASH_24M_TRIM_ADDR + 0x04);
  FLASH->TS2P    = M16(FLASH_24M_TRIM_ADDR + 0x08);
  FLASH->TPS3    = M16(FLASH_24M_TRIM_ADDR + 0x0A);
  FLASH->PERTPE  = M32(FLASH_24M_TRIM_ADDR + 0x0C);
  FLASH->SMERTPE = M32(FLASH_24M_TRIM_ADDR + 0x10);
  FLASH->PRGTPE  = M32(FLASH_24M_TRIM_ADDR + 0x14);
  FLASH->PRETPE  = M32(FLASH_24M_TRIM_ADDR + 0x18);
}

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#if defined(FLASH_MEM) || defined(FLASH_OTP)
int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{
  FLASH->KEYR = FLASH_KEY1;                              // Unlock Flash
  FLASH->KEYR = FLASH_KEY2;

  FLASH->SR |= FLASH_SR_EOP;                             // Clear EOP flag

  if ((FLASH->OPTR & FLASH_OPTR_WWDG_SW) == 0)           // Test if WWDG is running (IWDG in HW mode)
  {
    // Set WWDG time out to maximum
    WWDG->CFR = WWDG_CFR_WDGTB;                          // Set prescaler to maximum
  }

  if ((FLASH->OPTR & FLASH_OPTR_IWDG_SW) == 0)           // Test if IWDG is running (IWDG in HW mode)
  {
    // Set IWDG time out to maximum
    IWDG->KR  = 0x5555;                                  // Enable write access to IWDG_PR and IWDG_RLR
    IWDG->PR  = 0x06;                                    // Set prescaler to maximum
    IWDG->RLR = 0xFFF;                                   // Set reload value to maximum
  }

  InitRccAndFlashParam();

  return (0);
}
#endif

#ifdef FLASH_OB
int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{
  FLASH->KEYR = FLASH_KEY1;                              // Unlock Flash
  FLASH->KEYR = FLASH_KEY2;

  FLASH->OPTKEYR = FLASH_OPTKEY1;                        // Unlock Option Bytes
  FLASH->OPTKEYR = FLASH_OPTKEY2;

  FLASH->SR |= FLASH_SR_EOP;                             // Clear EOP flag

  if ((FLASH->OPTR & FLASH_OPTR_WWDG_SW) == 0)           // Test if WWDG is running (IWDG in HW mode)
  {
    // Set WWDG time out to maximum
    WWDG->CFR = WWDG_CFR_WDGTB;                          // Set prescaler to maximum
  }

  if ((FLASH->OPTR & FLASH_OPTR_IWDG_SW) == 0)           // Test if IWDG is running (IWDG in HW mode)
  {
    // Set IWDG time out to maximum
    IWDG->KR  = 0x5555;                                  // Enable write access to IWDG_PR and IWDG_RLR
    IWDG->PR  = 0x06;                                    // Set prescaler to maximum
    IWDG->RLR = 0xFFF;                                   // Set reload value to maximum
  }

  InitRccAndFlashParam();

  return (0);
}
#endif

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#if defined(FLASH_MEM) || defined(FLASH_OTP)
int UnInit(unsigned long fnc)
{

  FLASH->CR |=  FLASH_CR_LOCK;                          // Lock Flash

  return (0);
}
#endif

#ifdef FLASH_OB
int UnInit(unsigned long fnc)
{

  FLASH->CR |= FLASH_CR_LOCK;                          // Lock Flash
  FLASH->CR |= FLASH_CR_OPTLOCK;                        // Lock Option Bytes

  return (0);
}
#endif

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
#ifdef FLASH_MEM
int EraseSector(unsigned long adr)
{
  FLASH->SR |= FLASH_SR_EOP;                                 // Clear EOP flag

  FLASH->CR |= FLASH_CR_SER;                                 // Sector Erase Enabled

  FLASH->CR |= FLASH_CR_EOPIE;

  M32(adr) = 0xFFFFFFFF;                                     // Sector Address

  while (FLASH->SR & (FLASH_SR_BSY0 | FLASH_SR_BSY1))
  {
    WWDG->CR = WWDG_CR_T;
    IWDG->KR = 0xAAAAU;
  }

  FLASH->CR &= ~FLASH_CR_SER;                                // Sector Erase Disabled

  FLASH->CR &= ~FLASH_CR_EOPIE;

  if (READ_BIT(FLASH->SR, FLASH_SR_EOP) == FLASH_SR_EOP)     // Check for FLASH_SR_EOP
  {
    return (0);                                              // Done
  }
  else
  {
    return (1);                                              // Failed
  }
}
#endif

#ifdef FLASH_OTP
int EraseSector(unsigned long adr)
{
  FLASH->SR |= FLASH_SR_EOP;
  FLASH->CR |=  FLASH_CR_UPER;
  FLASH->CR |=  FLASH_CR_EOPIE;
  M32(adr) = 0xFF;
  __asm("DSB");

  while (FLASH->SR & (FLASH_SR_BSY0 | FLASH_SR_BSY1))
  {
    WWDG->CR = WWDG_CR_T;
    IWDG->KR = 0xAAAAU;
  }

  FLASH->CR &= ~FLASH_CR_UPER;
  FLASH->CR &= ~FLASH_CR_EOPIE;

  if (READ_BIT(FLASH->SR, FLASH_SR_EOP) == FLASH_SR_EOP)     // Check for FLASH_SR_EOP
  {
    return (0);                                              // Done
  }
  else
  {
    return (1);                                              // Failed
  }
}
#endif //FLASH_OTP

#ifdef FLASH_OB
int EraseSector(unsigned long adr)
{
  return (0);
}
#endif //FLASH_OB

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
#ifdef FLASH_MEM
int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  sz = (sz + (FLASH_PAGE_SIZE - 1)) & ~(FLASH_PAGE_SIZE - 1);   // Adjust size for 32 Words

  SET_BIT(FLASH->SR, FLASH_SR_EOP);                             // Clear EOP flag

  while (sz)
  {
    FLASH->CR  |=  FLASH_CR_PG;                                 // Programming Enabled

    for (uint8_t i = 0; i < FLASH_PAGE_SIZE / 4; i++)
    {

      M32(adr + i * 4) = *((uint32_t *)(buf + i * 4));          // Program the first word of the Double Word

      if (i == (FLASH_PAGE_SIZE / 4 - 2))
      {
        FLASH->CR  |= FLASH_CR_PGSTRT;
      }
    }

    while (FLASH->SR & (FLASH_SR_BSY0 | FLASH_SR_BSY1))
    {
      WWDG->CR = WWDG_CR_T;
      IWDG->KR = 0x0000AAAAU;
    }

    FLASH->CR  &= ~FLASH_CR_PG;                               // Programming Disabled

    //为了能够Program SRAM，屏蔽下面的EOP检查
//      if (FLASH_EOP != (FLASH->SR & FLASH_EOP)) {           // Check for FLASH_SR_EOP
//          FLASH->SR |= FLASH_EOP;
//          return (1);                                       // Failed
//      }

    adr += FLASH_PAGE_SIZE;                                   // Go to next Page
    buf += FLASH_PAGE_SIZE;
    sz  -= FLASH_PAGE_SIZE;
  }

  return (0);                                                 // Done
}
#endif // FLASH_MEM

#ifdef FLASH_OTP
int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  sz = (sz + (FLASH_PAGE_SIZE - 1)) & ~(FLASH_PAGE_SIZE - 1);   // Adjust size for 32 Words

  SET_BIT(FLASH->SR, FLASH_SR_EOP);                             // Clear EOP flag

  while (sz)
  {
    FLASH->CR  |=  FLASH_CR_UPG;                                 // Programming Enabled

    for (uint8_t i = 0; i < FLASH_PAGE_SIZE / 4; i++)
    {

      M32(adr + i * 4) = *((uint32_t *)(buf + i * 4));          // Program the first word of the Double Word

      if (i == (FLASH_PAGE_SIZE / 4 - 2))
      {
        FLASH->CR  |= FLASH_CR_PGSTRT;
      }
    }

    while (FLASH->SR & (FLASH_SR_BSY0 | FLASH_SR_BSY1))
    {
      WWDG->CR = WWDG_CR_T;
      IWDG->KR = 0x0000AAAAU;
    }

    FLASH->CR  &= ~FLASH_CR_UPG;                               // Programming Disabled

    //为了能够Program SRAM，屏蔽下面的EOP检查
//      if (FLASH_EOP != (FLASH->SR & FLASH_EOP)) {           // Check for FLASH_SR_EOP
//          FLASH->SR |= FLASH_EOP;
//          return (1);                                       // Failed
//      }

    adr += FLASH_PAGE_SIZE;                                   // Go to next Page
    buf += FLASH_PAGE_SIZE;
    sz  -= FLASH_PAGE_SIZE;
  }

  return (0);                                                 // Done
}
#endif // FLASH_OTP

#ifdef FLASH_OB
int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  uint32_t optr;
  uint32_t wrpr;
  uint32_t pcropr0;
  uint32_t pcropr1;

  optr    = M16(buf + 0x00) | (M16(buf + 0x04) << 16);
  wrpr    = M16(buf + 0x10) | (M16(buf + 0x14) << 16);
  pcropr0 = M16(buf + 0x18) | (M16(buf + 0x1C) << 16);
  pcropr1 = M32(buf + 0x20) | (M16(buf + 0x24) << 16);

  FLASH->SR |= FLASH_SR_EOP;                            // Reset FLASH_EOP

  FLASH->OPTR    = optr    & 0x03FFFFFF;
  FLASH->WRPR    = wrpr    & 0xFFFFFFFF;
  FLASH->PCROPR0 = pcropr0 & 0x01FF01FF;
  FLASH->PCROPR1 = pcropr1 & 0x01FF01FF;

  FLASH->CR |= FLASH_CR_OPTSTRT;
  FLASH->CR |= FLASH_CR_EOPIE;

  M32(0x1FFF1D00) = 0xFFFFFFFF;

  while (FLASH->SR & (FLASH_SR_BSY0 | FLASH_SR_BSY1))
  {
    WWDG->CR = WWDG_CR_T;
    IWDG->KR = 0xAAAAU;
  }

  FLASH->CR  &= ~FLASH_CR_OPTSTRT;                      // Programming Disabled
  FLASH->CR  &= ~FLASH_CR_EOPIE;                        // Reset FLASH_EOPIE

  return (0);                                           // Done
}
#endif //  FLASH_OB

#ifdef FLASH_OB
unsigned long Verify(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  unsigned long baseaddr = 0x1FFF1D00;
  unsigned long offset = 0;

  if (adr != baseaddr || sz < 0x24)
  {
    return (adr);
  }

  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  offset = 0x4;
  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  offset = 0x10;
  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  offset = 0x14;
  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  offset = 0x18;
  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  offset = 0x1C;
  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  offset = 0x20;
  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  offset = 0x24;
  if (M16(buf + offset) != M16(baseaddr + offset))
  {
    return (baseaddr + offset);
  }

  return (adr + sz);
}
#endif //  FLASH_OB


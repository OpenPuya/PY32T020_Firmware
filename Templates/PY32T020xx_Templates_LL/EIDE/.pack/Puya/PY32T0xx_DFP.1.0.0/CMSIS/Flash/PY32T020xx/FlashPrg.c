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
 * Project:      Flash Programming Functions for Puya PY32F002Bxx Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "FlashOS.h"        // FlashOS Structures
#include "py32t0xx.h"

uint32_t RCC_ICSCR_RESTORE;
uint32_t FLASH_ACR_RESTORE;
void InitRccAndFlashParam(void);
void UnInitRccAndFlashParam(void);
int ErasePage(unsigned long adr);

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{
  FLASH->KEYR = FLASH_KEY1;                             // Unlock Flash
  FLASH->KEYR = FLASH_KEY2;

  InitRccAndFlashParam();

#ifdef FLASH_OB
  FLASH->OPTKEYR = FLASH_OPTKEY1;                       // Unlock Option Bytes
  FLASH->OPTKEYR = FLASH_OPTKEY2;
#endif

  FLASH->ACR  = 0x00000000;                             // Zero Wait State, no Prefetch
  FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

  if ((FLASH->OPTR & 0x1000) == 0x00)                   // Test if IWDG is running (IWDG in HW mode)
  {
    // Set IWDG time out to ~32.768 second
    IWDG->KR  = 0x5555;                                 // Enable write access to IWDG_PR and IWDG_RLR
    IWDG->PR  = 0x06;                                   // Set prescaler to 256
    IWDG->RLR = 0xFFF;                                  // Set reload value to 4095
  }

  return (0);
}



/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit(unsigned long fnc)
{

  UnInitRccAndFlashParam();

  FLASH->CR    |=  FLASH_CR_LOCK;                          // Lock Flash
#ifdef FLASH_OB
  FLASH->CR    |=  FLASH_CR_OPTLOCK;                       // Lock Option Bytes
#endif
  return (0);
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int EraseChip(void)
{
#ifdef FLASH_OTP
  ErasePage(OTP_BASE);
#else
  FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

  FLASH->CR |=  FLASH_CR_MER;                              // Mass Erase Enabled
  FLASH->CR |=  FLASH_CR_EOPIE;
  M32(0x08000000) = 0xFF;
  __asm("DSB");

  while (FLASH->SR & FLASH_SR_BSY)
  {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR &= ~FLASH_CR_MER;                              // Mass Erase Disabled
  FLASH->CR &= ~FLASH_CR_EOPIE;                            // Reset FLASH_EOPIE

  if (FLASH_SR_EOP != (FLASH->SR & FLASH_SR_EOP))             // Check for FLASH_SR_EOP
  {
    FLASH->SR |= FLASH_SR_EOP;
    return (1);                                         // Failed
  }
#endif //FLASH_OTP
  return (0);                                           // Done
}
#endif //FLASH_MEM

#ifdef FLASH_OB
int EraseChip(void)
{

  /* erase chip is not needed for
     - Flash Option bytes
     - Flash One Time Programmable bytes
  */
  return (0);                                           // Done
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
#ifdef FLASH_OTP
  ErasePage(OTP_BASE);
#else
  FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

  FLASH->CR  |=  FLASH_CR_SER;                             // Sector Erase Enabled
  FLASH->CR  |=  FLASH_CR_EOPIE;
  M32(adr) = 0xFF;                                      // Sector Address
  __asm("DSB");

  while (FLASH->SR  & FLASH_SR_BSY)
  {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR  &= ~FLASH_CR_SER;                             // Sector Erase Disabled
  FLASH->CR  &= ~FLASH_CR_EOPIE;                           // Reset FLASH_EOPIE

//  if (FLASH_EOP != (FLASH->SR & FLASH_EOP)) {           // Check for FLASH_SR_EOP
//    FLASH->SR |= FLASH_EOP;
//    return (1);                                         // Failed
//  }
#endif //FLASH_OTP
  return (0);                                           // Done
}
#endif //FLASH_MEM


#ifdef FLASH_OB
int EraseSector(unsigned long adr)
{
  /* erase sector is not needed for
     - Flash Option bytes
     - Flash One Time Programmable bytes
  */
  return (0);                                           // Done
}
#endif

/*
 *  Blank Check Checks if Memory is Blank
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */

int BlankCheck(unsigned long adr, unsigned long sz, unsigned char pat)
{
  return (1);                                            // Always Force Erase
}


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

  sz = (sz + 127) & ~127;                               // Adjust size for 32 Words

  FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

  while (sz)
  {
    FLASH->CR  |=  FLASH_CR_PG;                            // Programming Enabled
    FLASH->CR  |=  FLASH_CR_EOPIE;

    for (uint8_t i = 0; i < 32; i++)
    {

      M32(adr + i * 4) = *((uint32_t *)(buf + i * 4));       // Program the first word of the Double Word
      if (i == 30)
      {
        FLASH->CR  |= FLASH_CR_PGSTRT;
      }
    }
    __asm("DSB");

    while (FLASH->SR & FLASH_SR_BSY)
    {
      IWDG->KR = 0xAAAA;                                // Reload IWDG
    }

    FLASH->CR  &= ~FLASH_CR_PG;                            // Programming Disabled
    FLASH->CR  &= ~FLASH_CR_EOPIE;                         // Reset FLASH_EOPIE

//    if (FLASH_EOP != (FLASH->SR & FLASH_EOP)) {         // Check for FLASH_SR_EOP
//      FLASH->SR |= FLASH_EOP;
//      return (1);                                       // Failed
//    }

    adr += 128;                                         // Go to next Page
    buf += 128;
    sz  -= 128;
  }

  return (0);                                           // Done
}
#endif //  FLASH_MEM

#ifdef FLASH_OB
int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  uint32_t optr;
  uint32_t sdkr;
  uint32_t btcr;
  uint32_t wrpr;

  optr = *((uint32_t *)(buf + 0x00));
  sdkr = *((uint32_t *)(buf + 0x04));
  btcr = *((uint32_t *)(buf + 0x08));
  wrpr = *((uint32_t *)(buf + 0x0C));

  FLASH->SR |= FLASH_SR_EOP;                            // Reset FLASH_EOP

  FLASH->OPTR = (optr & 0x0000FFFF);                 // Write OPTR values
  FLASH->SDKR = (sdkr & 0x0000FFFF);                 // Write SDKR values
  FLASH->BTCR = (btcr & 0x0000FFFF);                 // Write BTCR values
  FLASH->WRPR = (wrpr & 0x0000FFFF);                 // Write WRPR values

  FLASH->CR |= FLASH_CR_OPTSTRT;
  FLASH->CR |= FLASH_CR_EOPIE;
  M32(0x40022080) = 0xFF;
  __asm("DSB");

  while (FLASH->SR & FLASH_SR_BSY);

  FLASH->CR  &= ~FLASH_CR_OPTSTRT;                      // Programming Disabled
  FLASH->CR  &= ~FLASH_CR_EOPIE;                        // Reset FLASH_EOPIE

  //FLASH->CR |= FLASH_CR_OBL_LAUNCH;

  return (0);                                           // Done
}
#endif //  FLASH_OPT


/*
 *  Verify Flash Contents
 *    Parameter:      adr:  Start Address
 *                    sz:   Size (in bytes)
 *                    buf:  Data
 *    Return Value:   (adr+sz) - OK, Failed Address
 */

#ifdef FLASH_OB
unsigned long Verify(unsigned long adr, unsigned long sz, unsigned char *buf)
{
  uint32_t optr;
  uint32_t sdkr;
  uint32_t wrpr;

  optr = *((uint32_t *)(buf +  0x00));
  sdkr = *((uint32_t *)(buf +  0x04));
  wrpr = *((uint32_t *)(buf +  0x0C));

  if (M32(adr + 0x00) != optr)
  {
    return (adr + 0x00);
  }
  if (M32(adr + 0x04) != sdkr)
  {
    return (adr + 0x04);
  }
  if (M32(adr + 0x0C) != wrpr)
  {
    return (adr + 0x0C);
  }

  return (adr + sz);
}
#endif // FLASH_OB

void InitRccAndFlashParam(void)
{
  FLASH_ACR_RESTORE = FLASH->ACR;
  RCC_ICSCR_RESTORE = RCC->ICSCR;
  RCC->ICSCR = (RCC->ICSCR & 0xFFFF0000) | (M32(0x1fff0100) & 0xFFFF);
  while (RCC_CR_HSIRDY != READ_BIT(RCC->CR, RCC_CR_HSIRDY));

  FLASH->TS0     = ((M32(0x1FFF011C) >>  0) & 0x000001FF);
  FLASH->TS3     = ((M32(0x1FFF011C) >>  9) & 0x000001FF);
  FLASH->TS1     = ((M32(0x1FFF011C) >> 18) & 0x000003FF);
  FLASH->TS2P    = ((M32(0x1FFF0120) >>  0) & 0x000001FF);
  FLASH->TPS3    = ((M32(0x1FFF0120) >> 16) & 0x00000FFF);

  FLASH->PERTPE  = ((M32(0x1FFF0124) >>  0) & 0x0003FFFF);
  FLASH->SMERTPE = ((M32(0x1FFF0128) >>  0) & 0x0003FFFF);
  FLASH->PRGTPE  = ((M32(0x1FFF012C) >>  0) & 0x0000FFFF);
  FLASH->PRETPE  = ((M32(0x1FFF012C) >> 16) & 0x00003FFF);
}

void UnInitRccAndFlashParam(void)
{
  FLASH->ACR = FLASH_ACR_RESTORE;
  RCC->ICSCR = RCC_ICSCR_RESTORE;
}

int ErasePage(unsigned long adr)
{

  FLASH->SR  |= FLASH_SR_EOP;                              // Reset FLASH_EOP

  FLASH->CR  |=  FLASH_CR_PER;                             // Sector Erase Enabled
  FLASH->CR  |=  FLASH_CR_EOPIE;
  M32(adr) = 0xFF;                                      // Sector Address
  __asm("DSB");

  while (FLASH->SR  & FLASH_SR_BSY)
  {
    IWDG->KR = 0xAAAA;                                  // Reload IWDG
  }

  FLASH->CR  &= ~FLASH_CR_PER;                             // Sector Erase Disabled
  FLASH->CR  &= ~FLASH_CR_EOPIE;                           // Reset FLASH_EOPIE

//  if (FLASH_EOP != (FLASH->SR & FLASH_EOP)) {           // Check for FLASH_SR_EOP
//    FLASH->SR |= FLASH_EOP;
//    return (1);                                         // Failed
//  }

  return (0);                                           // Done
}

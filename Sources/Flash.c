/*!
 * @addtogroup Flash_module Flash module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for erasing and writing to the Flash.
 *
 *  This contains the functions needed for accessing the internal Flash.
 *
 *  @author Robert Carey & Harry Mace
 *  @date 2019-04-07
 */


// new types
#include "types.h"
#include "Flash.h"


static bool LaunchCommand(TFCCOB* ccob)
{
  // Check status of Access Error  and Flash Protection Violation
  if (FTFE_FSTAT & (FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_FPVIOL_MASK))
  {
    // Clear if set (w1c)
    FTFE_FSTAT = (FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_FPVIOL_MASK);
  }

  // Load FCCOB registers
  //Command
  FTFE_FCCOB0 = FTFE_FCCOB0_CCOBn(ccob->command);
  //Address
  FTFE_FCCOB1 = FTFE_FCCOB1_CCOBn(ccob->address >> BYTE_SHIFT*2);
  FTFE_FCCOB2 = FTFE_FCCOB2_CCOBn(ccob->address >> BYTE_SHIFT);
  FTFE_FCCOB3 = FTFE_FCCOB3_CCOBn(ccob->address);
  // Loads Phrase into registers
  // The Phrase should already be aligned
  FTFE_FCCOB4 = FTFE_FCCOB4_CCOBn(ccob->phrase >> BYTE_SHIFT*7);
  FTFE_FCCOB5 = FTFE_FCCOB5_CCOBn(ccob->phrase >> BYTE_SHIFT*6);
  FTFE_FCCOB6 = FTFE_FCCOB6_CCOBn(ccob->phrase >> BYTE_SHIFT*5);
  FTFE_FCCOB7 = FTFE_FCCOB7_CCOBn(ccob->phrase >> BYTE_SHIFT*4);
  FTFE_FCCOB8 = FTFE_FCCOB8_CCOBn(ccob->phrase >> BYTE_SHIFT*3);
  FTFE_FCCOB9 = FTFE_FCCOB9_CCOBn(ccob->phrase >> BYTE_SHIFT*2);
  FTFE_FCCOBA = FTFE_FCCOBA_CCOBn(ccob->phrase >> BYTE_SHIFT);
  FTFE_FCCOBB = FTFE_FCCOBB_CCOBn(ccob->phrase);

  // Clear Command Complete Interrupt Flag
  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;

  // Wait until Command Complete Interrupt flag is Set
  while (~FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK){}

  return true;
}

static bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  TFCCOB write;
  // Setup write Common Command Object
  write.command = FCMD_PROGRAM_PHRASE;
  write.address = address;
  write.phrase = phrase.l;
  // Call Launch command
  LaunchCommand(&write);// Pass address of ccob
}

static bool EraseSector(const uint32_t address)
{
  TFCCOB erase;
  // Setup erase Common Command Object
  erase.command = FCMD_ERASE_SECTOR;
  erase.address = address;
  // Call Launch command
  LaunchCommand(&erase);// Pass address of ccob
}

static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase)
{
  // Erase Sector and then write phrase
  return (EraseSector(address) && WritePhrase(address,phrase));
}

bool Flash_Init(void)
{
  // Enable Memory Protection Unit clock
  SIM_SCGC7 |= SIM_SCGC7_MPU_MASK;
  return true;
}

bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  // Mask used to store the availability of addresses
  static uint8_t availableAddrs = 0b11111111;

  // Variable to store current address mask
  uint8_t currentMask;

  // Switch case to store correct mask based on size passed
  switch (size)
  {
    case SIZE_BYTE:
      currentMask = ADDRS_BYTE;
      break;
    case SIZE_halfWord:
      currentMask = ADDRS_halfWord;
      break;
    case SIZE_WORD:
      currentMask = ADDRS_WORD;
      break;
    default:
      return false;
      break;
  }

  // Check against available addresses for a free spot
  for (uint8_t i = 0; i<8; i+=size)
  {
    if (availableAddrs & (currentMask>>i))// If there is a free spot
    {
      availableAddrs &= ~(currentMask>>i);// Update the availableAddrs
      *variable = (uint32_t*)(FLASH_DATA_START + i);// Assign the address to the variable
      return true;
    }
  }

  return false;
}

bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  // Calculate the offset
  uint8_t offset = (uint32_t)address - FLASH_DATA_START;
  uint64union_t phrase;

  // Check if the offset location aligns with phrase
  if (!(offset % SIZE_PHRASE))// Aligned
  {
    phrase.s.Hi = data;
    phrase.s.Lo = _FW(FLASH_DATA_START + offset + SIZE_WORD);// pass phrase aligned address
    return ModifyPhrase((uint32_t)address, phrase);
  }
  else// Not Aligned
  {
    phrase.s.Hi = _FW(FLASH_DATA_START + offset - SIZE_WORD);
    phrase.s.Lo = data;
    return ModifyPhrase((uint32_t)address - SIZE_WORD, phrase);// pass phrase aligned address
  }
}

bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  // Calculate the offset
  uint8_t offset = (uint32_t)address - FLASH_DATA_START;
  uint32union_t word;

  // Check if the offset location aligns with word
  if (!(offset % SIZE_WORD))// Aligned
  {
    word.s.Hi = _FH(FLASH_DATA_START + offset + SIZE_halfWord);
    word.s.Lo = data;
    return Flash_Write32((uint32_t*)address, word.l);// pass word aligned address
  }
  else// Not aligned
  {
    word.s.Hi = data;
    word.s.Lo = _FH(FLASH_DATA_START + offset - SIZE_halfWord);
    return Flash_Write32((uint32_t)address - SIZE_halfWord, word.l);// pass word aligned address
  }
}

bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  // Calculate the offset
  uint8_t offset = (uint32_t)address - FLASH_DATA_START;
  uint16union_t halfWord;

  // Check if the offset location aligns with half-word
  if (!(offset % SIZE_halfWord))// Aligned
  {
    halfWord.s.Hi = _FB(FLASH_DATA_START + offset + SIZE_BYTE);
    halfWord.s.Lo = data;
    return Flash_Write16((uint16_t*)address, halfWord.l);// pass half-word aligned address
  }
  else // Not aligned
  {
    halfWord.s.Hi = data;
    halfWord.s.Lo = _FB(FLASH_DATA_START + offset - SIZE_BYTE);
    return Flash_Write16((uint16_t*)(address - SIZE_BYTE), halfWord.l);// pass half-word aligned address
  }

}

bool Flash_Erase(void)
{
  // Calls erase sector
  EraseSector(FLASH_DATA_START);
}


/*!
 * @}
 */

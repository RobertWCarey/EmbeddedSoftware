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
 *  @date 2019-04-06
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
  //Phrase
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

  write.command = FCMD_PROGRAM_PHRASE;
  write.address = address;
  write.phrase = phrase.l;

  LaunchCommand(&write);
}

static bool EraseSector(const uint32_t address)
{
  TFCCOB erase;

  erase.command = FCMD_ERASE_SECTOR;
  erase.address = address;

  LaunchCommand(&erase);
}

static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase)
{
  return (EraseSector(address) && WritePhrase(address,phrase));
}

bool Flash_Init(void)
{
  // Enable Memory Protection Unit clock
  SIM_SCGC7 |= SIM_SCGC7_MPU_MASK;
  return true;
}

/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 *  @note Assumes Flash has been initialized.
 */
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
    case SIZE_HALF_WORD:
      currentMask = ADDRS_HALF_WORD;
      break;
    case SIZE_WORD:
      currentMask = ADDRS_WORD;
      break;
    default:
      return false;
      break;
  }

  // Check against available addresses for a free spot
  for (uint8_t i = 0; i<8; i++)
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

/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{

}

/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{

}

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{

}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void)
{

}


/*!
 * @}
 */

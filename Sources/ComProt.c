/* @addtogroup CmdProt_module Command Protocol module documentation
 * @{
 */
/*! @file
 *
 *  @brief Function for communicating using the PMcL Tower Communication Protocol.
 *
 *  This contains the functions for communicating via the PMcL Communication Protocol.
 *
 *  @author Robert Carey
 *  @date 2019-06-02
 */

#include "ComProt.h"

// Parameters for 0x04-Tower Startup
static const uint8_t TOWER_STARTUP_PARAM = 0x00;

// Parameters for 0x09-Special-Tower Version
static const uint8_t TOWER_SPECIAL_V = 0x76; // ASCII "v" in Hex
static const uint8_t TOWER_SPECIAL_X = 0x78; // ASCII "x" in Hex
static const uint8_t TOWER_VERSION_MAJOR = 0x01;
static const uint8_t TOWER_VERSION_MINOR = 0x00;

// Parameters for 0x0B-Tower Number
static const uint8_t TOWER_NUMBER_GET = 0x01;// Get Param
static const uint8_t TOWER_NUMBER_SET = 0x02;// Set Param

// Parameters for 0x0D-Tower Mode
static const uint8_t TOWER_MODE_GET = 0x01;// Get Param
static const uint8_t TOWER_MODE_SET = 0x02;// Set Param

// Parameters for 0x07-Program Byte
static const uint8_t PROGRAM_BYTE_ERASE = 0x08;// Erase Sector Param
static const uint8_t PROGRAM_BYTE_RANGE_LO = 0x00;// Lowest valid value Param
static const uint8_t PROGRAM_BYTE_RANGE_HI = 0x08;// Highest valid value Param

// Parameters for 0x08-Read Byte
static const uint8_t READ_BYTE_RANGE_LO = 0x00;// Lowest valid value Param
static const uint8_t READ_BYTE_RANGE_HI = 0x07;// Highest valid value Param

// Parameters for 0x70 - DOR
#define DOR_IDMT_CHARAC 0// Select "IDMT characteristic"
static const uint8_t DOR_IDMT_GET = 1;// GET IDMT characteristic
static const uint8_t DOR_IDMT_SET = 2;// SET IDMT characteristic
static const uint8_t DOR_IDMT_INVERSE = 0;// IDMT "Inverse"
static const uint8_t DOR_IDMT_VINVERSE = 1;// IDMT "Very Inverse"
static const uint8_t DOR_IDMT_EINVERSE = 2;// IDMT "Extremely Inverse"
#define DOR_CURRENT 1// Select "Get Currents"
#define DOR_FREQ 2// Select "Get Frequency"
#define DOR_TIMES_TRIPPED 3// Select "Get # of times Tripped"
#define DOR_FAULT_TYPE 4// Select "Get Fault Type"

bool towerStatupPacketHandler (volatile uint16union_t * const towerNb,volatile uint16union_t * const towerMode)
{
  // Check that params are valid
  if ( (Packet_Parameter1 == TOWER_STARTUP_PARAM) && (Packet_Parameter2 == TOWER_STARTUP_PARAM) && (Packet_Parameter3 == TOWER_STARTUP_PARAM) )
    return Packet_Put(CMD_TOWER_STARTUP,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM) &&
      Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR) &&
      Packet_Put(CMD_TOWER_NUMBER,TOWER_NUMBER_GET,towerNb->s.Lo,towerNb->s.Hi)&&
      Packet_Put(CMD_TOWER_MODE,TOWER_MODE_GET,towerMode->s.Lo,towerMode->s.Hi);

  // If invalid params return false
  return false;
}

/*! @brief Returns values for Special packet requested.
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool specialPacketHandler()
{
  // Check which special selected
  if ( (Packet_Parameter1 == TOWER_SPECIAL_V) && (Packet_Parameter2 == TOWER_SPECIAL_X) )// If Get Version selected
    return Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR);

  return false;
}

/*! @brief Handles Tower Number packets.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if packet successfully sent.
 */
static bool towerNumberPacketHandler(volatile uint16union_t * const towerNb)
{
  // Check which type of Tower Number Packet
  // If Get
  if ( (Packet_Parameter1 == TOWER_NUMBER_GET) && !(Packet_Parameter2) && !(Packet_Parameter3) )
    // Send out Tower Number packet
    return Packet_Put(CMD_TOWER_NUMBER,TOWER_NUMBER_GET,towerNb->s.Lo,towerNb->s.Hi);
  else if (Packet_Parameter1 == TOWER_NUMBER_SET) // If Set
  {
    //Update Tower Number Values
    return Flash_Write16((uint16_t*)towerNb,Packet_Parameter23);
  }

  return false;
}

/*! @brief Handles Tower Number packets.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if packet successfully sent.
 */
static bool towerModePacketHandler(volatile uint16union_t * const towerMode)
{
  // Check which type of Tower Number Packet
  // If Get
  if ( (Packet_Parameter1 == TOWER_MODE_GET) && !(Packet_Parameter2) && !(Packet_Parameter3) )
    // Send out Tower Number packet
    return Packet_Put(CMD_TOWER_MODE,TOWER_MODE_GET,towerMode->s.Lo,towerMode->s.Hi);
  else if (Packet_Parameter1 == TOWER_NUMBER_SET) // If Set
  {
    //Update Tower Number Values
    return Flash_Write16((uint16_t*)towerMode,Packet_Parameter23);
  }

  return false;
}

/*! @brief Executes Program byte.
 *
 *  @return bool - TRUE if data successfully programmed.
 */
static bool prgmBytePacketHandler()
{
  // Check offset is valid, and parameter byte is valid
  if ((Packet_Parameter1 < PROGRAM_BYTE_RANGE_LO) || (Packet_Parameter1 > PROGRAM_BYTE_RANGE_HI) || (Packet_Parameter2))
    return false;
  // Check if erase sector has been requested
  if (Packet_Parameter1 == PROGRAM_BYTE_ERASE)
    return Flash_Erase();
  // Write data to selected address offset
  return Flash_Write8((uint8_t*)(FLASH_DATA_START+Packet_Parameter1),Packet_Parameter3);

}

/*! @brief Executes Read byte.
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool readBytePacketHandler()
{
  // Check offset is valid, and parameter byte is valid
  if ((Packet_Parameter1 < READ_BYTE_RANGE_LO) || (Packet_Parameter1 > READ_BYTE_RANGE_HI) || (Packet_Parameter2) || (Packet_Parameter3))
    return false;

  return Packet_Put(CMD_READ_BYTE, Packet_Parameter1, 0, _FB(FLASH_DATA_START+Packet_Parameter1));

}

static bool myIDMTCharacteristicHandler(TIDMTCharacter* const characteristic)
{
  if ((Packet_Parameter2 == DOR_IDMT_GET) && !Packet_Parameter3)
  {
    return Packet_Put(CMD_DOR, Packet_Parameter1, DOR_IDMT_GET, *characteristic);
  }

  return false;
}

static bool dorPacketHandler(TIDMTCharacter* const characteristic)
{
  bool success = 0;

  switch (Packet_Parameter1)
  {
    case DOR_IDMT_CHARAC:
      success = myIDMTCharacteristicHandler(characteristic);
      break;
    case DOR_FREQ:
      break;
    case DOR_CURRENT:
      break;
    case DOR_TIMES_TRIPPED:
      break;
    case DOR_FAULT_TYPE:
      break;
    default:
      success = false;
      break;
  }


  return success;
}

void cmdHandler(volatile uint16union_t * const towerNb, volatile uint16union_t * const towerMode, TIDMTCharacter* const characteristic)
{

  // Isolate command packet
  uint8_t command = Packet_Command & ~PACKET_ACK_MASK;
  // Isolate ACK request
  uint8_t ack = Packet_Command & PACKET_ACK_MASK;
  // Variable to record if action was successful
  bool success = 0;

  switch (command)
  {
    case CMD_TOWER_STARTUP:
      success = towerStatupPacketHandler(towerNb,towerMode);
      break;
    case CMD_SPECIAL_TOWER_VERSION:
      success = specialPacketHandler();
      break;
    case CMD_TOWER_NUMBER:
      success = towerNumberPacketHandler(towerNb);
      break;
    case CMD_TOWER_MODE:
      success = towerModePacketHandler(towerMode);
      break;
    case CMD_PROGRAM_BYTE:
      success = prgmBytePacketHandler();
      break;
    case CMD_READ_BYTE:
      success = readBytePacketHandler();
      break;
    case CMD_DOR:
      success = dorPacketHandler(characteristic);
      break;
    default:
      break;
  }

  // Check if ACK Requested
  if (ack)
    // Check if action was successful
    if (success)// If success send same packet
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    else // If !success send packet with NACK
    {
      // Send Nack if required
      uint8_t nackCommand = Packet_Command & ~PACKET_ACK_MASK;
      Packet_Put(nackCommand,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }

  // Reset Packet variables
  Packet_Command = 0x00u;
  Packet_Parameter1 = 0x00u;
  Packet_Parameter2 = 0x00u;
  Packet_Parameter3 = 0x00u;
  Packet_Checksum = 0x00u;
}

/*!
 * @}
 */

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

bool towerStatupPacketHandler (volatile uint8_t* const characteristic)
{
  // Check that params are valid
  if ( (Packet_Parameter1 == TOWER_STARTUP_PARAM) && (Packet_Parameter2 == TOWER_STARTUP_PARAM) && (Packet_Parameter3 == TOWER_STARTUP_PARAM) )
    return Packet_Put(CMD_TOWER_STARTUP,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM) &&
      Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR) &&
      Packet_Put(CMD_DOR,DOR_IDMT_CHARAC,DOR_IDMT_GET,*characteristic);

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


/*! @brief Gets or Sets the IDMT Characteristic depending on the value of Parameter 2 and 3
 *
 *  @param  characteristic  pointer to a location containing the current IDMT Characteristic
 *
 *  @return bool - TRUE if packet successfully sent/ value successfully set.
 */
static bool myIDMTCharacteristicHandler(volatile uint8_t* const characteristic)
{
  if ((Packet_Parameter2 == DOR_IDMT_GET) && !Packet_Parameter3)
  {
    return Packet_Put(CMD_DOR, Packet_Parameter1, DOR_IDMT_GET, *characteristic);
  }
  else if ((Packet_Parameter2 == DOR_IDMT_SET) && (Packet_Parameter3 >= DOR_IDMT_INVERSE) && (Packet_Parameter3 <= DOR_IDMT_EINVERSE))
  {
    // Write data to selected address offset
    return Flash_Write8(characteristic,Packet_Parameter3);
  }

  return false;
}

/*! @brief Gets the current for all 3phases
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool myDORCurrentHandler()
{
  if (Packet_Parameter2 && Packet_Parameter3)
    return false;

  for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    float irms = ChannelThreadData[i].irms;
    uint8_t high = (uint8_t)(irms);
    uint8_t low = (uint8_t)((irms-high)*100);
    if (!Packet_Put(CMD_DOR_CURRENT, ChannelThreadData[i].channelNb, low, high))
      return false;
  }

  return true;
}

/*! @brief Gets the frequency from phase a
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool myDORFreqHandler()
{
  if (Packet_Parameter2 && Packet_Parameter3)
    return false;

  float freq = ChannelThreadData[0].frequency[0];
  uint8_t high = (uint8_t)(freq);
  uint8_t low = (uint8_t)((freq-high)*100);
  return Packet_Put(DOR_FREQ, Packet_Parameter1, low, high);

}

static bool dorPacketHandler(volatile uint8_t* const characteristic)
{
  bool success = 0;

  switch (Packet_Parameter1)
  {
    case DOR_IDMT_CHARAC:
      success = myIDMTCharacteristicHandler(characteristic);
      break;
    case DOR_FREQ:
      success = myDORFreqHandler();
      break;
    case DOR_CURRENT:
      success = myDORCurrentHandler();
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

void cmdHandler(volatile uint8_t* const characteristic)
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
      success = towerStatupPacketHandler(characteristic);
      break;
    case CMD_SPECIAL_TOWER_VERSION:
      success = specialPacketHandler();
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

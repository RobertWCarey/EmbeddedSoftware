/* @addtogroup FTM_module FTM module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for setting up the FlexTimer module (FTM) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the FlexTimer module (FTM).
 *
 *  @author PMcL
 *  @date 2015-09-04
 */

// new types
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

// Parameters for 0x0C-Time
static const uint8_t TIME_RANGE_LO = 0x00;// Lowest valid value
static const uint8_t TIME_HOURS_RANGE_HI = 23;// Highest hours valid value
static const uint8_t TIME_MINUTES_RANGE_HI = 59;// Highest minutes valid value
static const uint8_t TIME_SECONDS_RANGE_HI = 59;// Highest seconds valid value

// Parameters for 0x0A - Protocol Mode
static const uint8_t PROT_MODE_GET = 1;// Get Param
static const uint8_t PROT_MODE_SET = 2;// Set Param
static const uint8_t PROT_MODE_ASYNC = 0;// Asynchronous mode (PIT polling)
static const uint8_t PROT_MODE_SYNC = 1;// Synchronous mode (Accel interrupt)
static const uint8_t PROT_MODE_PARAM3 = 0;// Parameter 3 for 0x0A

bool towerStatupPacketHandler (volatile uint16union_t * const towerNb,volatile uint16union_t * const towerMode, const TAccelMode* AccelMode)
{
  // Check that params are valid
  if ( (Packet_Parameter1 == TOWER_STARTUP_PARAM) && (Packet_Parameter2 == TOWER_STARTUP_PARAM) && (Packet_Parameter3 == TOWER_STARTUP_PARAM) )
    return Packet_Put(CMD_TOWER_STARTUP,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM) &&
      Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR) &&
      Packet_Put(CMD_TOWER_NUMBER,TOWER_NUMBER_GET,towerNb->s.Lo,towerNb->s.Hi)&&
      Packet_Put(CMD_TOWER_MODE,TOWER_MODE_GET,towerMode->s.Lo,towerMode->s.Hi)&&
      Packet_Put(CMD_PROT_MODE, 0, *AccelMode, 0);

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

/*! @brief sets the time for the RTC.
 *
 *  @return bool - TRUE if data successfully programmed.
 */
static bool timePacketHandler()
{
  // Check lower values for valid range
  if ((Packet_Parameter1 >= TIME_RANGE_LO) && (Packet_Parameter2 >= TIME_RANGE_LO) && (Packet_Parameter3 >= TIME_RANGE_LO))
    //Check upper values for valid range
    if ((Packet_Parameter1 <= TIME_HOURS_RANGE_HI) && (Packet_Parameter2 <= TIME_MINUTES_RANGE_HI) && (Packet_Parameter3 <= TIME_SECONDS_RANGE_HI))
      {
	// Update the current RTC value
	RTC_Set(Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
	return true;
      }

  return false;
}

/*! @brief Executes Protocol Mode packet handler.
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool protModePacketHandler(TAccelMode* const AccelMode)
{
  // Check offset is valid, and parameter byte is valid
  if ((Packet_Parameter3 != PROT_MODE_PARAM3))
    return false;

  // Check for Valid get parameters
  if ( (Packet_Parameter1 == PROT_MODE_GET) && !(Packet_Parameter2) )
    {
      // Send current mode
      return Packet_Put(CMD_PROT_MODE, PROT_MODE_GET, *AccelMode, PROT_MODE_PARAM3);
    }
  // Check for valid Set parameters
  else if (Packet_Parameter1 == PROT_MODE_SET)
    {
      //Set mode based on packet_parameter2 as long as valid
      if ((Packet_Parameter2 == PROT_MODE_ASYNC) || (Packet_Parameter2 == PROT_MODE_SYNC))
	{
	  // Update accelerometer mode
	  Accel_SetMode(Packet_Parameter2);
	  // Update variable storing current mode
	  *AccelMode = Packet_Parameter2;
	  return true;
	}
    }

  return false;
}

void cmdHandler(volatile uint16union_t * const towerNb, volatile uint16union_t * const towerMode, const TFTMChannel* const aFTMChannel, TAccelMode* const AccelMode)
{
  //Starts a timer and turns on LED
  FTM_StartTimer(aFTMChannel);
  LEDs_On(LED_BLUE);

  // Isolate command packet
  uint8_t command = Packet_Command & ~PACKET_ACK_MASK;
  // Isolate ACK request
  uint8_t ack = Packet_Command & PACKET_ACK_MASK;
  // Variable to record if action was successful
  bool success = 0;

  switch (command)
  {
    case CMD_TOWER_STARTUP:
      success = towerStatupPacketHandler(towerNb,towerMode,AccelMode);
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
    case CMD_TIME_BYTE:
      success = timePacketHandler();
      break;
    case CMD_PROT_MODE:
      success = protModePacketHandler(AccelMode);
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

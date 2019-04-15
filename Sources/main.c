/* ###################################################################
**     Filename    : main.c
**     Project     : Lab2
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/

/*!
**  @addtogroup main_module Main module documentation
**  @{
*/
/*!
** @file main.c
** @version 2.0
** @brief
**         Main module.
**         This module contains user's application code.
*/
/* MODULE main */

// CPU module - contains low level hardware initialization routines
#include "Cpu.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "packet.h"
#include "UART.h"
#include "LEDs.h"
#include "Flash.h"

// Baud Rate
static const uint32_t BAUD_RATE = 115200;

// Command Values
#define CMD_TOWER_STARTUP 0x04u
#define CMD_SPECIAL_TOWER_VERSION 0x09u
#define CMD_TOWER_NUMBER 0x0Bu
#define CMD_PROGRAM_BYTE 0x07u
#define CMD_READ_BYTE 0x08u
#define CMD_TOWER_MODE 0x0Du

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

/*! @brief Sends out required packets for Tower Startup.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if all packets were successfully sent.
 */
static bool towerStatupPacketHandler (volatile uint16union_t * const towerNb,volatile uint16union_t * const towerMode)
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
  if(Packet_Parameter1 == PROGRAM_BYTE_ERASE)
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

static void flashSetup(volatile uint16union_t * const addrs, uint16_t defaultData)
{
  Flash_AllocateVar((void*)&addrs, sizeof(*addrs));
  if(addrs->l == 0xffff)
    Flash_Write16((uint16_t*)addrs, defaultData);
}

/*! @brief Performs necessary action for any valid packets received.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return void.
 */
static void cmdHandler(volatile uint16union_t * const towerNb, volatile uint16union_t * const towerMode)
{
  // Isolate command packet
  uint8_t command = Packet_Command & ~PACKET_ACK_MASK;
  // Isolate ACK request
  uint8_t ack = Packet_Command & PACKET_ACK_MASK;
  // Variable to record if action was successful
  bool success = 0;

  switch(command)
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
    default:
      break;
  }

  // Check if ACK Requested
  if (ack)
    // Check if action was successful
    if (success)// If success send same packet
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    else// If !success send packet with NACK
    {
      // Send Nack if required
      uint8_t nackCommand = Packet_Command & ~PACKET_ACK_MASK;
      Packet_Put(nackCommand,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
}

/*! @brief Runs all functions necessary for Tower to function.
 *
 *  @return bool - TRUE if all components successfully initialized.
 */
static bool towerInit(void)
{
  return Packet_Init(BAUD_RATE,CPU_BUS_CLK_HZ) &&
      LEDs_Init() &&
      Flash_Init();
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
  uint16_t defaultTowerNb = 9382;
  uint16_t defaultTowerMode = 1;
  volatile uint16union_t *nvTowerNb;
  volatile uint16union_t *nvTowerMode;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  // Initialize Tower
  if (towerInit())
  {
    LEDs_On(LED_ORANGE);
    Flash_AllocateVar((void*)&nvTowerNb, sizeof(*nvTowerNb));
    Flash_AllocateVar((void*)&nvTowerMode, sizeof(*nvTowerMode));
    if(nvTowerNb->l == 0xffff)
      Flash_Write16((uint16_t*)nvTowerNb, defaultTowerNb);
    if(nvTowerMode->l == 0xffff)
      Flash_Write16((uint16_t*)nvTowerMode, defaultTowerMode);

    towerStatupPacketHandler(nvTowerNb,nvTowerMode);
  }

  for (;;)
  {

    // Check status of UART
    UART_Poll();

    // Check if any valid Packets have been received
    if (!Packet_Get())
      continue;// If no valid packet go to start of loop

    // Deal with any received packets
    cmdHandler(nvTowerNb,nvTowerMode);

    // Reset Packet variables
    Packet_Command = 0x00u;
    Packet_Parameter1 = 0x00u;
    Packet_Parameter2 = 0x00u;
    Packet_Parameter3 = 0x00u;
    Packet_Checksum = 0x00u;
  }

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/

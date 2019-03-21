/* ###################################################################
**     Filename    : main.c
**     Project     : Lab1
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
** @file main.c
** @version 2.0
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


// CPU module - contains low level hardware initialization routines
#include "Cpu.h"
#include "packet.h"
#include "UART.h"

// Baud Rate
static uint32_t BAUD_RATE = 38400;

// Command Values
#define CMD_TOWER_STARTUP 0x04u
#define CMD_SPECIAL_TOWER_VERSION 0x09u
#define CMD_TOWER_NUMBER 0x0Bu

// Parameters for 0x04-Tower Startup
#define TOWER_STARTUP_PARAM 0x00u

// Parameters for 0x09-Special-Tower Version
#define TOWER_SPECIAL_V 0x76u // ASCII "v" in Hex
#define TOWER_VERSION_MAJOR 0x01u
#define TOWER_VERSION_MINOR 0x00u

// Parameters for 0x0B-Tower Number
#define TOWER_NUMBER_PARAM1 0x01u

/*! @brief Sends out required packets for Tower Startup.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if all packets were successfully sent.
 */
static bool towerStatupPacketHandler (uint16union_t * const towerNb)
{
  return Packet_Put(CMD_TOWER_STARTUP,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM) &&
      Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR) &&
      Packet_Put(CMD_TOWER_NUMBER,TOWER_NUMBER_PARAM1,towerNb->s.Lo,towerNb->s.Hi);
}

/*! @brief Returns values for Special packet requested.
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool specialPacketHandler()
{
  // Check which special selected
  if (Packet_Parameter1 == TOWER_SPECIAL_V)// If Get Version selected
    return Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR);

  return false;
}

/*! @brief Handles Tower Number packets.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if packet successfully sent.
 */
static bool towerNumberPacketHandler(uint16union_t * const towerNb)
{
  // Check which type of Tower Number Packet
  if (Packet_Parameter1 == 0x01u)// If Get
    // Send out Tower Number packet
    return Packet_Put(CMD_TOWER_NUMBER,TOWER_NUMBER_PARAM1,towerNb->s.Lo,towerNb->s.Hi);
  else if (Packet_Parameter1 == 0x02u) // If Set
  {
    //Update Tower Number Values
    towerNb->s.Lo = Packet_Parameter2;
    towerNb->s.Hi = Packet_Parameter3;
    return true;
  }

  return false;
}

/*! @brief Performs necessary action for any valid packets received.
 *
 *  @return void.
 */
static void cmdHandler(uint16union_t * const towerNb)
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
      success = towerStatupPacketHandler(towerNb);
      break;
    case CMD_SPECIAL_TOWER_VERSION:
      success = specialPacketHandler();
      break;
    case CMD_TOWER_NUMBER:
      success = towerNumberPacketHandler(towerNb);
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
      uint8_t NACK_Command = Packet_Command & ~PACKET_ACK_MASK;
      Packet_Put(NACK_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
}

/*! @brief Runs all functions necessary for Tower to function.
 *
 *  @return bool - TRUE if all components successfully initialized.
 */
static bool towerInit(void)
{
  return Packet_Init(BAUD_RATE,CPU_BUS_CLK_HZ);
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
  uint16union_t towerNumber;
  towerNumber.l = 9382;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  // Initialize Tower
  if (towerInit())
    towerStatupPacketHandler(&towerNumber);

  for (;;)
  {
    // Check status of UART
    UART_Poll();

    // Check if any valid Packets have been received
    if (!Packet_Get())
      continue;// If no valid packet go to start of loop

    // Deal with any received packets
    cmdHandler(&towerNumber);

    // Reset command variable
    Packet_Command = 0x00u;
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

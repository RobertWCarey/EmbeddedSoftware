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

#ifndef ComProt_H
#define ComProt_H

#include "types.h"
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
#include "PIT.h"


// Command Values
#define CMD_TOWER_STARTUP 0x04u         /* "Tower Startup" */
#define CMD_SPECIAL_TOWER_VERSION 0x09u /* "Special - Tower Version" */
#define CMD_TOWER_NUMBER 0x0Bu          /* "Tower Number" */
#define CMD_PROGRAM_BYTE 0x07u          /* "Flash - Program Byte" */
#define CMD_READ_BYTE 0x08u             /* "Flash - Read Byte" */
#define CMD_TOWER_MODE 0x0Du            /* "Tower Mode" */
#define CMD_DOR 0x70u                   /* "DOR" */

/*! @brief Performs necessary action for any valid packets received.
 *
 *  @param towerNb A pointer to the Tower Number.
 *  @param towerNb A pointer to the Tower Mode.
 *  @param towerNb A pointer to the the current FTMxChannely setup struct x=FTM no. y=Channel no..
 *  @param towerNb A pointer to the current accelerometer mode.
 *  @return void.
 */
void cmdHandler(volatile uint16union_t * const towerNb, volatile uint16union_t * const towerMode);

/*! @brief Sends out required packets for Tower Startup.
 *
 *  @param towerNb A pointer to the Tower Number.
 *  @param towerNb A pointer to the Tower Mode.
 *  @param towerNb A pointer to the current accelerometer mode.
 *  @return bool - TRUE if all packets were successfully sent.
 */
bool towerStatupPacketHandler (volatile uint16union_t * const towerNb,volatile uint16union_t * const towerMode);

#endif
/*!
 * @}
 */

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
#include "Flash.h"
#include "PIT.h"
#include "DOR.h"


// Command Values
#define CMD_TOWER_STARTUP 0x04u         /* "Tower Startup" */
#define CMD_SPECIAL_TOWER_VERSION 0x09u /* "Special - Tower Version" */
#define CMD_DOR 0x70u                   /* "DOR" */
#define CMD_DOR_CURRENT 0x71u           /* "DOR Current" */

/*! @brief Performs necessary action for any valid packets received.
 *
 *  @param characteristic A pointer to the current IDMT characteristic.
 *  @return void.
 */
void cmdHandler(volatile uint8_t* const characteristic, volatile uint16union_t* const timesTripped, volatile uint8_t* const faultType);

/*! @brief Sends out required packets for Tower Startup.
 *
 *  @param towerNb A pointer to the Tower Number.
 *  @param towerNb A pointer to the Tower Mode.
 *  @param towerNb A pointer to the current accelerometer mode.
 *  @return bool - TRUE if all packets were successfully sent.
 */
bool towerStatupPacketHandler (volatile uint8_t* const characteristic);

#endif
/*!
 * @}
 */

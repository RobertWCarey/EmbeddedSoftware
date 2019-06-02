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

#ifndef ComProt_H
#define ComProt_H

// new types
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
#include "RTC.h"
#include "FTM.h"
#include "accel.h"
#include "median.h"


// Command Values
#define CMD_TOWER_STARTUP 0x04u
#define CMD_SPECIAL_TOWER_VERSION 0x09u
#define CMD_TOWER_NUMBER 0x0Bu
#define CMD_PROGRAM_BYTE 0x07u
#define CMD_READ_BYTE 0x08u
#define CMD_TOWER_MODE 0x0Du
#define CMD_TIME_BYTE 0xCu
#define CMD_ACCEL_VAL 0x10u
#define CMD_PROT_MODE 0x0Au

/*! @brief Performs necessary action for any valid packets received.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return void.
 */
void cmdHandler(volatile uint16union_t * const towerNb, volatile uint16union_t * const towerMode, const TFTMChannel* const aFTMChannel, TAccelMode* const AccelMode);

/*! @brief Sends out required packets for Tower Startup.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if all packets were successfully sent.
 */
bool towerStatupPacketHandler (volatile uint16union_t * const towerNb,volatile uint16union_t * const towerMode, const TAccelMode* AccelMode);

#endif
/*!
 * @}
 */

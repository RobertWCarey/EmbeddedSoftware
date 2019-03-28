/*!
 * @addtogroup Packet_module Packet module documentation
 * @{
 */

/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author PMcL
 *  @date 2015-07-23
 */

#ifndef PACKET_H
#define PACKET_H

// New types
#include "types.h"

// Packet structure
extern uint8_t 	Packet_Command,		/*!< The packet's command */
		Packet_Parameter1, 	/*!< The packet's 1st parameter */
		Packet_Parameter2, 	/*!< The packet's 2nd parameter */
		Packet_Parameter3,	/*!< The packet's 3rd parameter */
		Packet_Checksum;	/*!< The packet's checksum */

// Acknowledgment bit mask
extern const uint8_t PACKET_ACK_MASK;

/*! @brief Returns the checksum for a packet.
 *
 *  @param command 	The packet's command.
 *  @param parameter1 	The packet's 1st parameter.
 *  @param parameter2 	The packet's 2nd parameter.
 *  @param parameter3 	The packet's 3rd parameter.
 *
 *  @return uint8_t - Returns the calculated packet checksum.
 */
static uint8_t returnChecksum(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);


/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk);

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void);

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);

#endif

/*!
 * @}
 */

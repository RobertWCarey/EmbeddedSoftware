/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author Robert Carey and Harry Mace
 *  @date 2018-03-17
 */

// Include Header Files
#include "packet.h"
#include "UART.h"

// Acknowledgment bit mask
const uint8_t PACKET_ACK_MASK = 0x80u;

// Packet structure
uint8_t   Packet_Command,	/*!< The packet's command */
	  Packet_Parameter1, 	/*!< The packet's 1st parameter */
	  Packet_Parameter2, 	/*!< The packet's 2nd parameter */
	  Packet_Parameter3,	/*!< The packet's 3rd parameter */
	  Packet_Checksum;	/*!< The packet's checksum */

bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  return UART_Init(baudRate,moduleClk);
}

bool Packet_Get(void)
{
  // Read Packet values into variables
  while(UART_InChar(&Packet_Checksum)){
      // Check if a valid packet
      if ( ((Packet_Command^Packet_Parameter1^Packet_Parameter2^Packet_Parameter3)==Packet_Checksum) && (Packet_Command!=0) )
	return true;
      // If not a valid packet cycle values
      Packet_Command = Packet_Parameter1;
      Packet_Parameter1 = Packet_Parameter2;
      Packet_Parameter2 = Packet_Parameter3;
      Packet_Parameter3 = Packet_Checksum;
  }
  // If no more values
  return false;
}

bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  // Send bytes of packet into UART
  if (!UART_OutChar(command))
    return false;
  if (!UART_OutChar(parameter1))
      return false;
  if (!UART_OutChar(parameter2))
      return false;
  if (!UART_OutChar(parameter3))
      return false;
  if (!UART_OutChar(command^parameter1^parameter2^parameter3))
    return false;

  // If all bytes successfully written to UART
  return true;
}


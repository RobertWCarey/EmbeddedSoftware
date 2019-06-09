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
 *  @author Robert Carey and Harry Mace
 *  @date 2018-03-17
 */

// Include Header Files
#include "packet.h"

// Acknowledgment bit mask
const uint8_t PACKET_ACK_MASK = 0x80u;

// Define Packet
TPacket Packet;

// Mutex to restrict access to Packet_Put() function
OS_ECB* PacketAccess;

static uint8_t returnChecksum(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  return (command^parameter1^parameter2^parameter3);
}

bool Packet_Init(const TPacketSetup* const packetSetup)
{
  // Create Semaphore
  PacketAccess = OS_SemaphoreCreate(1);

  // UART setup struct
  TUARTSetup UARTSetup;
  UARTSetup.baudRate = packetSetup->UARTBaudRate;
  UARTSetup.moduleClk = packetSetup->UARTModuleClk;
  UARTSetup.TxParams = packetSetup->UARTTxParams;
  UARTSetup.RxParams = packetSetup->UARTRxParams;

  return UART_Init(&UARTSetup);
}

bool Packet_Get(void)
{
  // Read Packet values into variables
  while (UART_InChar(&Packet_Checksum))
  {
    // Check if a valid packet
    if ( ((returnChecksum(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3))==Packet_Checksum) && (Packet_Command!=0) )
    {
      return true;
    }
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
  OS_SemaphoreWait(PacketAccess, 0);
  // Send bytes of packet into UART
  if (UART_OutChar(command) &&
    UART_OutChar(parameter1) &&
    UART_OutChar(parameter2) &&
    UART_OutChar(parameter3) &&
    UART_OutChar(returnChecksum(command,parameter1,parameter2,parameter3)))
  {
    //If all packet successfully sent
    OS_SemaphoreSignal(PacketAccess);
    return true;
  }

  // If all bytes are not successfully written to UART
  OS_SemaphoreSignal(PacketAccess);
  return false;
}
/*!
 * @}
 */

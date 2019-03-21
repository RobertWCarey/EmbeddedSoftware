/*! @file
 *
 *  @brief I/O routines for UART communications on the TWR-K70F120M.
 *
 *  This contains the functions for operating the UART (serial port).
 *
 *  @author Robert Carey and Harry Mace
 *  @date 2018-03-17
 */

// Include Header Files
#include "UART.h"
#include "MK70F12.h"
#include "FIFO.h"

// Declare Transmit and Receive buffers
static TFIFO Tx_Buffer, Rx_Buffer;

bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  // Enable UART2 Clock
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  // Enable PORTE Clock
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // Declare Variable to store baud Rate
  uint16union_t SBR;
  uint8_t BFRA;

  // Declare Variables used to calculate Baud Rate
  uint8_t aquisitionRate = 16;
  uint8_t BFRADivisor = 32;

  // Calculate SBR
  SBR.l = (moduleClk/baudRate)/aquisitionRate;
  // Calculate BRFA (Baud Rate Fine Adjust)
  BFRA = (moduleClk % (baudRate*aquisitionRate))*BFRADivisor/(baudRate*aquisitionRate);

  // Disable the UART2 TX and RX
  UART2_C2 &= ~UART_C2_RE_MASK;
  UART2_C2 &= ~UART_C2_TE_MASK;

  // Write baud rate variables to registers
  UART2_BDH |= UART_BDH_SBR(SBR.s.Hi);
  UART2_BDL = SBR.s.Lo;
  UART2_C4 |= UART_C4_BRFA(BFRA);

  // Mux the UART2 TX & RX pins for PORTE
  PORTE_PCR16 |= PORT_PCR_MUX(3);
  PORTE_PCR17 |= PORT_PCR_MUX(3);

  // Enable the UART2 TX and RX
  UART2_C2 |= UART_C2_RE_MASK;
  UART2_C2 |= UART_C2_TE_MASK;

  // Declare Buffers
  FIFO_Init(&Tx_Buffer);
  FIFO_Init(&Rx_Buffer);

  return true;
}

bool UART_InChar(uint8_t* const dataPtr)
{
  return FIFO_Get(&Rx_Buffer,dataPtr);
}

bool UART_OutChar(const uint8_t data)
{
  if (FIFO_Put(&Tx_Buffer,data)){
      UART2_C2 |= UART_C2_TIE_MASK; // Set Transfer Enable
      return true;
  }else
    return false;
}

void UART_Poll(void)
{
  // Retrieve State of UART2 status register 1
  uint8_t status = UART2_S1;

  // Check if Receive Data Register Full Flag is set
  if (status & UART_S1_RDRF_MASK)
    // Read data from UART into Receive buffer
    FIFO_Put(&Rx_Buffer,UART2_D);

  // Check if Transmit Data Register Empty Flag is set
  if (status & UART_S1_TDRE_MASK)
    //Write data from Transmit buffer
    if (!FIFO_Get(&Tx_Buffer,(uint8_t*)&UART2_D))// If buffer empty
      UART2_C2 &= ~UART_C2_TIE_MASK; // Set Transfer Disable
}
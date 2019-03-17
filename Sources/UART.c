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

bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk){
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

/*! @brief Get a character from the receive FIFO if it is not empty.
 *
 *  @param dataPtr A pointer to memory to store the retrieved byte.
 *  @return bool - TRUE if the receive FIFO returned a character.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_InChar(uint8_t* const dataPtr){

}

/*! @brief Put a byte in the transmit FIFO if it is not full.
 *
 *  @param data The byte to be placed in the transmit FIFO.
 *  @return bool - TRUE if the data was placed in the transmit FIFO.
 *  @note Assumes that UART_Init has been called.
 */
bool UART_OutChar(const uint8_t data){

}

/*! @brief Poll the UART status register to try and receive and/or transmit one character.
 *
 *  @return void
 *  @note Assumes that UART_Init has been called.
 */
void UART_Poll(void){

}

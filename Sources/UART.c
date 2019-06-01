/*!
 * @addtogroup UART_module UART module documentation
 * @{
 */
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

// Declare Transmit and Receive buffers
static TFIFO TxBuffer, RxBuffer;

static uint8_t RxData;

OS_ECB *TxSemaphore; /*!< Binary semaphore for signaling that data transmission */
OS_ECB *RxSemaphore;  /*!< Binary semaphore for signaling receiving of data */

bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  //Create semaphores
  TxSemaphore = OS_SemaphoreCreate(0);
  RxSemaphore = OS_SemaphoreCreate(0);

  // Enable UART2 Clock
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  // Enable PORTE Clock
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // Declare Variable to store baud Rate
  uint16union_t sbr;
  uint8_t brfa;

  // Declare Variables used to calculate Baud Rate
  uint8_t const AQUISITION_RATE = 16;
  uint8_t const BRFA_DIVISOR = 32;

  // Calculate sbr
  sbr.l = (moduleClk/baudRate)/AQUISITION_RATE;
  // Calculate BRFA (Baud Rate Fine Adjust)
  brfa = (moduleClk % (baudRate*AQUISITION_RATE))*BRFA_DIVISOR/(baudRate*AQUISITION_RATE);

  // Disable the UART2 TX and RX
  UART2_C2 &= ~UART_C2_RE_MASK;
  UART2_C2 &= ~UART_C2_TE_MASK;

  // Write baud rate variables to registers
  UART2_BDH |= UART_BDH_SBR(sbr.s.Hi);
  UART2_BDL = UART_BDL_SBR(sbr.s.Lo);
  UART2_C4 |= UART_C4_BRFA(brfa);

  // Mux the UART2 TX & RX pins for PORTE
  PORTE_PCR16 |= PORT_PCR_MUX(3);
  PORTE_PCR17 |= PORT_PCR_MUX(3);

  // Enable the UART2 TX and RX
  UART2_C2 |= UART_C2_RE_MASK;
  UART2_C2 |= UART_C2_TE_MASK;

  // Declare Buffers
  FIFO_Init(&TxBuffer);
  FIFO_Init(&RxBuffer);

  //Initialise NVIC for UART2 RX TX
  //Vector=65, IRQ=49, non-IPR=1
  //clear any pending interrupts at UART2
  NVICICPR1 = (1 << 17);
  //Enable interrupts from UART2
  NVICISER1 = (1 << 17);

  // Enable UART RIE
  UART2_C2 |= UART_C2_RIE_MASK;

  return true;
}


void UARTRxThread(void* pData)
{
  for(;;)
  {
    //wait for RXFIFO
    OS_SemaphoreWait(RxSemaphore,0);
//    OS_DisableInterrupts(); // Entering critical section
    FIFO_Put(&RxBuffer, RxData);
//    OS_EnableInterrupts();
  }
}

void UARTTxThread(void* pData)
{
  for(;;)
  {
    //wait for Txfifo
    OS_SemaphoreWait(TxSemaphore,0);
    FIFO_Get(&TxBuffer,(uint8_t*)&UART2_D);

  }
}


bool UART_InChar(uint8_t* const dataPtr)
{
  return FIFO_Get(&RxBuffer,dataPtr);
}

bool UART_OutChar(const uint8_t data)
{
  OS_DisableInterrupts(); // Entering critical section
  if (FIFO_Put(&TxBuffer,data))
    {
      UART2_C2 |= UART_C2_TIE_MASK;
      OS_EnableInterrupts(); // Exiting critical section
      return true;
    }

  OS_EnableInterrupts(); // Exiting critical section
  return false;
}


void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();

  // Retrieve State of UART2 status register 1
  uint8_t status = UART2_S1;

  // Check if Receive Data Register Full Flag is set
  if (status & UART_S1_RDRF_MASK)
    // Read data from UART into Receive buffer
    OS_SemaphoreSignal(RxSemaphore);
    RxData = UART2_D;
//    FIFO_Put(&RxBuffer,UART2_D);

  // Check if Transmit Data Register Empty Flag is set
  if ((status & UART_S1_TDRE_MASK) && (UART2_C2 & UART_C2_TIE_MASK))
    {
      //Write data from Transmit buffer
//      if (!FIFO_Get(&TxBuffer,(uint8_t*)&UART2_D))
      OS_SemaphoreSignal(TxSemaphore);

      UART2_C2 &= ~UART_C2_TIE_MASK;
    }

  OS_ISRExit();
}


/*!
 * @}
 */

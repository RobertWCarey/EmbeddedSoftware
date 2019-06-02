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

// Variable to store current data in data reg
// Used so that flag can be cleared in ISR
static uint8_t RxData;

static OS_ECB *TxSemaphore;   /*!< Incrementing semaphore for signaling data transmission */
static OS_ECB *RxSemaphore;   /*!< Incrementing semaphore for signaling receiving of data */

void UARTRxThread(void* pData)
{
  for(;;)
  {
    //wait for ISR to trigger semaphore to indicate data has been received
    OS_SemaphoreWait(RxSemaphore,0);
    // Place received data into receiving buffer
    FIFO_Put(&RxBuffer, RxData);
  }
}

void UARTTxThread(void* pData)
{
  for(;;)
  {
    //wait for ISR to trigger semaphore to indicate data is ready to be sent
    OS_SemaphoreWait(TxSemaphore,0);

    // Check if transmit is ready
    // Prevents unsuccessful writes/transmits
    if (UART2_S1 & UART_S1_TDRE_MASK)
    {
      // Check if there are still items in the transmit buffer
      if(FIFO_Get(&TxBuffer,(uint8_t*)&UART2_D))
        // Re-enable transmission interrupt
        UART2_C2 |= UART_C2_TIE_MASK;
    }
    else
      UART2_C2 |= UART_C2_TIE_MASK;
  }
}

bool UART_Init(const TUARTSetup* const UARTSetup)
{
  // assigning values to local constants
  const uint32_t baudRate = UARTSetup->baudRate;
  const uint32_t moduleClk = UARTSetup->moduleClk;

  //Create semaphores
  TxSemaphore = OS_SemaphoreCreate(0);
  RxSemaphore = OS_SemaphoreCreate(0);

  // Local variable to store any errors for OS
  OS_ERROR error;

  // Create Transmission thread
  error = OS_ThreadCreate(UARTTxThread,
			  UARTSetup->TxParams->pData,
			  UARTSetup->TxParams->pStack,
			  UARTSetup->TxParams->priority);
  // Create Receive thread
  error = OS_ThreadCreate(UARTRxThread,
  			  UARTSetup->RxParams->pData,
  			  UARTSetup->RxParams->pStack,
  			  UARTSetup->RxParams->priority);

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


bool UART_InChar(uint8_t* const dataPtr)
{
  // Retrieve data from receive buffer
  // Return bool value to indicate success
  return FIFO_Get(&RxBuffer,dataPtr);
}

bool UART_OutChar(const uint8_t data)
{
  OS_DisableInterrupts(); // Entering critical section

  // If data can be placed in transmit buffer
  if (FIFO_Put(&TxBuffer,data))
  {
    //Enable transmit interrupt
    UART2_C2 |= UART_C2_TIE_MASK;
    OS_EnableInterrupts(); // Exiting critical section
    return true;
  }

  OS_EnableInterrupts(); // Exiting critical section
  return false;
}


void __attribute__ ((interrupt)) UART_ISR(void)
{
  OS_ISREnter();// Entering ISR

  // Retrieve State of UART2 status register 1
  uint8_t status = UART2_S1;

  // Check if Transmit Data Register Empty Flag is set
  if(UART2_C2 & UART_C2_TIE_MASK)
  {
    // Check that Data reg is ready to transmit
    if (status & UART_S1_TDRE_MASK)
    {
      // Disable transmit interrupt
      UART2_C2 &= ~UART_C2_TIE_MASK;
      // Signal semaphore to allow UARTTxThread to run
      OS_SemaphoreSignal(TxSemaphore);
    }
  }

  // Check if Receive Data Register Full Flag is set
  if (status & UART_S1_RDRF_MASK)
  {
    // Read data from UART into Receive buffer to clear flag
    RxData = UART2_D;
    // Signal semaphore to allow UARTRxThread to run
    OS_SemaphoreSignal(RxSemaphore);
  }

  OS_ISRExit();// Exiting ISR
}


/*!
 * @}
 */

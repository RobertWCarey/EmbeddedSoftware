/* ###################################################################
**     Filename    : main.c
**     Project     : Lab4
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/

/*!
**  @addtogroup main_module Main module documentation
**  @{
*/
/*!
** @file main.c
** @version 4.0
** @brief
**         Main module.
**         This module contains user's application code.
*/
/* MODULE main */

#include "DOR.h"
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
#include "ComProt.h"
#include "OS.h"

// Pointers to non-volatile storage locations
volatile uint8_t *nvIDMTCharacter;      // Pointer to IDMT Characteristic
volatile uint16union_t *nvTimesTripped; // Pointer to Number of Times Tripped
volatile uint8_t *nvFaultType;          // Pointer to Most recent Fault Type

// Serial Baud Rate (bps)
static const uint32_t BAUD_RATE = 115200;

// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 1024

// Enumeration to store thread priorities
typedef enum
{
  InitModulesThreadPriority,
  UARTRxThreadPriority,
  DORPhaseATimingPriority,
  DORPhaseBTimingPriority,
  DORPhaseCTimingPriority,
  DORTripPriority,
  PacketThreadPriority,
  UARTTxThreadPriority,
} TThreadPriority;

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);     /*!< The stack for the Init Modules thread. */
OS_THREAD_STACK(UARTRxThreadStack, THREAD_STACK_SIZE);          /*!< The stack for the UART receive thread. */
OS_THREAD_STACK(DORPhaseATimingThreadStack, THREAD_STACK_SIZE); /*!< The stack for the DOR Phase A Timing thread. */
OS_THREAD_STACK(DORPhaseBTimingThreadStack, THREAD_STACK_SIZE); /*!< The stack for the DOR Phase B Timing thread. */
OS_THREAD_STACK(DORPhaseCTimingThreadStack, THREAD_STACK_SIZE); /*!< The stack for the DOR Phase C Timing thread. */
OS_THREAD_STACK(DORTripThreadStack, THREAD_STACK_SIZE);         /*!< The stack for the DOR Trip thread. */
OS_THREAD_STACK(PacketHandleThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the Packet Handle thread. */
OS_THREAD_STACK(UARTTxThreadStack, THREAD_STACK_SIZE);          /*!< The stack for the UART transmit thread. */

// Thread Parameters
// Initilisation of modules thread parameters
TOSThreadParams InitModulesThreadParams = {NULL,&InitModulesThreadStack[THREAD_STACK_SIZE - 1],InitModulesThreadPriority};
// UART receive thread parameters
TOSThreadParams UART_RxThreadParams = {NULL,&UARTRxThreadStack[THREAD_STACK_SIZE - 1],UARTRxThreadPriority};
// DOR Phase A Timing thread params for one channel
TOSThreadParams DOR_PhaseATimingThreadParams = {NULL,&DORPhaseATimingThreadStack[THREAD_STACK_SIZE - 1],DORPhaseATimingPriority};
// DOR Phase B Timing thread params for one channel
TOSThreadParams DOR_PhaseBTimingThreadParams = {NULL,&DORPhaseBTimingThreadStack[THREAD_STACK_SIZE - 1],DORPhaseBTimingPriority};
// DOR Phase C Timing thread params for one channel
TOSThreadParams DOR_PhaseCTimingThreadParams = {NULL,&DORPhaseCTimingThreadStack[THREAD_STACK_SIZE - 1],DORPhaseCTimingPriority};
// DOR Trip thread params
TOSThreadParams DOR_TripThreadParams = {NULL,&DORTripThreadStack[THREAD_STACK_SIZE - 1],DORTripPriority};
// UART transmit thread parameters
TOSThreadParams UART_TxThreadParams = {NULL,&UARTTxThreadStack[THREAD_STACK_SIZE - 1],UARTTxThreadPriority};
// Packet Handle thread parameters
TOSThreadParams PacketHandleThreadParams = {NULL,&PacketHandleThreadStack[THREAD_STACK_SIZE - 1],PacketThreadPriority};


/*! @brief Initialises the modules to support the Tower modules.
 *
 *  @param pData is used to store the FTM0 Channel0 data.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  //Default settings
  const uint8_t defaultIDMTCharacter = IDMT_V_INVERSE;
  const uint16_t defaultTimesTripped = 0;
  const uint8_t defaultFaultType = 0;

  //Packet setup struct
  TPacketSetup packetSetup;
  packetSetup.UARTBaudRate = BAUD_RATE;
  packetSetup.UARTModuleClk = CPU_BUS_CLK_HZ;
  packetSetup.UARTTxParams = &UART_TxThreadParams;
  packetSetup.UARTRxParams = &UART_RxThreadParams;

  //Disable Interrupts
  OS_DisableInterrupts();

  //Initialise Modules
  Packet_Init(&packetSetup); // Init Packet Module
  Flash_Init(); // Init Flash Module

  //Assign non-volatile memory locations
  Flash_AllocateVar((void*)&nvTimesTripped, sizeof(*nvTimesTripped));
  Flash_AllocateVar((void*)&nvIDMTCharacter, sizeof(*nvIDMTCharacter));
  Flash_AllocateVar((void*)&nvFaultType, sizeof(*nvFaultType));
  if (*nvIDMTCharacter == 0xff)
      Flash_Write8((uint8_t*)nvIDMTCharacter, defaultIDMTCharacter);
  if (nvTimesTripped->l == 0xffff)
        Flash_Write16((uint16_t*)nvTimesTripped, defaultTimesTripped);
  if (*nvFaultType == 0xff)
        Flash_Write8((uint8_t*)nvFaultType, defaultFaultType);

  //DOR Module Setup & Init
  TDORSetup dorSetup;
  dorSetup.moduleClk = CPU_BUS_CLK_HZ;
  dorSetup.PhaseAParams = &DOR_PhaseATimingThreadParams;
  dorSetup.PhaseBParams = &DOR_PhaseBTimingThreadParams;
  dorSetup.PhaseCParams = &DOR_PhaseCTimingThreadParams;
  TDORTripThreadData dorTripThreadData;
  dorTripThreadData.characteristic = nvIDMTCharacter;
  dorTripThreadData.timesTripped = nvTimesTripped;
  dorTripThreadData.faultType = nvFaultType;
  DOR_TripThreadParams.pData = &dorTripThreadData;
  dorSetup.TripParams = &DOR_TripThreadParams;
  DOR_Init(&dorSetup);

  //Send start-up packet
  towerStatupPacketHandler(nvIDMTCharacter);

  //Enable Interrupts
  OS_EnableInterrupts();

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Checks for any valid packets and then handles them.
 *
 *  @param pData is used to store the FTM0 Channel0 data.
 */
static void PacketHandleThread(void* pData)
{
  for (;;)
  {
    // Check if a packet is available
    if (Packet_Get())
      // Deal with any received packets
      cmdHandler(nvIDMTCharacter, nvTimesTripped, nvFaultType);
  }
}


/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
  // Local variable to store any errors for OS
  OS_ERROR error;


  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */

  // Initialize the RTOS - with flashing the orange LED "heartbeat"
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
			  InitModulesThreadParams.pData,
			  InitModulesThreadParams.pStack,
			  InitModulesThreadParams.priority); // Highest priority

  // Create the packet handling thread
  error = OS_ThreadCreate(PacketHandleThread, // Lowest priority
			  PacketHandleThreadParams.pData,
			  PacketHandleThreadParams.pStack,
			  PacketHandleThreadParams.priority);

  // Start multithreading - never returns!
  OS_Start();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/

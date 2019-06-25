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

// CPU module - contains low level hardware initialization routines
#include "DOR.h"
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
#include "ComProt.h"
#include "OS.h"


// Pointers to non-volatile storage locations
volatile uint16union_t *nvTowerNb;
volatile uint16union_t *nvTowerMode;

// Baud Rate (bps)
static const uint32_t BAUD_RATE = 115200;

// Global value for storing IDMT Characteristic
TIDMTCharacter IDMT_Characteristic = IDMT_V_INVERSE;

// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 1024

// Enumeration to store thread priorities
typedef enum
{
  InitModulesThreadPriority,
  UARTRxThreadPriority,
  DORTiming0Priority,
  DORTiming1Priority,
  DORTiming2Priority,
  DORTripPriority,
  PacketThreadPriority,
  UARTTxThreadPriority,
} TThreadPriority;

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);   /*!< The stack for the Init Modules thread. */
OS_THREAD_STACK(UARTRxThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the UART receive thread. */
OS_THREAD_STACK(UARTTxThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the UART transmit thread. */
OS_THREAD_STACK(DORTiming0ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(DORTiming1ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(DORTiming2ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(DORTripThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketHandleThreadStack, THREAD_STACK_SIZE);  /*!< The stack for the Packet Handle thread. */

// Thread Parameters
// Initilisation of modules thread parameters
TOSThreadParams InitModulesThreadParams = {NULL,&InitModulesThreadStack[THREAD_STACK_SIZE - 1],InitModulesThreadPriority};
// Analog thread params for one channel
TOSThreadParams DOR_Timing0ThreadParams = {NULL,&DORTiming0ThreadStack[THREAD_STACK_SIZE - 1],DORTiming0Priority};
// Analog thread params for one channel
TOSThreadParams DOR_Timing1ThreadParams = {NULL,&DORTiming1ThreadStack[THREAD_STACK_SIZE - 1],DORTiming1Priority};
// Analog thread params for one channel
TOSThreadParams DOR_Timing2ThreadParams = {NULL,&DORTiming2ThreadStack[THREAD_STACK_SIZE - 1],DORTiming2Priority};
// Trip thread params
TOSThreadParams DOR_TripThreadParams = {&IDMT_Characteristic,&DORTripThreadStack[THREAD_STACK_SIZE - 1],DORTripPriority};
// UART receive thread parameters
TOSThreadParams UART_RxThreadParams = {NULL,&UARTRxThreadStack[THREAD_STACK_SIZE - 1],UARTRxThreadPriority};
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
  const uint16_t defaultTowerNb = 9382; //Tower Mode no.
  const uint16_t defaultTowerMode = 1; //Tower Version no.

  //Packet setup struct
  TPacketSetup packetSetup;
  packetSetup.UARTBaudRate = BAUD_RATE;
  packetSetup.UARTModuleClk = CPU_BUS_CLK_HZ;
  packetSetup.UARTTxParams = &UART_TxThreadParams;
  packetSetup.UARTRxParams = &UART_RxThreadParams;

  //DOR Module Setup
  TDORSetup dorSetup;
  dorSetup.moduleClk = CPU_BUS_CLK_HZ;
  dorSetup.Channel0Params = &DOR_Timing0ThreadParams;
  dorSetup.Channel1Params = &DOR_Timing1ThreadParams;
  dorSetup.Channel2Params = &DOR_Timing2ThreadParams;
  dorSetup.TripParams = &DOR_TripThreadParams;

  //Disable Interrupts
  OS_DisableInterrupts();

  //Initialise Modules
  Packet_Init(&packetSetup);
  LEDs_Init();
  Flash_Init();
  DOR_Init(&dorSetup);

  //Assign non-volatile memory locations
  Flash_AllocateVar((void*)&nvTowerNb, sizeof(*nvTowerNb));
  Flash_AllocateVar((void*)&nvTowerMode, sizeof(*nvTowerMode));
  if (nvTowerNb->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerNb, defaultTowerNb);
  if (nvTowerMode->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerMode, defaultTowerMode);

  //Send start-up packet
  towerStatupPacketHandler(nvTowerNb,nvTowerMode);

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
      cmdHandler(nvTowerNb,nvTowerMode,&IDMT_Characteristic);
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

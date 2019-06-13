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
#include "RTC.h"
#include "FTM.h"
#include "accel.h"
#include "median.h"
#include "ComProt.h"
#include "OS.h"


// Pointers to non-volatile storage locations
volatile uint16union_t *nvTowerNb;
volatile uint16union_t *nvTowerMode;

// Baud Rate (bps)
static const uint32_t BAUD_RATE = 115200;

//Accelerometer mode global
static TAccelMode AccelMode = ACCEL_POLL;
//Accelerometer latest data
static TAccelData AccelData;
//Last three values for each accelerometer axis
static uint8_t XValues[3], YValues[3], ZValues[3];

// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 1024

// Enumeration to store thread priorities
typedef enum
{
  InitModulesThreadPriority,
  UARTRxThreadPriority,
  PacketThreadPriority,
  UARTTxThreadPriority,
  DORChannel0Priority,
  RTCThreadPriority,
  FTMThreadPriority
} TThreadPriority;

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);   /*!< The stack for the Init Modules thread. */
OS_THREAD_STACK(UARTRxThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the UART receive thread. */
OS_THREAD_STACK(UARTTxThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the UART transmit thread. */
OS_THREAD_STACK(DORChannel0ThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);           /*!< The stack for the RTC thread. */
OS_THREAD_STACK(FTMThreadStack, THREAD_STACK_SIZE);           /*!< The stack for the FTM thread. */
OS_THREAD_STACK(PacketHandleThreadStack, THREAD_STACK_SIZE);  /*!< The stack for the Packet Handle thread. */

// Thread Parameters
// Initilisation of modules thread parameters
TOSThreadParams InitModulesThreadParams = {NULL,&InitModulesThreadStack[THREAD_STACK_SIZE - 1],InitModulesThreadPriority};
// UART receive thread parameters
TOSThreadParams UART_RxThreadParams = {NULL,&UARTRxThreadStack[THREAD_STACK_SIZE - 1],UARTRxThreadPriority};
// UART transmit thread parameters
TOSThreadParams UART_TxThreadParams = {NULL,&DORChannel0ThreadStack[THREAD_STACK_SIZE - 1],UARTTxThreadPriority};
// Analog thread params for one channel
TOSThreadParams DOR_Channel0ThreadParams = {NULL,&UARTTxThreadStack[THREAD_STACK_SIZE - 1],DORChannel0Priority};
// Real Time Clock thread parameters
TOSThreadParams RTC_ThreadParams = {NULL,&RTCThreadStack[THREAD_STACK_SIZE - 1],RTCThreadPriority};
// Flexible Timer Module thread parameters
TOSThreadParams FTM_ThreadParams = {NULL,&FTMThreadStack[THREAD_STACK_SIZE - 1],FTMThreadPriority};
// Packet Handle thread parameters
TOSThreadParams PacketHandleThreadParams = {NULL,&PacketHandleThreadStack[THREAD_STACK_SIZE - 1],PacketThreadPriority};

/*! @brief Interrupt callback function to be called when RTC_ISR occurs
 * Turn on yellow LED and send the time
 *  @param arg The user argument that comes with the callback
 */
void RTCCallback(void* arg)
{
  //toggle yellow LED
  LEDs_Toggle(LED_YELLOW);
  //send the current time
  static Time time;
  RTC_Get(&time.hours,&time.minutes,&time.seconds);
  Packet_Put(CMD_TIME_BYTE,time.hours,time.minutes,time.seconds);
}

/*! @brief Interrupt callback function to be called when FTM_ISR occurs (output compare match)
 * Turn off blue LED
 *  @param arg The user argument that comes with the callback
 */
void FTMCallback(void* arg)
{
  //turn off blue LED
  LEDs_Off(LED_BLUE);
}


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

  //RTC setup struct
  TRTCSetup rtcSetup;
  rtcSetup.CallbackFunction = RTCCallback;
  rtcSetup.CallbackArguments = NULL;
  rtcSetup.ThreadParams = &RTC_ThreadParams;

  //DOR Module Setup
  TDORSetup dorSetup;
  dorSetup.moduleClk = CPU_BUS_CLK_HZ;
  dorSetup.Channel0Params = &DOR_Channel0ThreadParams;

  //Disable Interrupts
  OS_DisableInterrupts();

  //Initialise Modules
  Packet_Init(&packetSetup);
  LEDs_Init();
  RTC_Init(&rtcSetup);
  FTM_Init(&FTM_ThreadParams);
  Flash_Init();
  DOR_Init(&dorSetup);

  //Set FTM Timer
  FTM_Set(pData);

  //Assign non-volatile memory locations
  Flash_AllocateVar((void*)&nvTowerNb, sizeof(*nvTowerNb));
  Flash_AllocateVar((void*)&nvTowerMode, sizeof(*nvTowerMode));
  if (nvTowerNb->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerNb, defaultTowerNb);
  if (nvTowerMode->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerMode, defaultTowerMode);

  //Send start-up packet
  towerStatupPacketHandler(nvTowerNb,nvTowerMode,&AccelMode);

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
      cmdHandler(nvTowerNb,nvTowerMode,pData,&AccelMode);
  }
}


/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
  // Local variable to store any errors for OS
  OS_ERROR error;

  //Configure struct for FTM_Set()
  static TFTMChannel channelSetup0;
  channelSetup0.channelNb = 0;
  channelSetup0.delayCount = 1 * CPU_MCGFF_CLK_HZ_CONFIG_0; // Frequency of Fixed Frequency clock
  channelSetup0.timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE;
  channelSetup0.ioType.inputDetection = TIMER_INPUT_OFF;
  channelSetup0.ioType.outputAction = TIMER_OUTPUT_DISCONNECT; // triggers channel interrupt
  channelSetup0.callbackFunction = FTMCallback;
  channelSetup0.callbackArguments = NULL;

  // Store FTM0 Channel0 data into thread data for later use
  PacketHandleThreadParams.pData = &channelSetup0;
  InitModulesThreadParams.pData = &channelSetup0;

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

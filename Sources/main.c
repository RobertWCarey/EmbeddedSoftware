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

// Pit time period (nano seconds)
static const uint32_t PIT_TIME_PERIOD = 1000e6;

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
  UARTTxThreadPriority,
  I2CThreadPriority,
  AccelThreadPriority,
  PITThreadPriority,
  RTCThreadPriority,
  FTMThreadPriority,
  PacketThreadPriority
} TThreadPriority;

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketHandleThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTRxThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the UART receive thread. */
OS_THREAD_STACK(UARTTxThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the UART transmit thread. */
OS_THREAD_STACK(I2CThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the UART transmit thread. */
OS_THREAD_STACK(AccelThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PITThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(FTMThreadStack, THREAD_STACK_SIZE);

// Thread Parameters
// Initilisation of modules thread parameters
TOSThreadParams InitModulesThreadParams = {NULL,&InitModulesThreadStack[THREAD_STACK_SIZE - 1],InitModulesThreadPriority};
// UART receive thread parameters
TOSThreadParams UART_RxThreadParams = {NULL,&UARTRxThreadStack[THREAD_STACK_SIZE - 1],UARTRxThreadPriority};
// UART transmit thread parameters
TOSThreadParams UART_TxThreadParams = {NULL,&UARTTxThreadStack[THREAD_STACK_SIZE - 1],UARTTxThreadPriority};
// I2C thread parameters
TOSThreadParams I2C_ThreadParams = {NULL,&I2CThreadStack[THREAD_STACK_SIZE - 1],I2CThreadPriority};
// Accelerometer thread parameters
TOSThreadParams Accel_ThreadParams = {NULL,&AccelThreadStack[THREAD_STACK_SIZE - 1],AccelThreadPriority};
// Periodic Interrupt Timer thread parameters
TOSThreadParams PIT_ThreadParams = {NULL,&PITThreadStack[THREAD_STACK_SIZE - 1],PITThreadPriority};
// Real Time Clock thread parameters
TOSThreadParams RTC_ThreadParams = {NULL,&RTCThreadStack[THREAD_STACK_SIZE - 1],RTCThreadPriority};
// Flexible Timer Module thread parameters
TOSThreadParams FTM_ThreadParams = {NULL,&FTMThreadStack[THREAD_STACK_SIZE - 1],FTMThreadPriority};
// Packet Handle thread parameters
TOSThreadParams PacketHandleThreadParams = {NULL,&PacketHandleThreadStack[THREAD_STACK_SIZE - 1],PacketThreadPriority};

/*! @brief Interrupt callback function to be called when PIT_ISR occurs
 *
 *  @param arg The user argument that comes with the callback
 */
void PITCallback(void* arg)
{
  if (AccelMode == ACCEL_POLL)
    //Read values
    Accel_ReadXYZ(AccelData.bytes);
}

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

/*! @brief Interrupt callback function to be called when Accelerometer
 * @brief data ready interrupt occours, Synchronous mode
 *
 *  @param arg The user argument that comes with the callback
 */
void AccelDataReadyCallback(void* arg)
{
  if (AccelMode == ACCEL_INT)
    //Read values
    Accel_ReadXYZ(AccelData.bytes);
}

/*! @brief shifts each value over one position in the array and loads new val
 *
 *  @param array[3] The array with values to be shifted
 *  @param value    The new value to be loaded into the array
 */
static void shiftVals(uint8_t array[3], uint8_t value)
{
  array[2] = array[1];
  array[1] = array[0];
  array[0] = value;
}

/*! @brief I2C0 callback function, outputs the accelerometer values
 *
 *   @param arg The user argument that comes with the callback
 */
void I2CCallback(void* arg)
{
  // Static variable to store the previously read Accel Data
  static TAccelData prevAccelData;

  // Load in the latest values
  shiftVals(XValues,AccelData.axes.x);
  shiftVals(YValues,AccelData.axes.y);
  shiftVals(ZValues,AccelData.axes.z);

  //Check if the values have change || in sychronous mode
  if ( (AccelMode == ACCEL_POLL &&  (prevAccelData.bytes != AccelData.bytes)) ||  AccelMode == ACCEL_INT)
  {
    // Send last median values regardless of changing
    Packet_Put(CMD_ACCEL_VAL,
       Median_Filter3(XValues[0], XValues[1], XValues[2]),
       Median_Filter3(YValues[0], YValues[1], YValues[2]),
       Median_Filter3(ZValues[0], ZValues[1], ZValues[2]));
  }

  LEDs_Toggle(LED_GREEN);

  // Store current value
  prevAccelData = AccelData;
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

  //Accelerometer setup struct
  TAccelSetup accelSetup;
  accelSetup.moduleClk = CPU_BUS_CLK_HZ;
  accelSetup.dataReadyCallbackArguments = NULL;
  accelSetup.dataReadyCallbackFunction = AccelDataReadyCallback;
  accelSetup.readCompleteCallbackArguments = NULL;
  accelSetup.readCompleteCallbackFunction = I2CCallback;
  accelSetup.I2CThreadParams = &I2C_ThreadParams;
  accelSetup.ThreadParams = &Accel_ThreadParams;

  //Packet setup struct
  TPacketSetup packetSetup;
  packetSetup.UARTBaudRate = BAUD_RATE;
  packetSetup.UARTModuleClk = CPU_BUS_CLK_HZ;
  packetSetup.UARTTxParams = &UART_TxThreadParams;
  packetSetup.UARTRxParams = &UART_RxThreadParams;

  //PIT setup struct
  TPITSetup pitSetup;
  pitSetup.moduleClk = CPU_BUS_CLK_HZ;
  pitSetup.CallbackFunction = PITCallback;
  pitSetup.CallbackArguments = NULL;
  pitSetup.ThreadParams = &PIT_ThreadParams;

  //RTC setup struct
  TRTCSetup rtcSetup;
  rtcSetup.CallbackFunction = RTCCallback;
  rtcSetup.CallbackArguments = NULL;
  rtcSetup.ThreadParams = &RTC_ThreadParams;

  //Disable Interrupts
  OS_DisableInterrupts();

  //Initilise Modules
  Packet_Init(&packetSetup);
  LEDs_Init();
  PIT_Init(&pitSetup);
  RTC_Init(&rtcSetup);
  FTM_Init(&FTM_ThreadParams);
  Accel_Init(&accelSetup);
  Flash_Init();

  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true);

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

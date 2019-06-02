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

const uint16_t defaultTowerNb = 9382;
const uint16_t defaultTowerMode = 1;
volatile uint16union_t *nvTowerNb;
volatile uint16union_t *nvTowerMode;

// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 1024

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketHandleThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(UARTRxThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the UART receive thread. */
OS_THREAD_STACK(UARTTxThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the UART transmit thread. */


// Baud Rate
static const uint32_t BAUD_RATE = 115200;

// Pit time period (nano seconds)
static const uint32_t PIT_TIME_PERIOD = 1000e6;

//Accelerometer mode global
static TAccelMode AccelMode = ACCEL_POLL;

//Accelerometer latest data
static TAccelData AccelData;
//Last three values for each accelerometer axis
static uint8_t XValues[3], YValues[3], ZValues[3];



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
//TFTMChannel channelSetup0;
//Configure struct for FTM_Set()
 TFTMChannel channelSetup0 = {0,
			      (1 * CPU_MCGFF_CLK_HZ_CONFIG_0),
			      TIMER_FUNCTION_OUTPUT_COMPARE,
//			      TIMER_INPUT_OFF,
			      TIMER_OUTPUT_DISCONNECT,
			      FTMCallback,
			      NULL};
//    channelSetup0.channelNb = 0;
//    channelSetup0.delayCount = 1 * CPU_MCGFF_CLK_HZ_CONFIG_0; // Frequency of Fixed Frequency clock
//    channelSetup0.timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE;
//    channelSetup0.ioType.inputDetection = TIMER_INPUT_OFF;
//    channelSetup0.ioType.outputAction = TIMER_OUTPUT_DISCONNECT; // triggers channel interrupt
//    channelSetup0.callbackFunction = FTMCallback;
//    channelSetup0.callbackArguments = NULL;


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
  static TAccelData prevAccelData;

  shiftVals(XValues,AccelData.axes.x);
  shiftVals(YValues,AccelData.axes.y);
  shiftVals(ZValues,AccelData.axes.z);

  if ( (AccelMode == ACCEL_POLL &&  (prevAccelData.bytes != AccelData.bytes)) ||  AccelMode == ACCEL_INT)
    {
      // Send last median values regardless of changing
      Packet_Put(CMD_ACCEL_VAL,
         Median_Filter3(XValues[0], XValues[1], XValues[2]),
         Median_Filter3(YValues[0], YValues[1], YValues[2]),
         Median_Filter3(ZValues[0], ZValues[1], ZValues[2]));
    }


  LEDs_Toggle(LED_GREEN);

  prevAccelData = AccelData;
}


/*! @brief Runs all functions necessary for Tower to function.
 *
 *  @return bool - TRUE if all components successfully initialized.
 */
static bool towerInit(void)
{

}


/*! @brief Initialises the modules to support the LEDs and low power timer.
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  //Accelerometer setup struct
  TAccelSetup accelSetup;
  accelSetup.moduleClk = CPU_BUS_CLK_HZ;
  accelSetup.dataReadyCallbackArguments = NULL;
  accelSetup.dataReadyCallbackFunction = AccelDataReadyCallback;
  accelSetup.readCompleteCallbackArguments = NULL;
  accelSetup.readCompleteCallbackFunction = I2CCallback;

  //Configure struct for FTM_Set()
//  TFTMChannel channelSetup0;
//  channelSetup0.channelNb = 0;
//  channelSetup0.delayCount = 1 * CPU_MCGFF_CLK_HZ_CONFIG_0; // Frequency of Fixed Frequency clock
//  channelSetup0.timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE;
//  channelSetup0.ioType.inputDetection = TIMER_INPUT_OFF;
//  channelSetup0.ioType.outputAction = TIMER_OUTPUT_DISCONNECT; // triggers channel interrupt
//  channelSetup0.callbackFunction = FTMCallback;
//  channelSetup0.callbackArguments = NULL;

  OS_DisableInterrupts();//Disable Interrupts
  Packet_Init(BAUD_RATE,CPU_BUS_CLK_HZ);
  LEDs_Init();
  PIT_Init(CPU_BUS_CLK_HZ, PITCallback, NULL);
  RTC_Init(RTCCallback,NULL);
  FTM_Init();
  Accel_Init(&accelSetup);
  Flash_Init();


  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true);

  //Set FTM Timer
  FTM_Set(&channelSetup0);

  Flash_AllocateVar((void*)&nvTowerNb, sizeof(*nvTowerNb));
  Flash_AllocateVar((void*)&nvTowerMode, sizeof(*nvTowerMode));
  if (nvTowerNb->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerNb, defaultTowerNb);
  if (nvTowerMode->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerMode, defaultTowerMode);

  towerStatupPacketHandler(nvTowerNb,nvTowerMode,&AccelMode);


  OS_EnableInterrupts();//Enable Interrupts

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

static void PacketHandleThread(void* pData)
{
  for (;;)
  {
    if (Packet_Get())
      // Deal with any received packets
      cmdHandler(nvTowerNb,nvTowerMode,&channelSetup0,&AccelMode);
  }
}



/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;
  /* Write your local variable definition here */
//  uint16_t defaultTowerNb = 9382;
//  uint16_t defaultTowerMode = 1;
//  volatile uint16union_t *nvTowerNb;
//  volatile uint16union_t *nvTowerMode;
//
//  //Configure struct for FTM_Set()
//  TFTMChannel channelSetup0;
//  channelSetup0.channelNb = 0;
//  channelSetup0.delayCount = 1 * CPU_MCGFF_CLK_HZ_CONFIG_0; // Frequency of Fixed Frequency clock
//  channelSetup0.timerFunction = TIMER_FUNCTION_OUTPUT_COMPARE;
//  channelSetup0.ioType.inputDetection = TIMER_INPUT_OFF;
//  channelSetup0.ioType.outputAction = TIMER_OUTPUT_DISCONNECT; // triggers channel interrupt
//  channelSetup0.callbackFunction = FTMCallback;
//  channelSetup0.callbackArguments = NULL;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */

  // Initialize the RTOS - with flashing the orange LED "heartbeat"
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
		          0); // Highest priority
  error = OS_ThreadCreate(
			  UARTTxThread,
			  NULL,
			  &UARTTxThreadStack[THREAD_STACK_SIZE - 1],
			  2);
  error = OS_ThreadCreate(
			  UARTRxThread,
			  NULL,
			  &UARTRxThreadStack[THREAD_STACK_SIZE - 1],
			  1);
  error = OS_ThreadCreate(PacketHandleThread, // Lowest priority
  			  NULL,
  			  &PacketHandleThreadStack[THREAD_STACK_SIZE - 1],
  			  7);



  // Start multithreading - never returns!
  OS_Start();

//  // Initialize Tower
//  __DI();
//  if (towerInit())
//  {
//    LEDs_On(LED_ORANGE);
//    Flash_AllocateVar((void*)&nvTowerNb, sizeof(*nvTowerNb));
//    Flash_AllocateVar((void*)&nvTowerMode, sizeof(*nvTowerMode));
//    if (nvTowerNb->l == 0xffff)
//      Flash_Write16((uint16_t*)nvTowerNb, defaultTowerNb);
//    if (nvTowerMode->l == 0xffff)
//      Flash_Write16((uint16_t*)nvTowerMode, defaultTowerMode);
//
//    towerStatupPacketHandler(nvTowerNb,nvTowerMode);
//  }
//  __EI();
//
//  //Set PIT Timer
//  PIT_Set(PIT_TIME_PERIOD, true);
//
//  //Set FTM Timer
//  FTM_Set(&channelSetup0);


//  for (;;)
//  {
//
//    // Check if any valid Packets have been received
//    if (Packet_Get())
//      // Deal with any received packets
//      cmdHandler(nvTowerNb,nvTowerMode,&channelSetup0);
//
//
//  }

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

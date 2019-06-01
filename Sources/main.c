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

// Command Values
#define CMD_TOWER_STARTUP 0x04u
#define CMD_SPECIAL_TOWER_VERSION 0x09u
#define CMD_TOWER_NUMBER 0x0Bu
#define CMD_PROGRAM_BYTE 0x07u
#define CMD_READ_BYTE 0x08u
#define CMD_TOWER_MODE 0x0Du
#define CMD_TIME_BYTE 0xCu
#define CMD_ACCEL_VAL 0x10u
#define CMD_PROT_MODE 0x0Au

// Parameters for 0x04-Tower Startup
static const uint8_t TOWER_STARTUP_PARAM = 0x00;

// Parameters for 0x09-Special-Tower Version
static const uint8_t TOWER_SPECIAL_V = 0x76; // ASCII "v" in Hex
static const uint8_t TOWER_SPECIAL_X = 0x78; // ASCII "x" in Hex
static const uint8_t TOWER_VERSION_MAJOR = 0x01;
static const uint8_t TOWER_VERSION_MINOR = 0x00;

// Parameters for 0x0B-Tower Number
static const uint8_t TOWER_NUMBER_GET = 0x01;// Get Param
static const uint8_t TOWER_NUMBER_SET = 0x02;// Set Param

// Parameters for 0x0D-Tower Mode
static const uint8_t TOWER_MODE_GET = 0x01;// Get Param
static const uint8_t TOWER_MODE_SET = 0x02;// Set Param

// Parameters for 0x07-Program Byte
static const uint8_t PROGRAM_BYTE_ERASE = 0x08;// Erase Sector Param
static const uint8_t PROGRAM_BYTE_RANGE_LO = 0x00;// Lowest valid value Param
static const uint8_t PROGRAM_BYTE_RANGE_HI = 0x08;// Highest valid value Param

// Parameters for 0x08-Read Byte
static const uint8_t READ_BYTE_RANGE_LO = 0x00;// Lowest valid value Param
static const uint8_t READ_BYTE_RANGE_HI = 0x07;// Highest valid value Param

// Parameters for 0x0C-Time
static const uint8_t TIME_RANGE_LO = 0x00;// Lowest valid value
static const uint8_t TIME_HOURS_RANGE_HI = 23;// Highest hours valid value
static const uint8_t TIME_MINUTES_RANGE_HI = 59;// Highest minutes valid value
static const uint8_t TIME_SECONDS_RANGE_HI = 59;// Highest seconds valid value

// Parameters for 0x0A - Protocol Mode
static const uint8_t PROT_MODE_GET = 1;// Get Param
static const uint8_t PROT_MODE_SET = 2;// Set Param
static const uint8_t PROT_MODE_ASYNC = 0;// Asynchronous mode (PIT polling)
static const uint8_t PROT_MODE_SYNC = 1;// Synchronous mode (Accel interrupt)
static const uint8_t PROT_MODE_PARAM3 = 0;// Parameter 3 for 0x0A

// Pit time period (nano seconds)
static const uint32_t PIT_TIME_PERIOD = 1000e6;

//Accelerometer mode global
static TAccelMode AccelMode = ACCEL_POLL;

//Accelerometer latest data
static TAccelData AccelData;
//Last three values for each accelerometer axis
static uint8_t XValues[3], YValues[3], ZValues[3];

/*! @brief Sends out required packets for Tower Startup.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if all packets were successfully sent.
 */
static bool towerStatupPacketHandler (volatile uint16union_t * const towerNb,volatile uint16union_t * const towerMode)
{
  // Check that params are valid
  if ( (Packet_Parameter1 == TOWER_STARTUP_PARAM) && (Packet_Parameter2 == TOWER_STARTUP_PARAM) && (Packet_Parameter3 == TOWER_STARTUP_PARAM) )
    return Packet_Put(CMD_TOWER_STARTUP,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM,TOWER_STARTUP_PARAM);// &&
//      Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR) &&
//      Packet_Put(CMD_TOWER_NUMBER,TOWER_NUMBER_GET,towerNb->s.Lo,towerNb->s.Hi)&&
//      Packet_Put(CMD_TOWER_MODE,TOWER_MODE_GET,towerMode->s.Lo,towerMode->s.Hi)&&
//      Packet_Put(CMD_PROT_MODE, 0, AccelMode, 0);

  // If invalid params return false
  return false;
}

/*! @brief Returns values for Special packet requested.
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool specialPacketHandler()
{
  // Check which special selected
  if ( (Packet_Parameter1 == TOWER_SPECIAL_V) && (Packet_Parameter2 == TOWER_SPECIAL_X) )// If Get Version selected
    return Packet_Put(CMD_SPECIAL_TOWER_VERSION,TOWER_SPECIAL_V,TOWER_VERSION_MAJOR,TOWER_VERSION_MINOR);

  return false;
}

/*! @brief Handles Tower Number packets.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if packet successfully sent.
 */
static bool towerNumberPacketHandler(volatile uint16union_t * const towerNb)
{
  // Check which type of Tower Number Packet
  // If Get
  if ( (Packet_Parameter1 == TOWER_NUMBER_GET) && !(Packet_Parameter2) && !(Packet_Parameter3) )
    // Send out Tower Number packet
    return Packet_Put(CMD_TOWER_NUMBER,TOWER_NUMBER_GET,towerNb->s.Lo,towerNb->s.Hi);
  else if (Packet_Parameter1 == TOWER_NUMBER_SET) // If Set
  {
    //Update Tower Number Values
    return Flash_Write16((uint16_t*)towerNb,Packet_Parameter23);
  }

  return false;
}

/*! @brief Handles Tower Number packets.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return bool - TRUE if packet successfully sent.
 */
static bool towerModePacketHandler(volatile uint16union_t * const towerMode)
{
  // Check which type of Tower Number Packet
  // If Get
  if ( (Packet_Parameter1 == TOWER_MODE_GET) && !(Packet_Parameter2) && !(Packet_Parameter3) )
    // Send out Tower Number packet
    return Packet_Put(CMD_TOWER_MODE,TOWER_MODE_GET,towerMode->s.Lo,towerMode->s.Hi);
  else if (Packet_Parameter1 == TOWER_NUMBER_SET) // If Set
  {
    //Update Tower Number Values
    return Flash_Write16((uint16_t*)towerMode,Packet_Parameter23);
  }

  return false;
}

/*! @brief Executes Program byte.
 *
 *  @return bool - TRUE if data successfully programmed.
 */
static bool prgmBytePacketHandler()
{
  // Check offset is valid, and parameter byte is valid
  if ((Packet_Parameter1 < PROGRAM_BYTE_RANGE_LO) || (Packet_Parameter1 > PROGRAM_BYTE_RANGE_HI) || (Packet_Parameter2))
    return false;
  // Check if erase sector has been requested
  if (Packet_Parameter1 == PROGRAM_BYTE_ERASE)
    return Flash_Erase();
  // Write data to selected address offset
  return Flash_Write8((uint8_t*)(FLASH_DATA_START+Packet_Parameter1),Packet_Parameter3);

}

/*! @brief Executes Read byte.
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool readBytePacketHandler()
{
  // Check offset is valid, and parameter byte is valid
  if ((Packet_Parameter1 < READ_BYTE_RANGE_LO) || (Packet_Parameter1 > READ_BYTE_RANGE_HI) || (Packet_Parameter2) || (Packet_Parameter3))
    return false;

  return Packet_Put(CMD_READ_BYTE, Packet_Parameter1, 0, _FB(FLASH_DATA_START+Packet_Parameter1));

}

/*! @brief sets the time for the RTC.
 *
 *  @return bool - TRUE if data successfully programmed.
 */
static bool timePacketHandler()
{
  // Check lower values for valid range
  if ((Packet_Parameter1 >= TIME_RANGE_LO) && (Packet_Parameter2 >= TIME_RANGE_LO) && (Packet_Parameter3 >= TIME_RANGE_LO))
    //Check upper values for valid range
    if ((Packet_Parameter1 <= TIME_HOURS_RANGE_HI) && (Packet_Parameter2 <= TIME_MINUTES_RANGE_HI) && (Packet_Parameter3 <= TIME_SECONDS_RANGE_HI))
      {
	// Update the current RTC value
	RTC_Set(Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
	return true;
      }

  return false;
}

/*! @brief Executes Protocol Mode packet handler.
 *
 *  @return bool - TRUE if packet successfully sent.
 */
static bool protModePacketHandler()
{
  // Check offset is valid, and parameter byte is valid
  if ((Packet_Parameter3 != PROT_MODE_PARAM3))
    return false;

  // Check for Valid get parameters
  if ( (Packet_Parameter1 == PROT_MODE_GET) && !(Packet_Parameter2) )
    {
      // Send current mode
      return Packet_Put(CMD_PROT_MODE, PROT_MODE_GET, AccelMode, PROT_MODE_PARAM3);
    }
  // Check for valid Set parameters
  else if (Packet_Parameter1 == PROT_MODE_SET)
    {
      //Set mode based on packet_parameter2 as long as valid
      if ((Packet_Parameter2 == PROT_MODE_ASYNC) || (Packet_Parameter2 == PROT_MODE_SYNC))
	{
	  // Update accelerometer mode
	  Accel_SetMode(Packet_Parameter2);
	  // Update variable storing current mode
	  AccelMode = Packet_Parameter2;
	  return true;
	}
    }

  return false;
}

/*! @brief Performs necessary action for any valid packets received.
 *
 *  @param towerNb A variable containing the Tower Number.
 *  @return void.
 */
static void cmdHandler(volatile uint16union_t * const towerNb, volatile uint16union_t * const towerMode, const TFTMChannel* const aFTMChannel)
{
  //Starts a timer and turns on LED
//  FTM_StartTimer(aFTMChannel);
  LEDs_On(LED_BLUE);

  // Isolate command packet
  uint8_t command = Packet_Command & ~PACKET_ACK_MASK;
  // Isolate ACK request
  uint8_t ack = Packet_Command & PACKET_ACK_MASK;
  // Variable to record if action was successful
  bool success = 0;

  switch (command)
  {
    case CMD_TOWER_STARTUP:
      success = towerStatupPacketHandler(towerNb,towerMode);
      break;
    case CMD_SPECIAL_TOWER_VERSION:
      success = specialPacketHandler();
      break;
    case CMD_TOWER_NUMBER:
      success = towerNumberPacketHandler(towerNb);
      break;
    case CMD_TOWER_MODE:
      success = towerModePacketHandler(towerMode);
      break;
    case CMD_PROGRAM_BYTE:
      success = prgmBytePacketHandler();
      break;
    case CMD_READ_BYTE:
      success = readBytePacketHandler();
      break;
    case CMD_TIME_BYTE:
      success = timePacketHandler();
      break;
    case CMD_PROT_MODE:
      success = protModePacketHandler();
      break;
    default:
      break;
  }

  // Check if ACK Requested
  if (ack)
    // Check if action was successful
    if (success)// If success send same packet
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    else // If !success send packet with NACK
    {
      // Send Nack if required
      uint8_t nackCommand = Packet_Command & ~PACKET_ACK_MASK;
      Packet_Put(nackCommand,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }

  // Reset Packet variables
  Packet_Command = 0x00u;
  Packet_Parameter1 = 0x00u;
  Packet_Parameter2 = 0x00u;
  Packet_Parameter3 = 0x00u;
  Packet_Checksum = 0x00u;
}

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
//      Packet_Put(CMD_ACCEL_VAL,0x64,0x64,0x64);
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

  OS_DisableInterrupts();//Disable Interrupts
  Packet_Init(BAUD_RATE,CPU_BUS_CLK_HZ);
  LEDs_Init();
  PIT_Init(CPU_BUS_CLK_HZ, PITCallback, NULL);
//  RTC_Init(RTCCallback,NULL);
//  FTM_Init();
  Accel_Init(&accelSetup);
  Flash_Init();


  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true);

  Flash_AllocateVar((void*)&nvTowerNb, sizeof(*nvTowerNb));
  Flash_AllocateVar((void*)&nvTowerMode, sizeof(*nvTowerMode));
  if (nvTowerNb->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerNb, defaultTowerNb);
  if (nvTowerMode->l == 0xffff)
    Flash_Write16((uint16_t*)nvTowerMode, defaultTowerMode);

  towerStatupPacketHandler(nvTowerNb,nvTowerMode);


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
      cmdHandler(nvTowerNb,nvTowerMode,&channelSetup0);
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

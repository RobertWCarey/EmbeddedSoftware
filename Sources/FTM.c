/*!
 * @addtogroup FTM_module FTM module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for setting up the FlexTimer module (FTM) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the FlexTimer module (FTM).
 *
 *  @author Robert Carey & Harry Mace
 *  @date 2019-05-06
 */


// new types
#include "types.h"
#include "FTM.h"
#include "MK70F12.h"
#include "CPU.h"
#include "OS.h"

//Addresses for function that will store user callback function
//define an array for multiple channel functions and arguments
static void (*UserFunction[8])(void*);
static void* UserArguments[8];

static bool Execute[8];

#define THREAD_STACK_SIZE 1024
OS_THREAD_STACK(FTMThreadStack, THREAD_STACK_SIZE);

static OS_ECB *FTMSemaphore;


bool FTM_Init()
{
  OS_ERROR error;
  FTMSemaphore = OS_SemaphoreCreate(0);

  error = OS_ThreadCreate(FTMThread,
		      NULL,
		      &FTMThreadStack[THREAD_STACK_SIZE - 1],
		      8);

  // Enable FTM0 in Clock gate
  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

  // Disable the clock
  FTM0_SC |= FTM_SC_CLKS(0);

  // Disable Timer overflow interrupt
  FTM0_SC |= ~FTM_SC_TOIE_MASK;

  // FTM to up counting mode
  FTM0_SC &= ~FTM_SC_CPWMS_MASK;

  // Count initial
  FTM0_CNTIN = FTM_CNTIN_INIT(0);

  // Mod value (overflow)
  FTM0_MOD = FTM_MOD_MOD_MASK;

  // FTM_CNT for counting
  FTM0_CNT = FTM_CNT_COUNT(0);

  // Enable FTM clock source
  // Write to Status and Control in FTM0. 0d2 = 0b10 = Fixed Frequency Clock
  FTM0_SC = FTM_SC_CLKS(2);

  //Set up interrupt vector
  //Vector=, IRQ=62, non-IPR=1
  //clear any pending interrupts at RTC_seconds
  NVICICPR1 = (1 << 30);
  //Enable interrupts from RTC_seconds
  NVICISER1 = (1 << 30);

  return true;
}

bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
  //Channel interrupt enable
  FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_CHIE_MASK;

  // TimerFunction = 0 if INPUT CAPTURE, 1 if OUTPUT COMPARE
  FTM0_CnSC(aFTMChannel->channelNb) |= aFTMChannel->timerFunction << FTM_CnSC_MSA_SHIFT;

  // Check timer function:
  if (aFTMChannel->timerFunction == TIMER_FUNCTION_INPUT_CAPTURE) // 0 = Input Capture
  {
    // Pass the input detection config into ELSB:ELSA
    FTM0_CnSC(aFTMChannel->channelNb) |=  aFTMChannel->ioType.inputDetection << FTM_CnSC_ELSA_SHIFT;
  }
  else if (aFTMChannel->timerFunction == TIMER_FUNCTION_OUTPUT_COMPARE) // 1 = Output Compare
  {
    // Pass the output action config into ELSB:ELSA (Edge or Level Select)
    FTM0_CnSC(aFTMChannel->channelNb) |=  aFTMChannel->ioType.outputAction << FTM_CnSC_ELSA_SHIFT;
  }

  return true;
}

bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{
  // Read the current counter value of FTM0
  uint16_t currentCnt = (FTM0_CNT & FTM_CNT_COUNT_MASK); // Mask to ensure only "COUNT" value read

  //Add the delay to the current Count to get channel value CnV which will cause the interrupt when cnt values match
  //FTM_MOD_MOD_MASK is to make sure if the sum is greater than the 16-bit it raps around
  FTM0_CnV(aFTMChannel->channelNb) = FTM_CnV_VAL((currentCnt + aFTMChannel->delayCount) % (FTM_MOD_MOD_MASK));

  //Initialise local versions for userFunction and userArgument
  UserFunction[aFTMChannel->channelNb] = aFTMChannel->callbackFunction;
  UserArguments[aFTMChannel->channelNb] = aFTMChannel->callbackArguments;

  return true;
}

void FTMThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(FTMSemaphore,0);
//    sizeof (data)/sizeof (data[0])
    for (int i = 0; i <= 7; i++)
      {
	if (Execute[i])
	  if (UserFunction[i])
	    (*UserFunction[i])(UserArguments[i]);
      }

  }
}

void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  uint8_t statusReg0 = FTM0_C0SC;

  // Clear the CHF and execute callback function;
  for (int channelNb = 0; channelNb <= 7; channelNb++)
  {
    if (FTM0_CnSC(channelNb) & FTM_CnSC_CHF_MASK)
    {
      FTM0_CnSC(channelNb) &= ~FTM_CnSC_CHF_MASK;
      Execute[channelNb] = true;
      OS_SemaphoreSignal(FTMSemaphore);
//      if (UserFunction[channelNb])
//	(*UserFunction[channelNb])(UserArguments[channelNb]);
    }
  }

}

/*!
 * @}
 */

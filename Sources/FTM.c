/*! @file
 *
 *  @brief Routines for setting up the FlexTimer module (FTM) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the FlexTimer module (FTM).
 *
 *  @author PMcL
 *  @date 2015-09-04
 */


// new types
#include "types.h"
#include "FTM.h"
#include "MK70F12.h"

//Addresses for function that will store user callback function
//define an array for multiple channel functions and arguments
static void (*UserFunction[8])(void*);
static void* UserArguments[8];


/*! @brief Sets up the FTM before first use.
 *
 *  Enables the FTM as a free running 16-bit counter.
 *  @return bool - TRUE if the FTM was successfully initialized.
 */
bool FTM_Init()
{
  // Enable FTM0 in Clock gate
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

    // Disable the clock
    FTM0_SC |= FTM_SC_CLKS(0);

    // Count initial
    FTM0_CNTIN = FTM_CNTIN_INIT(0);

    // Mod value (overflow)
    FTM0_MOD = FTM_MOD_MOD_MASK;

    // FTM_CNT for counting
    FTM0_CNT = FTM_CNT_COUNT(0);

    // Write to Status and Control in FTM0. 0d2 = 0b10 = Fixed Frequency Clock
    FTM0_SC |= FTM_SC_CLKS(2);

    //Set up interrupt vector
    //Vector=, IRQ=62, non-IPR=1
    //clear any pending interrupts at RTC_seconds
    NVICICPR1 = (1 << 30);
    //Enable interrupts from RTC_seconds
    NVICISER1 = (1 << 30);

    return true;
}

/*! @brief Sets up a timer channel.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *    channelNb is the channel number of the FTM to use.
 *    delayCount is the delay count (in module clock periods) for an output compare event.
 *    timerFunction is used to set the timer up as either an input capture or an output compare.
 *    ioType is a union that depends on the setting of the channel as input capture or output compare:
 *      outputAction is the action to take on a successful output compare.
 *      inputDetection is the type of input capture detection.
 *    callbackFunction is a pointer to a user callback function.
 *    callbackArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the timer was set up successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
  //Channel interrupt enable
    FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_CHIE_MASK;
    // TimerFunction = 0 if INPUT CAPTURE, 1 if OUTPUT COMPARE
    FTM0_CnSC(aFTMChannel->channelNb) |= aFTMChannel->timerFunction << FTM_CnSC_MSB_SHIFT;
    // Check timer function:
    // 0 = Input Capture
    if(aFTMChannel->timerFunction == 0)
    {
      // Pass the input detection config into ELSB:ELSA
      FTM0_CnSC(aFTMChannel->channelNb) |=  aFTMChannel->ioType.inputDetection << FTM_CnSC_ELSA_SHIFT;
    }
    // 1 = Output Compare
    else if(aFTMChannel->timerFunction == 1)
    {
      // Pass the output action config into ELSB:ELSA
      FTM0_CnSC(aFTMChannel->channelNb) |=  aFTMChannel->ioType.outputAction << FTM_CnSC_ELSA_SHIFT;
    }
}


/*! @brief Starts a timer if set up for output compare.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *  @return bool - TRUE if the timer was started successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{
  // Read the current counter value of FTM0
    uint16_t currentCnt = (FTM0_CNT & FTM_CNT_COUNT_MASK);

    //Add the delay to the current Count to get channel value CnV which will cause the interrupt when cnt values match
    FTM0_CnV(aFTMChannel->channelNb) = FTM_CnV_VAL((currentCnt + aFTMChannel->delayCount) % (FTM_MOD_MOD_MASK));
    //Initialise local versions for userFunction and userArgument
    UserFunction[aFTMChannel->channelNb] = aFTMChannel->callbackFunction;
    UserArguments[aFTMChannel->channelNb] = aFTMChannel->callbackArguments;
    //Execute the callback function
    if(aFTMChannel->callbackFunction)
      (*aFTMChannel->callbackFunction)(aFTMChannel->callbackArguments);
    return true;
}


/*! @brief Interrupt service routine for the FTM.
 *
 *  If a timer channel was set up as output compare, then the user callback function will be called.
 *  @note Assumes the FTM has been initialized.
 */
void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  uint8_t statusReg0 = FTM0_C0SC;

    // Clear the CHF and execute callback function;
    for(int channelNb = 0; channelNb <= 7; channelNb++)
    {
      if(FTM0_CnSC(channelNb) & FTM_CnSC_CHF_MASK)
      {
        FTM0_CnSC(channelNb) &= (~FTM_CnSC_CHF_MASK);
        if(UserFunction[channelNb])
  	  (*UserFunction[channelNb])(UserArguments[channelNb]);
      }
    }
}

/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Rob Carey & Harrison Mace
 *  @date 2019-05-02
 */

#include "PIT.h"
#include "MK70F12.h"
#include "types.h"
#include "CPU.h"

//PIT Module Clock Period in ns
static uint8_t pitClkPeriod;

static void (*UserFunction)(void*);
static void* UserArguments;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{
  //Initialise local versions for userFunction and userArgument
  UserFunction = userFunction;
  UserArguments = userArguments;

  // Enable PIT Clock
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

  //Calculate clk period based on modlueClk value
  pitClkPeriod = ((1e9)/moduleClk);

  //Enable standard PIT timers
  PIT_Enable(true);

  //Freeze timer when Debugging
  PIT_MCR = PIT_MCR_FRZ_MASK;

  //Initialise NVIC for PIT Channel0
  //Vector=84, IRQ=68, non-IPR=2
  //clear any pending interrupts at PIT0
  NVICICPR2 = (1 << 4);
  //Enable interrupts from PIT0
  NVICISER2 = (1 << 4);


  return true;
}

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t period, const bool restart)
{
  if (restart)
    {
      PIT_Enable(0);
      PIT_LDVAL0 = period;
      PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
      PIT_Enable(1);
    }
  else
    PIT_LDVAL0 = period;//Number of ticks to be loaded
  uint32_t nbTicks = (period/pitClkPeriod)-1;

  if(restart)
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
  }

  //Load value
  PIT_LDVAL0 = PIT_LDVAL_TSV(nbTicks);

  //Clear Timer Interrupt
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
  //Enable PIT Timer and Interrupt
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
  PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
}

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable)
{
  if (enable)
    PIT_MCR |= PIT_MCR_MDIS_MASK;
  else
    PIT_MCR &= ~PIT_MCR_MDIS_MASK;
}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT_ISR(void)
{
  EnterCritical();
    //Clear Timer Interrupt
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
    // Call user function
    if(UserFunction)
      (*UserFunction)(UserArguments);
    ExitCritical();
}


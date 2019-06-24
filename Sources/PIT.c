/*!
 * @addtogroup PIT_module PIT module documentation
 * @{
 */
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


// Local global to store PIT Module Clock Period in ns
static uint8_t PITClkPeriod;

// Local globals to store callback function
static void (*UserFunction[2])(void*);
static void* UserArguments[2];

static OS_ECB* PITSemaphore[2]; /*!< Incrementing semaphore for PITThread execution */

bool PIT_Init(const TPITSetup* const PITSetup)
{
  // Local variable to store any errors for OS
  OS_ERROR error;

  for (int i=0;i<2;i++)
  {
    //Create semaphore
    PITSemaphore[i] = PITSetup->Semaphore[i];
    //Initialise local versions for userFunction and userArgument
    UserFunction[i] = PITSetup->CallbackFunction[i];
    UserArguments[i] = PITSetup->CallbackArguments[i];
  }


  if (PITSetup->EnablePITThread)
  {
    // Create PIT thread
    error = OS_ThreadCreate(PITThread,
          PITSetup->ThreadParams->pData,
          PITSetup->ThreadParams->pStack,
          PITSetup->ThreadParams->priority);
  }

  // Enable PIT Clock
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

  //Calculate clk period based on modlueClk value
  PITClkPeriod = ((1e9)/PITSetup->moduleClk);

  //Enable standard PIT timers
  //Must be done before any setup
  PIT_Enable(true);

  //Freeze timer when Debugging
  PIT_MCR = PIT_MCR_FRZ_MASK;

  //Initialise NVIC for PIT Channel0
  //Vector=84, IRQ=68, non-IPR=2
  //clear any pending interrupts at PIT0
  NVICICPR2 = (1 << 4);
  //Enable interrupts from PIT0
  NVICISER2 = (1 << 4);
  //clear any pending interrupts at PIT0
  NVICICPR2 = (1 << 5);
  //Enable interrupts from PIT0
  NVICISER2 = (1 << 5);

  return true;
}

void PIT_Set(const uint32_t period, const bool restart, const uint8_t PIT)
{
  uint32_t nbTicks = (period/PITClkPeriod)-1;

  if (restart)
    //Disable PIT
    PIT_TCTRL(PIT) &= ~PIT_TCTRL_TEN_MASK;

  //Load value
  PIT_LDVAL(PIT) = PIT_LDVAL_TSV(nbTicks);

  //Clear Timer Interrupt
  PIT_TFLG(PIT) |= PIT_TFLG_TIF_MASK;

  //Enable PIT Timer and Interrupt
  PIT_TCTRL(PIT) |= PIT_TCTRL_TIE_MASK;
  PIT_TCTRL(PIT) |= PIT_TCTRL_TEN_MASK;
}

void PIT_Enable(const bool enable)
{
  if (enable)
    PIT_MCR &= ~PIT_MCR_MDIS_MASK;// Set MDIS = 0 to enable PIT module
  else
    PIT_MCR |= PIT_MCR_MDIS_MASK;// Set MDIS = 1 to disable module
}

void PITThread(void* pData)
{
  for (;;)
  {
    //wait for ISR to trigger semaphore to indicate loaded time has elapsed
    OS_SemaphoreWait(PITSemaphore,0);

  }
}

void __attribute__ ((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();


  for (int i = 0;i<2;i++)
  {
    if (PIT_TFLG(i) & PIT_TFLG_TIF_MASK)
      {
        //Clear Timer Interrupt
        PIT_TFLG(i) |= PIT_TFLG_TIF_MASK;

        // Execute the passed callback function
        if (UserFunction[i])
          (*UserFunction[i])(UserArguments[i]);
        //Signal semaphore to indicate loaded time has elapsed
        if (!i)
          OS_SemaphoreSignal(PITSemaphore[i]);
      }
  }





  OS_ISRExit();
}

/*!
 * @}
 */

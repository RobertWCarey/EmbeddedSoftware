/*!
 * @addtogroup RTC_module RTC module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for controlling the Real Time Clock (RTC) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the real time clock (RTC).
 *
 *  @author Robert Carey & Harry Mace
 *  @date 2019-05-06
 */


// new types

#include "RTC.h"


static void (*UserFunction)(void*);
static void* UserArguments;

#define THREAD_STACK_SIZE 1024
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);

static OS_ECB *RTCSemaphore;

bool RTC_Init(void (*userFunction)(void*), void* userArguments)
{
  OS_ERROR error;
  RTCSemaphore = OS_SemaphoreCreate(0);

  error = OS_ThreadCreate(RTCThread,NULL,&RTCThreadStack[THREAD_STACK_SIZE - 1],6);

  //Initialise local versions for userFunction and userArgument
  UserFunction = userFunction;
  UserArguments = userArguments;

  // Enable RTC clock in clock gate 6
  SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;

  //Disable time count
  RTC_SR &= ~RTC_SR_TCE_MASK;

  //Clear TIF and TOF by writing to TSR
  //Check if TOF or TIF is raised before clearing TSR
  if (RTC_SR & (RTC_SR_TOF_MASK | RTC_SR_TIF_MASK) )
    RTC_TSR = RTC_TSR_TSR(1);// re-setting value back to beginning (not zero as recommended by manual, 47.2.1)

  //Check if load capacitor values have already been set
  if (!(RTC_LR & RTC_LR_CRL_MASK))
    {
      //Disable RTC Oscillator
      RTC_CR &= ~RTC_CR_OSCE_MASK;

      //Set Load Capacitance to 18PF (as per schematic)
      RTC_CR |= RTC_CR_SC2P_MASK | RTC_CR_SC16P_MASK;

      //Lock control register to indicate load capacitor values have already been set
      RTC_LR &= ~RTC_LR_CRL_MASK;

      //Enable RTC Oscillator
      RTC_CR |= RTC_CR_OSCE_MASK;

      //Wait for oscillator to stabilise
      for(uint16_t i = 0; i<0xFFFF;i++);
    }

  // Enable RTC Time Seconds Interrupt
  RTC_IER |= RTC_IER_TSIE_MASK;

  //Enable RTC Time Count
  RTC_SR |= RTC_SR_TCE_MASK;

  //Set up interrupt vector
  //Vector=, IRQ=67, non-IPR=2
  //clear any pending interrupts at RTC_seconds
  NVICICPR2 = (1 << 3);
  //Enable interrupts from RTC_seconds
  NVICISER2 = (1 << 3);

  return true;
}

void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  //Disable RTC Time Count so value can be written
  RTC_SR &= ~RTC_SR_TCE_MASK;

  //write to RTC_TSR
  RTC_TSR = seconds + (minutes*60) + (hours*3600);

  //Enable RTC Time Count
  RTC_SR |= RTC_SR_TCE_MASK;
}

void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  uint32_t tsr = RTC_TSR;
  /* If it is necessary for software to read the prescaler or seconds counter when they could be incrementing,
   * it is recommended that two read accesses are performed and that software verifies that the
   * same data was returned for both reads.
   */
  while ( RTC_TSR != tsr)
    tsr = RTC_TSR;

  tsr = tsr % 86400U;   	/* Seconds left */
  *hours = tsr / 3600U;     	/* Hours */
  tsr = tsr % 3600u;        	/* Seconds left */
  *minutes = tsr / 60U;     	/* Minutes */
  *seconds = tsr % 60U;     	/* Seconds */
}

void RTCThread(void* pData)
{
  for (;;)
  {
    OS_SemaphoreWait(RTCSemaphore,0);

    if (UserFunction)
      (*UserFunction)(UserArguments);

  }
}

void __attribute__ ((interrupt)) RTC_ISR(void)
{
  // Call user function
//  if (UserFunction)
//    (*UserFunction)(UserArguments);
  OS_SemaphoreSignal(RTCSemaphore);
}

/*!
 * @}
 */

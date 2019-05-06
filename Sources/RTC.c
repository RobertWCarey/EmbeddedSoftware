/*! @file
 *
 *  @brief Routines for controlling the Real Time Clock (RTC) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the real time clock (RTC).
 *
 *  @author PMcL
 *  @date 2015-08-24
 */


// new types
#include "types.h"
#include "MK70F12.h"

static void (*UserFunction)(void*);
static void* UserArguments;

/*! @brief Initializes the RTC before first use.
 *
 *  Sets up the control register for the RTC and locks it.
 *  Enables the RTC and sets an interrupt every second.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user argum	ents to use with the user callback function.
 *  @return bool - TRUE if the RTC was successfully initialized.
 */
bool RTC_Init(void (*userFunction)(void*), void* userArguments)
{
  //Initialise local versions for userFunction and userArgument
    UserFunction = userFunction;
    UserArguments = userArguments;

    // Enable RTC clock in clock gate 6
    SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;

    //Disable time count
    RTC_SR &= ~RTC_SR_TCE_MASK;

    //Clear TIF and TOF by writing to TSR
    //Check if TOF or TIF is raised before clearing TSR
    if(RTC_SR & (RTC_SR_TOF_MASK | RTC_SR_TIF_MASK) )
      RTC_TSR = RTC_TSR_TSR(0);

    //Disable RTC Oscillator
    RTC_CR &= ~RTC_CR_OSCE_MASK;

    //Set Load Capacitance to 18PF
    RTC_CR |= RTC_CR_SC2P_MASK;
    RTC_CR |= RTC_CR_SC16P_MASK;

    //Enable RTC Oscillator
    RTC_CR |= RTC_CR_OSCE_MASK;

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

/*! @brief Sets the value of the real time clock.
 *
 *  @param hours The desired value of the real time clock hours (0-23).
 *  @param minutes The desired value of the real time clock minutes (0-59).
 *  @param seconds The desired value of the real time clock seconds (0-59).
 *  @note Assumes that the RTC module has been initialized and all input parameters are in range.
 */
void RTC_Set(const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  //Disable RTC Time Count
   RTC_SR &= ~RTC_SR_TCE_MASK;

   //write to RTC_TSR
   RTC_TSR = seconds + (minutes*60) + (hours*3600);

   //Enable RTC Time Count
   RTC_SR |= RTC_SR_TCE_MASK;
}

/*! @brief Gets the value of the real time clock.
 *
 *  @param hours The address of a variable to store the real time clock hours.
 *  @param minutes The address of a variable to store the real time clock minutes.
 *  @param seconds The address of a variable to store the real time clock seconds.
 *  @note Assumes that the RTC module has been initialized.
 */
void RTC_Get(uint8_t* const hours, uint8_t* const minutes, uint8_t* const seconds)
{
  uint32_t TSR = RTC_TSR;
  TSR = TSR % 86400U;          /* Seconds left */
  *hours = TSR / 3600U;     	/* Hours */
  TSR = TSR % 3600u;           /* Seconds left */
  *minutes = TSR / 60U;     /* Minutes */
  *seconds = TSR % 60U;     /* Seconds */
}

/*! @brief Interrupt service routine for the RTC.
 *
 *  The RTC has incremented one second.
 *  The user callback function will be called.
 *  @note Assumes the RTC has been initialized.
 */
void __attribute__ ((interrupt)) RTC_ISR(void)
{
  // Call user function
    if(UserFunction)
      (*UserFunction)(UserArguments);
}

/*!
 * @addtogroup LEDs_module LEDs module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines to access the LEDs on the TWR-K70F120M.
 *
 *  This contains the functions for operating the LEDs.
 *
 *  @author Harry Mace & Rob Carey
 *  @date 2019-04-01
 */

#include "LEDs.h"
#include "MK70F12.h"
#include "CPU.h"


bool LEDs_Init(void)
{
  // Enable PORTA clock
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // LED pins mux
  PORTA_PCR10 |= PORT_PCR_MUX(1);
  PORTA_PCR11 |= PORT_PCR_MUX(1);
  PORTA_PCR28 |= PORT_PCR_MUX(1);
  PORTA_PCR29 |= PORT_PCR_MUX(1);

  // Set drive strength to high
  PORTA_PCR10 |= PORT_PCR_DSE_MASK;
  PORTA_PCR11 |= PORT_PCR_DSE_MASK;
  PORTA_PCR28 |= PORT_PCR_DSE_MASK;
  PORTA_PCR29 |= PORT_PCR_DSE_MASK;

  // Initialise LEDs as off
  GPIOA_PSOR |= ( LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE );

  // Set ports as outputs
  GPIOA_PDDR |= ( LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE );

  return true;
}

void LEDs_On(const TLED color)
{
  EnterCritical(); // Entering critical section
  // Set logic low to put a voltage across LED
  GPIOA_PCOR = color;
  ExitCritical(); // Exiting critical section
}

void LEDs_Off(const TLED color)
{
  EnterCritical(); // Entering critical section
  // Set logic high to clear voltage across LED
  GPIOA_PSOR = color;
  ExitCritical(); // Exiting critical section
}

void LEDs_Toggle(const TLED color)
{
  EnterCritical(); // Entering critical section
  // Toggle LED
  GPIOA_PTOR = color;
  ExitCritical(); // Exiting critical section
}
/*!
 * @}
 */


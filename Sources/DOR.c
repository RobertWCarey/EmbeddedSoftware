/*!
 * @addtogroup DOR_module DOR module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for the accelerometer
 *
 *  This contains the functions for operating the DOR
 *
 *  @author Robert Carey
 *  @date 2019-06-09
 */

#include "DOR.h"
#include "analog.h"
#include "PIT.h"
#include "FIFO.h"


// Pit time period (nano seconds)
static const uint32_t PIT_TIME_PERIOD = 1250e3;//Sampling 16per cycle at 50Hz

static TFIFO ch0Buffer;

#define NB_ANALOG_CHANNELS 1


/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  }
};

bool DOR_Init(const TDORSetup* const dorSetup)
{
  AnalogThreadData[0].semaphore = OS_SemaphoreCreate(0);

  //PIT setup struct
  TPITSetup pitSetup;
  pitSetup.moduleClk = dorSetup->moduleClk;
  pitSetup.EnablePITThread = 0;
  pitSetup.Semaphore = AnalogThreadData[0].semaphore;

  PIT_Init(&pitSetup);
  Analog_Init(dorSetup->moduleClk);

  // Initiliase fifo
  FIFO_Init(&ch0Buffer);

  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true);

  OS_ERROR error;

  error = OS_ThreadCreate(DOR_Thread,
                          &AnalogThreadData[0],
                          dorSetup->Channel0Params->pStack,
                          dorSetup->Channel0Params->priority);

}

void DOR_Thread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);
  }
}


/*!
 * @}
*/

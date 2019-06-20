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
#include "stdio.h"
#include "stdlib.h"
#include "types.h"
#include "math.h"


// Pit time period (nano seconds)
static const uint32_t PIT_TIME_PERIOD = 1250e3;//Sampling 16per cycle at 50Hz

uint16_t analogInputValue;

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

static void PITCallback(void* arg)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define Data ((TAnalogThreadData*)arg)

  Analog_Get(0, &analogInputValue);
}

bool DOR_Init(const TDORSetup* const dorSetup)
{
  AnalogThreadData[0].semaphore = OS_SemaphoreCreate(0);

  //PIT setup struct
  TPITSetup pitSetup;
  pitSetup.moduleClk = dorSetup->moduleClk;
  pitSetup.EnablePITThread = 0;
  pitSetup.Semaphore = AnalogThreadData[0].semaphore;
  pitSetup.CallbackFunction = PITCallback;
  pitSetup.CallbackArguments = &AnalogThreadData[0];

  PIT_Init(&pitSetup);
  Analog_Init(dorSetup->moduleClk);


  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true);

  OS_ERROR error;

  error = OS_ThreadCreate(DOR_Thread,
                          &AnalogThreadData[0],
                          dorSetup->Channel0Params->pStack,
                          dorSetup->Channel0Params->priority);

}


static float returnRMS(float sampleData[])
{

  float square = 0;
  float volts;
  for (uint8_t i = 0; i<16;i++)
  {
    volts=(float)sampleData[i]/(float)3276;

    square += (volts*volts);
  }

  return (float)sqrt(square/16);
}

void DOR_Thread(void* pData)
{
  #define analogData (*(TAnalogThreadData*)pData)
  int count;
  float vrms;
  for (;;)
  {
    (void)OS_SemaphoreWait(analogData.semaphore, 0);

    analogData.samples[count] = analogInputValue;

    count ++;
    if (count == 16)
    {

      vrms = returnRMS(analogData.samples);

      count = 0;
    }
  }
}


/*!
 * @}
*/

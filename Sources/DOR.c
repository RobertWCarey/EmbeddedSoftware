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

//Output channels
static const uint8_t TIMING_OUPUT_CHANNEL = 1;
static const uint8_t TRIP_OUTPUT_CHANNEL = 2;

static const uint16_t ADC_CONVERSION = 3276;

uint16_t analogInputValue;

#define NB_ANALOG_CHANNELS 1

#define channelData (*(TAnalogThreadData*)pData)


/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData ChannelThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0,
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
  ChannelThreadData[0].semaphore = OS_SemaphoreCreate(0);

  //PIT setup struct
  TPITSetup pitSetup;
  pitSetup.moduleClk = dorSetup->moduleClk;
  pitSetup.EnablePITThread = 0;
  pitSetup.Semaphore = ChannelThreadData[0].semaphore;
  pitSetup.CallbackFunction = PITCallback;
  pitSetup.CallbackArguments = &ChannelThreadData[0];

  PIT_Init(&pitSetup);
  Analog_Init(dorSetup->moduleClk);


  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true);

  OS_ERROR error;

  error = OS_ThreadCreate(DOR_TimingThread,
                          &ChannelThreadData[0],
                          dorSetup->Channel0Params->pStack,
                          dorSetup->Channel0Params->priority);

  error = OS_ThreadCreate(DOR_TimingThread,
                          &ChannelThreadData[0],
                          dorSetup->Channel0Params->pStack,
                          dorSetup->Channel0Params->priority);

  return true;
}


static float returnRMS(int16_t sampleData[])
{

  float square = 0;
  float volts;
  float vrms;
  for (uint8_t i = 0; i<16;i++)
  {
    volts=(float)sampleData[i]/(float)ADC_CONVERSION;

    square += (volts*volts);
  }

  vrms = sqrt(square/16);

  return (float)((vrms*40)/13);
}

static int16_t v2raw(float voltage)
{
  return (int16_t)(voltage*ADC_CONVERSION);
}

void DOR_TimingThread(void* pData)
{
  int count;
  for (;;)
  {
    (void)OS_SemaphoreWait(channelData.semaphore, 0);

    channelData.samples[count] = analogInputValue;

    count ++;
    if (count == 16)
    {
      channelData.irms = returnRMS(channelData.samples);
      count = 0;
    }

    if (channelData.irms > 1.03)
      Analog_Put(TIMING_OUPUT_CHANNEL,v2raw(5));
    else
      Analog_Put(TIMING_OUPUT_CHANNEL,v2raw(0));

  }
}

void DOR_TripThread(void* pData)
{
  for(;;)
  {

  }
}


/*!
 * @}
*/

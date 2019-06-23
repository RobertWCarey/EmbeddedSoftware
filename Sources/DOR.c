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
static const uint8_t TIMING_OUTPUT_CHANNEL = 1;
static const uint8_t TRIP_OUTPUT_CHANNEL = 2;

static const uint16_t ADC_CONVERSION = 3276;

static const TIDMTData INV_TRIP_TIME[20] =
{
  {.y=236746,.x=1.03},{.y=10029,.x=2},{.y=6302,.x=3},{.y=4980,.x=4},{.y=4280,.x=5},
  {.y=3837,.x=6},{.y=3528,.x=7},{.y=3297,.x=8},{.y=3116,.x=9},{.y=2971,.x=10},
  {.y=2850,.x=11},{.y=2748,.x=12},{.y=2660,.x=13},{.y=2583,.x=14},{.y=2516,.x=15},
  {.y=2455,.x=16},{.y=2401,.x=17},{.y=2353,.x=18},{.y=2308,.x=19},{.y=2267,.x=20},
};

static const TIDMTData VINV_TRIP_TIME[20] =
{
  {.y=450000,.x=1.03},{.y=13500,.x=2},{.y=6750,.x=3},{.y=4500,.x=4},{.y=3375,.x=5},
  {.y=2700,.x=6},{.y=2250,.x=7},{.y=1929,.x=8},{.y=1688,.x=9},{.y=1500,.x=10},
  {.y=1350,.x=11},{.y=1227,.x=12},{.y=1125,.x=13},{.y=1038,.x=14},{.y=964,.x=15},
  {.y=900,.x=16},{.y=844,.x=17},{.y=794,.x=18},{.y=750,.x=19},{.y=711,.x=20},
};

static const TIDMTData EINV_TRIP_TIME[20] =
{
  {.y=1313629,.x=1.03},{.y=26667,.x=2},{.y=10000,.x=3},{.y=5333,.x=4},{.y=3333,.x=5},
  {.y=2286,.x=6},{.y=1667,.x=7},{.y=1270,.x=8},{.y=1000,.x=9},{.y=808,.x=10},
  {.y=667,.x=11},{.y=559,.x=12},{.y=476,.x=13},{.y=410,.x=14},{.y=357,.x=15},
  {.y=314,.x=16},{.y=278,.x=17},{.y=248,.x=18},{.y=222,.x=19},{.y=201,.x=20},
};


uint16_t analogInputValue;

#define NB_ANALOG_CHANNELS 3

#define channelData (*(TAnalogThreadData*)pData)


static OS_ECB* TripSemaphore;


/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData ChannelThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0,
    .timerStatus = 0,
    .currentTimeCount = 0,
  },
  {
    .semaphore = NULL,
    .channelNb = 1,
    .timerStatus = 0,
    .currentTimeCount = 0,
  },
  {
    .semaphore = NULL,
    .channelNb = 2,
    .timerStatus = 0,
    .currentTimeCount = 0,
  }
};

static void PIT0Callback(void* arg)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer

  for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    Analog_Get(0, &ChannelThreadData[i].sample);
  }

}

static void PIT1Callback(void* arg)
{
  for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    if (ChannelThreadData[i].timerStatus)
    {
      ChannelThreadData[i].currentTimeCount++;
    }
  }

}

bool DOR_Init(const TDORSetup* const dorSetup)
{
  ChannelThreadData[0].semaphore = OS_SemaphoreCreate(0);

  TripSemaphore = OS_SemaphoreCreate(0);


  //PIT setup struct
  TPITSetup pitSetup;
  pitSetup.moduleClk = dorSetup->moduleClk;
  pitSetup.EnablePITThread = 0;
  pitSetup.Semaphore[0] = ChannelThreadData[0].semaphore;
  pitSetup.Semaphore[1] = TripSemaphore;
  pitSetup.CallbackFunction[0] = PIT0Callback;
  pitSetup.CallbackArguments[0] = NULL;
  pitSetup.CallbackFunction[1] = PIT1Callback;
  pitSetup.CallbackArguments[1] = NULL;

  PIT_Init(&pitSetup);
  Analog_Init(dorSetup->moduleClk);


  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true,0);
  PIT_Set(PIT_TIME_PERIOD, true,1);

  OS_ERROR error;

  error = OS_ThreadCreate(DOR_TimingThread,
                          &ChannelThreadData[0],
                          dorSetup->Channel0Params->pStack,
                          dorSetup->Channel0Params->priority);

  error = OS_ThreadCreate(DOR_TripThread,
                          &ChannelThreadData[0],
                          dorSetup->TripParams->pStack,
                          dorSetup->TripParams->priority);

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

    channelData.samples[count] = channelData.sample;

    count ++;
    if (count == 16)
    {
      channelData.irms = returnRMS(channelData.samples);
      count = 0;
    }

    if (channelData.irms > 1.03)
    {
      Analog_Put(TIMING_OUTPUT_CHANNEL,v2raw(5));
      channelData.timerStatus = 1;
      channelData.currentTimeCount = 0;
    }
    else
    {
      Analog_Put(TIMING_OUTPUT_CHANNEL,v2raw(0));
      channelData.timerStatus = 0;
    }

  }
}

static uint32_t interpolate(TIDMTData data[], double val)
{
  double result = 0; // Initialize result

  for (int i=1; i<20; i++)
  {
      // Compute individual terms of above formula
      double term = data[i].y;
      double temp = data[i].x;
      for (int j=1;j<20;j++)
      {
          double temp = data[j].x;
          if (j!=i)
          {
            temp = (val - data[j].x);
            temp = temp/(data[i].x - data[j].x);
            term = term*temp;
          }
//              term = term*((val - data[j].x)/(data[i].x - data[j].x));
      }

      // Add current term to result
      result += term;
  }

  return (uint32_t)result;
}

void DOR_TripThread(void* pData)
{
  for(;;)
  {
    (void)OS_SemaphoreWait(TripSemaphore, 0);

    // TODO: Add multi channel functionality
//    for (int i = 0; i < NB_ANALOG_CHANNELS; i++)

    //check if timer started
    if (channelData.timerStatus)
    {
      channelData.tripTime = interpolate(INV_TRIP_TIME,channelData.irms);
      if (channelData.currentTimeCount >= channelData.tripTime)
      {
        // Set Output high
        Analog_Put(TRIP_OUTPUT_CHANNEL,v2raw(5));
      }
      else
      {
        // Set Output low
        Analog_Put(TRIP_OUTPUT_CHANNEL,v2raw(0));
      }
    }

  }
}


/*!
 * @}
*/

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
// Todo: change to proper case for vairable
static uint32_t PIT_TIME_PERIOD = 1250e3;//Sampling 16per cycle at 50Hz
//static uint32_t PIT_TIME_PERIOD = 1e6;//Sampling 16per cycle at 50Hz
static uint32_t PIT1_TIME_PERIOD = 1000000; // 1ms

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

#define NB_ANALOG_CHANNELS 1

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
    .offset1 = 0,
    .offset2 = 0,
    .numberOfSamples = 0,
    .crossing = 0,
  },
//  {
//    .semaphore = NULL,
//    .channelNb = 1,
//    .timerStatus = 0,
//    .currentTimeCount = 0,
//    .offset1 = 0,
//    .offset2 = 0,
//    .numberOfSamples = 0,
//    .crossing = 0,
//  },
//  {
//    .semaphore = NULL,
//    .channelNb = 2,
//    .timerStatus = 0,
//    .currentTimeCount = 0,
//    .offset1 = 0,
//    .offset2 = 0,
//    .numberOfSamples = 0,
//    .crossing = 0,
//  }
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
  pitSetup.Semaphore[1] = NULL;
  pitSetup.CallbackFunction[0] = PIT0Callback;
  pitSetup.CallbackArguments[0] = NULL;
  pitSetup.CallbackFunction[1] = PIT1Callback;
  pitSetup.CallbackArguments[1] = NULL;

  PIT_Init(&pitSetup);
  Analog_Init(dorSetup->moduleClk);


  //Set PIT Timer
  PIT_Set(PIT_TIME_PERIOD, true,0);
  PIT_Set(PIT1_TIME_PERIOD, true,1);

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


static float returnRMS(float sampleData[])
{

  float square = 0;
  float volts;
  float vrms;
  for (uint8_t i = 0; i<16;i++)
  {
    volts=sampleData[i];

    square += (volts*volts);
  }

  vrms = sqrt(square/16);

  return (float)((vrms*40)/13);
}

static int16_t v2raw(float voltage)
{
  return (int16_t)(voltage*ADC_CONVERSION);
}

static float raw2v(int16_t voltage)
{
  return (float)voltage/(float)ADC_CONVERSION;
}

static float getOffset(float sample0, float sample1)
{
  float m = (sample1-sample0)/1;//Always one becuase 0ne sample diff
  return (-sample0)/m;
}

static void getFrequency(TAnalogThreadData* Data, uint8_t count)
{

  if (count > 0 )
  {
    // check at full cycle zero crossing
    if (Data->samples[count] > 0 && Data->samples[count-1] < 0)
    {
      switch (Data->crossing)
      {
      case 0:
        Data->numberOfSamples = 2;
        Data->offset1 = getOffset(Data->samples[count-1],Data->samples[count]);
        Data->crossing = 1;
        break;
      case 1:
        Data->offset2 = getOffset(Data->samples[count-1],Data->samples[count]);
        float period = (Data->numberOfSamples - Data->offset1 + Data->offset2)*PIT_TIME_PERIOD;
        float freq = 1/(period/1e-9);
        if (freq >= 47.5 && freq <= 52.5)
        {
          Data->frequency = freq;
          PIT_TIME_PERIOD = period;
          PIT_Set(PIT_TIME_PERIOD,false,0);
        }
        Data->crossing = 0;
        break;
      default:
        Data->crossing = 0;
        break;
      }
    }
  }
  else
  {
    Data->numberOfSamples++;
  }
}

void DOR_TimingThread(void* pData)
{
  int count;
  for (;;)
  {
    (void)OS_SemaphoreWait(channelData.semaphore, 0);

    channelData.samples[count] = raw2v(channelData.sample);

    count ++;
    if (count == 16)
    {
      channelData.irms = returnRMS(channelData.samples);
      count = 0;
    }

//    if (channelData.channelNb == 0)
//    {
//      getFrequency(&channelData, count);
//    }

    if (channelData.irms > 1.03 && !channelData.timerStatus)
    {
      Analog_Put(TIMING_OUTPUT_CHANNEL,v2raw(5));
      channelData.timerStatus = 1;
      channelData.currentTimeCount = 0;
    }
    else if (channelData.irms < 1.03)
    {
      Analog_Put(TIMING_OUTPUT_CHANNEL,v2raw(0));
      channelData.timerStatus = 0;
    }

    OS_SemaphoreSignal(TripSemaphore);
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
//      channelData.tripTime = interpolate(INV_TRIP_TIME,channelData.irms);
      channelData.tripTime = 17610;
      if (channelData.currentTimeCount >= channelData.tripTime)
      {
        // Set Output high
        Analog_Put(TRIP_OUTPUT_CHANNEL,v2raw(5));
      }

    }
    else
    {
      // Set Output low
      Analog_Put(TRIP_OUTPUT_CHANNEL,v2raw(0));
    }


  }
}


/*!
 * @}
*/

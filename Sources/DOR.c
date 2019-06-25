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
#include "Flash.h"


// Pit time period (nano seconds)
// Todo: change to proper case for vairable
static uint32_t PIT_TIME_PERIOD = 1250e3;//Sampling 16per cycle at 50Hz
//static uint32_t PIT_TIME_PERIOD = 10000000; // 1ms
//static uint32_t PIT_TIME_PERIOD = 1e6;//Sampling 16per cycle at 50Hz
static uint32_t PIT1_TIME_PERIOD = 1000000; // 1ms

//Output channels
static const uint8_t TIMING_OUTPUT_CHANNEL = 1;
static const uint8_t TRIP_OUTPUT_CHANNEL = 2;

static const uint16_t ADC_CONVERSION = 3276;



uint16_t analogInputValue;



#define channelData (*(TAnalogThreadData*)pData)


static OS_ECB* TripSemaphore;


/*! @brief Analog thread configuration data
 *
 */
TAnalogThreadData ChannelThreadData[NB_ANALOG_CHANNELS] =
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
  {
    .semaphore = NULL,
    .channelNb = 1,
    .timerStatus = 0,
    .currentTimeCount = 0,
    .offset1 = 0,
    .offset2 = 0,
    .numberOfSamples = 0,
    .crossing = 0,
  },
  {
    .semaphore = NULL,
    .channelNb = 2,
    .timerStatus = 0,
    .currentTimeCount = 0,
    .offset1 = 0,
    .offset2 = 0,
    .numberOfSamples = 0,
    .crossing = 0,
  }
};

static void PIT0Callback(void* arg)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer

  for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
//    Analog_Get(0, &ChannelThreadData[i].sample);
    OS_SemaphoreSignal(ChannelThreadData[i].semaphore);
  }
//  Analog_Get(0, &ChannelThreadData[0].sample);
//  OS_SemaphoreSignal(ChannelThreadData[0].semaphore);

}

static void PIT1Callback(void* arg)
{
  for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    if (ChannelThreadData[i].timerStatus)
    {
      ChannelThreadData[i].currentTimeCount++;
    }
    OS_SemaphoreSignal(TripSemaphore);
  }

}

bool DOR_Init(const TDORSetup* const dorSetup)
{
  ChannelThreadData[0].semaphore = OS_SemaphoreCreate(0);
  ChannelThreadData[1].semaphore = OS_SemaphoreCreate(0);
  ChannelThreadData[2].semaphore = OS_SemaphoreCreate(0);

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

  error = OS_ThreadCreate(DOR_TimingThread,
                          &ChannelThreadData[1],
                          dorSetup->Channel1Params->pStack,
                          dorSetup->Channel1Params->priority);
  error = OS_ThreadCreate(DOR_TimingThread,
                          &ChannelThreadData[2],
                          dorSetup->Channel2Params->pStack,
                          dorSetup->Channel2Params->priority);

  error = OS_ThreadCreate(DOR_TripThread,
                          dorSetup->TripParams->pData,
                          dorSetup->TripParams->pStack,
                          dorSetup->TripParams->priority);

  return true;
}

static int16_t v2raw(float voltage)
{
  return (int16_t)(voltage*ADC_CONVERSION);
}

static float raw2v(int16_t voltage)
{
  return (float)voltage/(float)ADC_CONVERSION;
}

static float returnRMS(TAnalogThreadData* Data)
{

  float square = 0;
  float volts;
  float vrms;
  for (uint8_t i = 0; i<16;i++)
  {
    volts=raw2v(Data->samples[i]);

    square += (volts*volts);
  }

  vrms = sqrt(square/16);

  return (float)((vrms*40)/13);
}



static float getOffset(int16_t sample0, int16_t sample1)
{
  float m = (sample1-sample0);//Always  divide by one becuase 0ne sample diff
  return (-sample0)/m;
}

static void getFrequency(TAnalogThreadData* Data,int16_t prevVal, int16_t curVal)
{
  float period;
  static int tempLoop=0;

    // check at full cycle zero crossing
//    if (Data->samples[count] > 0 && Data->samples[count-1] < 0)
//    {
      switch (Data->crossing)
      {
      case 0:
        Data->numberOfSamples = 0;
        Data->offset1 = getOffset(prevVal,curVal);
        Data->crossing = 1;
        break;
      case 1:
        Data->offset2 = getOffset(prevVal,curVal);
//        float period1 = Data->offset1*PIT_TIME_PERIOD;
//        float period2 = Data->offset2*PIT_TIME_PERIOD;
        period = (Data->numberOfSamples+1-Data->offset1+Data->offset2)*(float)PIT_TIME_PERIOD;
        float freq = 1/((float)(period)*1e-9);

        Data->frequency[tempLoop] = freq;
        tempLoop++;
        if (tempLoop >=3)
          tempLoop = 0;
//
//        if (freq >= 47.5 && freq <= 52.5)
//        {
////          Data->frequency = freq;
////          PIT_TIME_PERIOD = period;
////          PIT_Set(PIT_TIME_PERIOD,false,0);
//        }
        Data->crossing = 0;
        break;
      default:
        Data->crossing = 0;
        break;
    }
//    }
}

static void setTimer()
{
  // Check if any channel has timer status set
  if (ChannelThreadData[0].timerStatus ||
      ChannelThreadData[1].timerStatus ||
      ChannelThreadData[2].timerStatus)
  {
    // Set timer output to 5 volts
    Analog_Put(TIMING_OUTPUT_CHANNEL,v2raw(5));
  }
  else
  {
    // Set timer output to 0 volts once all are cleared
    Analog_Put(TIMING_OUTPUT_CHANNEL,v2raw(0));
  }
}

static void setTrip()
{
  // Check if any channel has trip status set
  if (ChannelThreadData[0].tripStatus ||
      ChannelThreadData[1].tripStatus ||
      ChannelThreadData[2].tripStatus)
  {
    // Set timer output to 5 volts
    Analog_Put(TRIP_OUTPUT_CHANNEL,v2raw(5));
  }
  else
  {
    // Set timer output to 0 volts once all are cleared
    Analog_Put(TRIP_OUTPUT_CHANNEL,v2raw(0));
  }
}

void DOR_TimingThread(void* pData)
{
  int count;
  for (;;)
  {
    (void)OS_SemaphoreWait(channelData.semaphore, 0);

    Analog_Get(channelData.channelNb, &channelData.sample);
    channelData.samples[count] = channelData.sample;

    if (channelData.channelNb == 0)
    {
      if (count > 0)
      {
        if ((channelData.samples[count] > 0 && channelData.samples[count-1] < 0))
        {
        //        float temp = channelData.samples[count];
        //        float temp1 = channelData.samples[count-1];
         getFrequency(&channelData,channelData.samples[count-1], channelData.samples[count]);

        }
        channelData.numberOfSamples++;
      }

    }

    count ++;
    if (count == 16)
    {
      channelData.irms = returnRMS(&channelData);
      count = 0;
    }





    if (channelData.irms > 1.03 && !channelData.timerStatus)
    {
//      bool temp = Analog_Put(TIMING_OUTPUT_CHANNEL,v2raw(5));
      channelData.timerStatus = 1;
      channelData.currentTimeCount = 0;
      setTimer();
    }
    else if (channelData.irms < 1.03)
    {
      channelData.timerStatus = 0;
      setTimer();
    }


  }
}

static uint32_t getTripTime(float irms, TIDMTCharacter characteristic)
{
  switch (characteristic)
  {
  case 0:
    return INV_TRIP_TIME[((uint16_t)irms*100)-103];
    break;
  case 1:
    return VINV_TRIP_TIME[((uint16_t)irms*100)-103];
    break;
  case 2:
    return EINV_TRIP_TIME[((uint16_t)irms*100)-103];
    break;
  default:
    break;
  }
}

void DOR_TripThread(void* pData)
{
  TDORTripThreadData* tripThreadData = pData;
  for(;;)
  {
    (void)OS_SemaphoreWait(TripSemaphore, 0);

    // TODO: Add multi channel functionality
    for (int i = 0; i < 3; i++)
    {
      //check if timer started
      if (ChannelThreadData[i].timerStatus && !ChannelThreadData[i].tripStatus)
      {
  //      channelData.tripTime = interpolate(INV_TRIP_TIME,channelData.irms);
  //      channelData.tripTime = 17610;
  //      channelData.tripTime = ((float)0.14/(pow(channelData.irms,0.02)-1))*1000;
  //      uint16_t temp = (uint16_t)(channelData.irms*100)-103;

//        ChannelThreadData[i].tripTime = INV_TRIP_TIME[((uint16_t)ChannelThreadData[i].irms*100)-103];

        ChannelThreadData[i].tripTime = getTripTime(ChannelThreadData[i].irms, *tripThreadData->characteristic);

        if (ChannelThreadData[i].currentTimeCount >= ChannelThreadData[i].tripTime)
        {
          // Set Output high
          ChannelThreadData[i].tripStatus = 1;
          Flash_Write16((uint16_t*)tripThreadData->timesTripped,tripThreadData->timesTripped->l+1);
          Flash_Write8(tripThreadData->faultType,*tripThreadData->faultType | (1<<ChannelThreadData[i].channelNb));
          setTrip();
        }

      }
      else if (ChannelThreadData[i].tripStatus && !ChannelThreadData[i].timerStatus)
      {
        // Set Output low
        ChannelThreadData[i].tripStatus = 0;
        Flash_Write8(tripThreadData->faultType,*tripThreadData->faultType & ~(1<<ChannelThreadData[i].channelNb));
        setTrip();
      }
    }

  }
}


/*!
 * @}
*/

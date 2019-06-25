/*!
 * @addtogroup DOR_module DOR module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for the Digital Overcurrent Relay
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


// Pit time periods (nano seconds)
/*
 * PIT0 sample period.
 * Default sampling 16per cycle at 50Hz.
 * Used for sampling phases
 */
static uint32_t MyPIT0TimePeriod = 1250e3;
/*
 * PIT1 sample period.
 * Default sampling every 1ms.
 * Used for incrementing currentTimeCount for each phase.
 */
static uint32_t MyPIT1TimePeriod = 1000000;

// Output channels
static const uint8_t TIMING_OUTPUT_CHANNEL = 1; // "Timing" output signal
static const uint8_t TRIP_OUTPUT_CHANNEL = 2; // "Trip" output signal

// Constant to convert ADC/DAC 16 bit value to volts
static const uint16_t ADC_CONVERSION = 3276;

// Variable to address thread data - makes code easier to read
#define channelData (*(TDORPhaseData*)pData)

// Semaphore for tripThread
static OS_ECB* TripSemaphore;

// Configuration of array storing data for all three phases
TDORPhaseData DOR_PhaseData[NB_ANALOG_CHANNELS] =
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
    .count = 0,
    .subtract = 0,
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
    .count = 0,
    .subtract = 0,
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
    .count = 0,
    .subtract = 0,
  }
};

/*! @brief Checks for any valid packets and then handles them.
 *
 *  @param pData is used to store the FTM0 Channel0 data.
 */
static void PIT0Callback(void* arg)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer

  for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
//    Analog_Get(0, &ChannelThreadData[i].sample);
    OS_SemaphoreSignal(DOR_PhaseData[i].semaphore);
  }
//  Analog_Get(0, &ChannelThreadData[0].sample);
//  OS_SemaphoreSignal(ChannelThreadData[0].semaphore);

}

static void PIT1Callback(void* arg)
{
  for (int i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    if (DOR_PhaseData[i].timerStatus)
    {
      DOR_PhaseData[i].currentTimeCount++;
    }
    OS_SemaphoreSignal(TripSemaphore);
  }

}

bool DOR_Init(const TDORSetup* const dorSetup)
{
  DOR_PhaseData[0].semaphore = OS_SemaphoreCreate(0);
  DOR_PhaseData[1].semaphore = OS_SemaphoreCreate(0);
  DOR_PhaseData[2].semaphore = OS_SemaphoreCreate(0);

  TripSemaphore = OS_SemaphoreCreate(0);


  //PIT setup struct
  TPITSetup pitSetup;
  pitSetup.moduleClk = dorSetup->moduleClk;
  pitSetup.EnablePITThread = 0;
  pitSetup.Semaphore[0] = DOR_PhaseData[0].semaphore;
  pitSetup.Semaphore[1] = NULL;
  pitSetup.CallbackFunction[0] = PIT0Callback;
  pitSetup.CallbackArguments[0] = NULL;
  pitSetup.CallbackFunction[1] = PIT1Callback;
  pitSetup.CallbackArguments[1] = NULL;

  PIT_Init(&pitSetup);
  Analog_Init(dorSetup->moduleClk);


  //Set PIT Timer
  PIT_Set(MyPIT0TimePeriod, true,0);
  PIT_Set(MyPIT1TimePeriod, true,1);

  OS_ERROR error;

  error = OS_ThreadCreate(DOR_TimingThread,
                          &DOR_PhaseData[0],
                          dorSetup->Channel0Params->pStack,
                          dorSetup->Channel0Params->priority);

  error = OS_ThreadCreate(DOR_TimingThread,
                          &DOR_PhaseData[1],
                          dorSetup->Channel1Params->pStack,
                          dorSetup->Channel1Params->priority);
  error = OS_ThreadCreate(DOR_TimingThread,
                          &DOR_PhaseData[2],
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

static float returnRMS(TDORPhaseData* Data)
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

static void getFrequency(TDORPhaseData* Data,int16_t prevVal, int16_t curVal)
{
  float period;

  switch (Data->crossing)
  {
  case 0:
    Data->numberOfSamples = 0;
    Data->offset1 = getOffset(prevVal,curVal);
    Data->crossing = 1;
    break;
  case 1:
    Data->offset2 = getOffset(prevVal,curVal);
    period = (Data->numberOfSamples+1-Data->offset1+Data->offset2)*(float)MyPIT0TimePeriod;
    float freq = 1/((float)(period)*1e-9);

    if (freq >= 47.5 && freq <= 52.5)
    {
      Data->frequency = freq;
      MyPIT0TimePeriod = period/16;
      PIT_Set(MyPIT0TimePeriod,false,0);
    }
    Data->crossing = 0;
    break;
  default:
    Data->crossing = 0;
    break;
  }
}

static void setTimer()
{
  // Check if any channel has timer status set
  if (DOR_PhaseData[0].timerStatus ||
      DOR_PhaseData[1].timerStatus ||
      DOR_PhaseData[2].timerStatus)
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
  if (DOR_PhaseData[0].tripStatus ||
      DOR_PhaseData[1].tripStatus ||
      DOR_PhaseData[2].tripStatus)
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
//  static int count;
//  static int32_t sumSquares;
  for (;;)
  {
    (void)OS_SemaphoreWait(channelData.semaphore, 0);

    Analog_Get(channelData.channelNb, &channelData.sample);
    channelData.samples[channelData.count] = channelData.sample;

    if (channelData.channelNb == 0)
    {
      if (channelData.count > 0)
      {
        if ((channelData.samples[channelData.count] > 0 && channelData.samples[channelData.count-1] < 0))
        {
        //        float temp = channelData.samples[count];
        //        float temp1 = channelData.samples[count-1];
         getFrequency(&channelData,channelData.samples[channelData.count-1], channelData.samples[channelData.count]);

        }
        channelData.numberOfSamples++;
      }

    }


//    channelData.sumSquares += channelData.samples[channelData.count]*channelData.samples[channelData.count];
//    if (channelData.count == 0)
//    {
//      channelData.sumSquares -= channelData.samples[15]*channelData.samples[15];
//    }
//    else
//    {
//      channelData.sumSquares -= channelData.samples[channelData.count-1]*channelData.samples[channelData.count-1];
//    }
    channelData.squares[channelData.count] = pow(channelData.sample,2);
    channelData.sumSquares += channelData.squares[channelData.count];

    if (channelData.subtract)
    {
      if(channelData.count == NB_SAMPLES-1)
        channelData.sumSquares -= channelData.squares[0];
      else
        channelData.sumSquares -= channelData.squares[channelData.count+1];


    float vrms = sqrt(channelData.sumSquares/NB_SAMPLES);
    vrms = raw2v(vrms);
    float temp = ((vrms*40)/13);
    int16_t temp1 = (temp/1);
    channelData.irms = (vrms*40)/13 ;
    }

    channelData.count ++;
    if (channelData.count == NB_SAMPLES)
    {
//      channelData.irms = returnRMS(&channelData);
      channelData.count = 0;
      channelData.subtract = 1;
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
  // check irms lower than upper range
  if (irms > 20)
    irms = 20;

  uint16_t postion = (uint16_t)(irms*100)-103;

  switch (characteristic)
  {
  case 0:
    return INV_TRIP_TIME[postion];
    break;
  case 1:
    return VINV_TRIP_TIME[postion];
    break;
  case 2:
    return EINV_TRIP_TIME[postion];
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
      if (DOR_PhaseData[i].timerStatus && !DOR_PhaseData[i].tripStatus)
      {
  //      channelData.tripTime = interpolate(INV_TRIP_TIME,channelData.irms);
  //      channelData.tripTime = 17610;
  //      channelData.tripTime = ((float)0.14/(pow(channelData.irms,0.02)-1))*1000;
  //      uint16_t temp = (uint16_t)(channelData.irms*100)-103;

//        ChannelThreadData[i].tripTime = INV_TRIP_TIME[((uint16_t)ChannelThreadData[i].irms*100)-103];

        DOR_PhaseData[i].tripTime = getTripTime(DOR_PhaseData[i].irms, *tripThreadData->characteristic);

        if (DOR_PhaseData[i].currentTimeCount >= DOR_PhaseData[i].tripTime)
        {
          // Set Output high
          DOR_PhaseData[i].tripStatus = 1;
          DOR_PhaseData[i].currentWTripped = DOR_PhaseData[i].irms;
          Flash_Write16((uint16_t*)tripThreadData->timesTripped,tripThreadData->timesTripped->l+1);
          Flash_Write8(tripThreadData->faultType,*tripThreadData->faultType | (1<<DOR_PhaseData[i].channelNb));
          setTrip();
        }

      }
      else if (DOR_PhaseData[i].tripStatus && !DOR_PhaseData[i].timerStatus)
      {
        // Set Output low
        DOR_PhaseData[i].tripStatus = 0;
        Flash_Write8(tripThreadData->faultType,*tripThreadData->faultType & ~(1<<DOR_PhaseData[i].channelNb));
        setTrip();
      }
    }

  }
}


/*!
 * @}
*/

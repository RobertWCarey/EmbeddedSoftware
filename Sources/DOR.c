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
static const uint16_t ANALOG_16BIT_CONVERSION = 3276;

// Semaphore for tripThread
static OS_ECB* TripSemaphore;

// Voltages for Outputs as 16bit values
static const int16_t IDLE_OUTPUT_VOLTAGE_16BIT = 0;       // Equivalent to OV
static const int16_t ACTIVE_OUTPUT_VOLTAGE_16BIT = 16384; // Equivalent to 5V

// Current limits for trip of DOR
static const float DOR_CURRENT_LIMIT_LOWER = 1.03;
static const float DOR_CURRENT_LIMIT_UPPER = 20;

// Configuration of array storing data for all three phases
TDORPhaseData DOR_PhaseData[DOR_NB_PHASES] =
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
    .windowFilled = 0,
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
    .windowFilled = 0,
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
    .windowFilled = 0,
  }
};

/*! @brief Interrupt callback function to be called when PIT_ISR trigger by PIT0.
 *  @brief Used to trigger samples from the ADC at set sample rate.
 *
 *  @param arg The user argument that comes with the callback.
 */
static void PIT0Callback(void* arg)
{
  // Cycle through all phases
  for (int i = 0; i < DOR_NB_PHASES; i++)
  {
    // Signal TimingThread to sample data
    OS_SemaphoreSignal(DOR_PhaseData[i].semaphore);
  }
}

/*! @brief Interrupt callback function to be called when PIT_ISR trigger by PIT1.
 *  @brief Used to increment each phase's currentTimeCount every 1ms.
 *
 *  @param arg The user argument that comes with the callback.
 */
static void PIT1Callback(void* arg)
{
  // Cycle through all phases
  for (int i = 0; i < DOR_NB_PHASES; i++)
  {
    // If phase's "Timer" output has been triggered
    if (DOR_PhaseData[i].timerStatus)
    {
      // Increment phase's currentTimeCount by 1ms.
      DOR_PhaseData[i].currentTimeCount++;
    }
    // Signal the Trip Thread to process
    OS_SemaphoreSignal(TripSemaphore);
  }
}

bool DOR_Init(const TDORSetup* const dorSetup)
{
  // Create all DOR semaphores
  DOR_PhaseData[PHASE_A].semaphore = OS_SemaphoreCreate(0); // Phase A Timing Thread semaphore
  DOR_PhaseData[PHASE_B].semaphore = OS_SemaphoreCreate(0); // Phase B Timing Thread semaphore
  DOR_PhaseData[PHASE_C].semaphore = OS_SemaphoreCreate(0); // Phase C Timing Thread semaphore
  TripSemaphore = OS_SemaphoreCreate(0);  // DOR Trip Thread semaphore

  //PIT setup struct
  TPITSetup pitSetup;
  pitSetup.moduleClk = dorSetup->moduleClk;
  pitSetup.CallbackFunction[0] = PIT0Callback;
  pitSetup.CallbackArguments[0] = NULL;
  pitSetup.CallbackFunction[1] = PIT1Callback;
  pitSetup.CallbackArguments[1] = NULL;

  // Initilise Modules
  PIT_Init(&pitSetup); // Initilise Pit Module
  Analog_Init(dorSetup->moduleClk); // Initilise Analog Module

  //Set PIT Timers to default values
  PIT_Set(MyPIT0TimePeriod, true,0);
  PIT_Set(MyPIT1TimePeriod, true,1);

  // Variable to catch any OS errors
  OS_ERROR error;

  // Create Timing threads for all three phases
  // Phase A
  error = OS_ThreadCreate(DOR_TimingThread,
                          &DOR_PhaseData[0],
                          dorSetup->PhaseAParams->pStack,
                          dorSetup->PhaseAParams->priority);
  // Phase B
  error = OS_ThreadCreate(DOR_TimingThread,
                          &DOR_PhaseData[1],
                          dorSetup->PhaseBParams->pStack,
                          dorSetup->PhaseBParams->priority);
  //Phase C
  error = OS_ThreadCreate(DOR_TimingThread,
                          &DOR_PhaseData[2],
                          dorSetup->PhaseCParams->pStack,
                          dorSetup->PhaseCParams->priority);
  // Create DOR Trip Thread
  error = OS_ThreadCreate(DOR_TripThread,
                          dorSetup->TripParams->pData,
                          dorSetup->TripParams->pStack,
                          dorSetup->TripParams->priority);

  return true;
}

/*! @brief Converts voltage to 16 bit value for Analog module.
 *
 *  @param voltage value of voltage to be converted.
 */
static int16_t volts2Analog(float voltage)
{
  // Return 16 bit equivalent of voltage
  return (int16_t)(voltage*ANALOG_16BIT_CONVERSION);
}

/*! @brief Converts 16 bit value from Analog module to voltage.
 *
 *  @param analogVal 16 bit Analog Value to be converted.
 */
static float analog2Volts(int16_t analogVal)
{
  // Return float equivalent voltage of 16 bit analog value
  return (float)analogVal/(float)ANALOG_16BIT_CONVERSION;
}

/*! @brief Calculates the RMS current in a sliding window 32 samples wide.
 *
 *  @param data is a pointer the a Phase's TDORPhaseData struct.
 */
static void calculateRMS(TDORPhaseData* data)
{
  // Square the current sample and store in the "squares" array
  data->squares[data->count] = pow(data->sample,2);
  // Add the current squared value to the "sumSquares" variable
  data->sumSquares += data->squares[data->count];

  // Check that a full window of samples has been collected
  // Otherwise incorrect RMS value for first 32 reads
  if (data->windowFilled)
  {
    // If at end of array
    if(data->count == NB_SAMPLES-1)
      // Subtract oldest data from first position
      data->sumSquares -= data->squares[0];
    else
      // Subtract oldest data from one position ahead of current position
      data->sumSquares -= data->squares[data->count+1];

    // Store analog RMS in temporary variable
    float aRMS = sqrt(data->sumSquares/NB_SAMPLES);
    // Convert analog RMS to a RMS voltage
    float vRMS = analog2Volts(aRMS);

    // Convert RMS Voltage to RMS Current and store  in Phase's TDORPhaseData struct
    data->irms = (vRMS*40)/13 ;
  }

  // Increment count to move position up in arrays
  data->count ++;
  // If count reaches number of samples loop
  if (data->count == NB_SAMPLES)
  {
    // Set count to zero to loop
    data->count = 0;
    // This is to indicate a window of samples has been collected
    // After the first "NB_SAMPLES" has been collected this is always true.
    data->windowFilled = true;
  }
}

/*! @brief Calculates the offset of the sample before the zero crossing as a portion of a sample.
 *
 *  @param sample0 is the value of the sample before the zero crossing.
 *  @param sample1 is the value of the sample after the zero crossing.
 */
static float getZeroCrossingOffset(int16_t sample0, int16_t sample1)
{
  // using y=mx
  // Calculate gradient
  float m = (sample1-sample0);//Always  divide by one becuase one sample. i.e. x1=0 x2=1
  // Return the offset as a portion of a sample
  return (-sample0)/m;
}

/*! @brief Calculates the frequency of the phase using a complete period
 *
 *  @param data is a pointer the a Phase's TDORPhaseData struct.
 */
static void calculateFreq(TDORPhaseData* data)
{
  // Check that not at the first position in the array
  if (data->count > 0)
  {
    // Check if at a zero crossing with a positive gradient
    if ((data->samples[data->count] > 0 && data->samples[data->count-1] < 0))
    {
      // Temporary value to store period in
      float period;

      // Switch between first and second zero crossing
      switch (data->crossing)
      {
      case 0:
        data->numberOfSamples = 1;
        // Calculate offset at first zero crossing
        data->offset1 = getZeroCrossingOffset(data->samples[data->count-1],data->samples[data->count]);
        data->crossing = 1;
        break;
      case 1:
        // Calculate offset at second zero crossing
        data->offset2 = getZeroCrossingOffset(data->samples[data->count-1],data->samples[data->count]);
        // Calculate period between zero crossings
        // Number of samples * current PIT0 sampling period
        period = (data->numberOfSamples-data->offset1+data->offset2)*(float)MyPIT0TimePeriod;
        // Store calculated frequency in a temporary variable so value can be checked
        float freq = 1/((float)(period)*1e-9);

        // Check if the freq is within the valid range
        if (freq >= 47.5 && freq <= 52.5)
        {
          // Store in Phase's data struct
          data->frequency = freq;
          // Update sampling period to ensure 16 samples per cycle
          MyPIT0TimePeriod = period/16;
          PIT_Set(MyPIT0TimePeriod,false,0);
        }
        data->crossing = 0;
        break;
      default:
        data->crossing = 0;
        break;
      }
    }
    // Increment to record number of samples between zero crossing
    data->numberOfSamples++;
    }
}

/*! @brief Determines whether the "Timer" output should be set/cleared.
 *         Based on the "timerStatus" set in each Phase's timing thread
 *
 */
static void setTimerOutput()
{
  // Check if any channel has timer status set
  if (DOR_PhaseData[PHASE_A].timerStatus ||
      DOR_PhaseData[PHASE_B].timerStatus ||
      DOR_PhaseData[PHASE_C].timerStatus)
  {
    // Set timer output to 5 volts
    Analog_Put(TIMING_OUTPUT_CHANNEL,ACTIVE_OUTPUT_VOLTAGE_16BIT);
  }
  else
  {
    // Set timer output to 0 volts once all are cleared
    Analog_Put(TIMING_OUTPUT_CHANNEL,IDLE_OUTPUT_VOLTAGE_16BIT);
  }
}

/*! @brief Determines whether the "Trip" output should be set/cleared.
 *         Based on the "tripStatus" set in each Phase's timing thread
 *
 */
static void setTripOutput()
{
  // Check if any channel has trip status set
  if (DOR_PhaseData[0].tripStatus ||
      DOR_PhaseData[1].tripStatus ||
      DOR_PhaseData[2].tripStatus)
  {
    // Set timer output to 5 volts
    Analog_Put(TRIP_OUTPUT_CHANNEL,ACTIVE_OUTPUT_VOLTAGE_16BIT);
  }
  else
  {
    // Set timer output to 0 volts once all are cleared
    Analog_Put(TRIP_OUTPUT_CHANNEL,IDLE_OUTPUT_VOLTAGE_16BIT);
  }
}

void DOR_TimingThread(void* pData)
{
  for (;;)
  {
    // Variable to address thread data - makes code easier to read
    TDORPhaseData* phaseData = pData;

    // Wait for PIT0 to signal semaphore
    (void)OS_SemaphoreWait(phaseData->semaphore, 0);

    // Read data from ADC and store in array for processing
    Analog_Get(phaseData->channelNb, &phaseData->sample);
    phaseData->samples[phaseData->count] = phaseData->sample;

    // Only calculate the frequency for Phase A
    // Assumes that all phases share the same frequency
    if (phaseData->channelNb == PHASE_A)
      calculateFreq(phaseData);

    // Calculate the current RMS in sliding window
    calculateRMS(phaseData);

    // If current over the threshold and "Timer" output hasnt been set
    if (phaseData->irms > DOR_CURRENT_LIMIT_LOWER && !phaseData->timerStatus)
    {
      // Set "Timer" output and reset Phase's currentTimeCount
      phaseData->timerStatus = 1;
      phaseData->currentTimeCount = 0;
      setTimerOutput();
    }
    else if (phaseData->irms < DOR_CURRENT_LIMIT_LOWER)
    {
      // Once below threshold reset Phase's status
      phaseData->timerStatus = 0;
      setTimerOutput();
    }
  }
}

/*! @brief gets the trip time depending on current characteristic.
 *
 *  @param  irms            is the current for which a trip time needs to be returned.
 *  @param  characteristic  is the current characteristic curve selected.
 *  @return uint32_t - trip time in milliseconds.
 */
static uint32_t getTripTime(float irms, TIDMTCharacter characteristic)
{
  // If current greater than upper limit
  if (irms > DOR_CURRENT_LIMIT_UPPER)
    // Set to highest valid value
    irms = DOR_CURRENT_LIMIT_UPPER;

  // Calculate position in IDMT curve arrays
  // "-103" is to align position to start at 1.03 amps
  uint16_t postion = (uint16_t)(irms*100)-103;

  // Return relevant trip time in milliseconds
  switch (characteristic)
  {
  case IDMT_INVERSE:
    return INV_TRIP_TIME[postion];
    break;
  case IDMT_V_INVERSE:
    return VINV_TRIP_TIME[postion];
    break;
  case IDMT_E_INVERSE:
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
          setTripOutput();
        }

      }
      else if (DOR_PhaseData[i].tripStatus && !DOR_PhaseData[i].timerStatus)
      {
        // Set Output low
        DOR_PhaseData[i].tripStatus = 0;
        Flash_Write8(tripThreadData->faultType,*tripThreadData->faultType & ~(1<<DOR_PhaseData[i].channelNb));
        setTripOutput();
      }
    }

  }
}


/*!
 * @}
*/

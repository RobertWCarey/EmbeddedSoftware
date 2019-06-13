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

static TFIFO ch0Buffer;

static int16_t analogInputValue;
static uint32_t ch0Sum = 0;

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
  #define analogData ((TAnalogThreadData*)pData)
  for (;;)
  {
    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    double volts;
    analogInputValue = abs(analogInputValue);

    volts = (double)analogInputValue/(double)3276;

    uint16union_t square;
    square.l = volts*volts;
    ch0Sum += square.l;
    FIFO_Put(&ch0Buffer,square.s.Lo);
    FIFO_Put(&ch0Buffer,square.s.Hi);
    if (ch0Buffer.ItemsAvailable->count > 96)
    {
      uint16union_t temp;
      FIFO_Get(&ch0Buffer, &temp.s.Lo);
      FIFO_Get(&ch0Buffer, &temp.s.Hi);
      ch0Sum -= temp.l;
    }
    uint32_t mean = ch0Sum/ch0Buffer.ItemsAvailable->count;

    uint32_t RMS = (uint32_t)sqrt(mean);

    uint16 I = (uint16_t)(RMS/0.35);

    Analog_Get(0, &analogInputValue);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);
  }
}


/*!
 * @}
*/

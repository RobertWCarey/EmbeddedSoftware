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
  Analog_Init(dorSetup->moduleClk);

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

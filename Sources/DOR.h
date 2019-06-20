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

#ifndef DOR_H
#define DOR_H

#include "analog.h"
#include "types.h"
#include "MK70F12.h"
#include "OS.h"
#include "PE_types.h"

typedef struct
{
  uint32_t moduleClk;         /*!< The module clock rate in Hz. */
  TOSThreadParams* Channel0Params;  /*!< Thread parameters for Channel0. */
  TOSThreadParams* TripParams;
} TDORSetup;

typedef struct ChannelThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  float irms;
  int16_t samples[16];
} TAnalogThreadData;

bool DOR_Init(const TDORSetup* const dorSetup);

void DOR_TimingThread(void* pData);

void DOR_TripThread(void* pData);

#endif

/*!
 * @}
*/

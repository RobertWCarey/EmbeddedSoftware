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

#ifndef UART_H
#define UART_H

#include "types.h"
#include "MK70F12.h"
#include "OS.h"

typedef struct
{
  uint32_t moduleClk;         /*!< The module clock rate in Hz. */
  uint32_t baudRate;          /*!< The desired baud rate in bits/sec. */
  TOSThreadParams* Channel0Params;  /*!< Thread parameters for Channel0. */
} TDORSetup;

bool DOR_Init(const TDORsetup* const dorSetup);

#endif

/*!
 * @}
*/

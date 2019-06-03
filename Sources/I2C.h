/*!
 * @addtogroup I2C_module I2C module documentation
 * @{
 */
/*! @file
 *
 *  @brief I/O routines for the K70 I2C interface.
 *
 *  This contains the functions for operating the I2C (inter-integrated circuit) module.
 *
 *  @author PMcL
 *  @date 2015-09-17
 */

#ifndef I2C_H
#define I2C_H

// new types
#include "types.h"
#include "OS.h"
#include "math.h"
#include "stdlib.h"
#include "types.h"
#include "MK70F12.h"
#include "Cpu.h"

typedef struct
{
  uint8_t primarySlaveAddress;                  /*!< The Slave address used unless I2C_SelectSlaveDevice() is called. */
  uint32_t baudRate;                            /*!< The desired baud rate in bits/sec. */
  uint32_t moduleClk;                           /*!< The module clock rate in Hz. */
  void (*readCompleteCallbackFunction)(void*);  /*!< The user's read complete callback function. */
  void* readCompleteCallbackArguments;          /*!< The user's read complete callback function arguments. */
  TOSThreadParams* ThreadParams;                /*!< Thread parameters for RTCThread. */
} TI2CSetup;


void I2CThread(void* pData);

/*! @brief Sets up the I2C before first use.
 *
 *  @param aI2Cetup is a pointer to an I2C setup structure.
 *  @return BOOL - TRUE if the I2C module was successfully initialized.
 */
bool I2C_Init(const TI2CSetup* const aI2CSetup);

/*! @brief Selects the current slave device
 *
 * @param slaveAddress The slave device address.
 */
void I2C_SelectSlaveDevice(const uint8_t slaveAddress);

/*! @brief Write a byte of data to a specified register
 *
 * @param registerAddress The register address.
 * @param data The 8-bit data to write.
 */
void I2C_Write(const uint8_t registerAddress, const uint8_t data);

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses polling as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes);

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses interrupts as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes);

/*! @brief Interrupt service routine for the I2C.
 *
 *  Only used for reading data.
 *  At the end of reception, the user callback function will be called.
 *  @note Assumes the I2C module has been initialized.
 */
void __attribute__ ((interrupt)) I2C_ISR(void);

#endif

/*!
 * @}
 */


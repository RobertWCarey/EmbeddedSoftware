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
 *  @author Robert Carey
 *  @date 2019-05-17
 */


// new types
#include "I2C.h"
#include "math.h"
#include "stdlib.h"
#include "types.h"
#include "MK70F12.h"
#include "Cpu.h"

//Variable to place user defined functions passed form aI2CModule
static void (*UserFunction)(void*);
static void* UserArguments;

#define I2C0_START_BIT I2C0_C1 |= I2C_C1_MST_MASK;
#define I2C0_STOP_BIT I2C0_C1 &= ~I2C_C1_MST_MASK;

#define I2C0_TRANSMIT I2C0_C1 |= I2C_C1_TX_MASK;
#define I2C0_RECIEVE I2C0_C1 &= ~I2C_C1_TX_MASK;

static const uint8_t ReadBit = 0x00;
static const uint8_t WriteBit = 0x01;

//Number of values in the ICR range
//#define as used to create array
#define I2C_ICR_RANGE 64
// All possible SCL Divider Values for the I2C ICR
static const uint32 I2C_SCLDividerValues[I2C_ICR_RANGE] =
    {20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56, 68,
    48, 56, 64, 72, 80, 88, 104, 128, 80, 96, 112, 128, 144, 160,
    192, 240, 160, 192, 224, 256, 288, 320, 384, 480, 320, 384, 448,
    512, 579, 640, 768, 960, 640, 768, 896, 1024, 1152, 1280, 1536,
    1920, 1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840};

//Private global to store slaveAddress
static uint8_t SlaveAddress;

static bool transferComplete()
{
  //wait for the IICIF flag to be raised
  while (!(I2C0_S & I2C_S_IICIF_MASK));

  //Clear the flag
  I2C0_S |= I2C_S_IICIF_MASK;

  //Check if transfer complete is what raised the flag
  if (I2C0_S & I2C_S_TCF_MASK)
    return true;

  return false;
}

/*! @brief Sets up the I2C before first use.
 *
 *  @param aI2CModule is a structure containing the operating conditions for the module.
 *  @param moduleClk The module clock in Hz.
 *  @return BOOL - TRUE if the I2C module was successfully initialized.
 */
bool I2C_Init(const TI2CModule* const aI2CModule, const uint32_t moduleClk)
{
  //Load in user functions
  UserFunction = aI2CModule->readCompleteCallbackFunction;
  UserArguments = aI2CModule->readCompleteCallbackArguments;

  //Enable clk gate for portE
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  //Mux port for I2C0 on PTE18/19 for Accel SDA/SCL
  //Enable open drain
  //Configure as internal pullup resitor
  //Enable internal pullup
  PORTE_PCR18 = PORT_PCR_MUX(4) | PORT_PCR_ODE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
  PORTE_PCR19 = PORT_PCR_MUX(4) | PORT_PCR_ODE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

  //Configure the baud rate for the I2C

  //The desired divider value
  uint32_t targetSCLDiv = moduleClk / aI2CModule->baudRate;

  //Used to store the smallest error
  uint32_t minDiff = 0xFFFFFFFF;
  //used to record the closed values used
  uint8_t closestMult;
  uint8_t closestICR;

  //Cycle through all the ICR range
  for (int i = 0; i < I2C_ICR_RANGE; i++)
    {
      //Cycle through the possible mult values
      for (int j = 0; j < 3; j++)
	{
	  uint32_t diff = abs(I2C_SCLDividerValues[i] * pow(2,j) - targetSCLDiv);
	  if (diff < minDiff)
	    {
	      minDiff = diff;
	      closestMult = j;
	      closestICR = i;
	    }
	}
    }

  //load calculated values into the freq divider register
  I2C0_F |= I2C_F_ICR(closestICR);
  I2C0_F |= I2C_F_MULT(closestMult);

  //Initilise NVIC for I2C0
  //Non-IPR=0  IRQ=24
  //clear any pending interrupts at I2C0
  NVICICPR1 = (1 << 24);
  //Enable interrupts from I2C0
  NVICISER1 = (1 << 24);

  //Select Slave Address
  I2C_SelectSlaveDevice(aI2CModule->primarySlaveAddress);

  //Interrupts intentionally not enabled

  //Enable the I2C
  I2C0_C1 |= I2C_C1_IICEN_MASK;

  return true;
}

/*! @brief Selects the current slave device
 *
 * @param slaveAddress The slave device address.
 */
void I2C_SelectSlaveDevice(const uint8_t slaveAddress)
{
  //Store passed value in private global;
  SlaveAddress = slaveAddress;
}

/*! @brief Write a byte of data to a specified register
 *
 * @param registerAddress The register address.
 * @param data The 8-bit data to write.
 */
void I2C_Write(const uint8_t registerAddress, const uint8_t data)
{
  //Wait for the bus to clear
  while (!(I2C0_S & I2C_S_BUSY_MASK));

  //Select transmit mode
  I2C0_TRANSMIT;

  //Send Start bit
  I2C0_START_BIT

  //Send Slave address + Write bit
  I2C0_D = ((SlaveAddress << 1) + WriteBit);

  //Wait for transfer to complete
  while (!transferComplete());

  //Send Register address
  I2C0_D = registerAddress;

  //Wait for transfer to complete
  while (!transferComplete());

  //Send data to be written
  I2C0_D = data;

  //Wait for transfer to complete
  while (!transferComplete());

  //Send Stop bit
  I2C0_STOP_BIT
}

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses polling as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{

}

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses interrupts as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{

}

/*! @brief Interrupt service routine for the I2C.
 *
 *  Only used for reading data.
 *  At the end of reception, the user callback function will be called.
 *  @note Assumes the I2C module has been initialized.
 */
void __attribute__ ((interrupt)) I2C_ISR(void)
{

}


/*!
 * @}
 */

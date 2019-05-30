/*!
 * @addtogroup accel_module accel module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for the accelerometer
 *
 *  This contains the functions for communicating with the accelerometer
 *
 *  @author Robert Carey
 *  @date 2019-05-20
 */

// Accelerometer functions
#include "accel.h"

// Inter-Integrated Circuit
#include "I2C.h"

// Median filter
#include "median.h"

// K70 module registers
#include "MK70F12.h"

// CPU and PE_types are needed for critical section variables and the defintion of NULL pointer
#include "CPU.h"
#include "PE_types.h"

// Accelerometer registers
#define ADDRESS_OUT_X_MSB 0x01

#define ADDRESS_INT_SOURCE 0x0C

static union
{
  uint8_t byte;			/*!< The INT_SOURCE bits accessed as a byte. */
  struct
  {
    uint8_t SRC_DRDY   : 1;	/*!< Data ready interrupt status. */
    uint8_t               : 1;
    uint8_t SRC_FF_MT  : 1;	/*!< Freefall/motion interrupt status. */
    uint8_t SRC_PULSE  : 1;	/*!< Pulse detection interrupt status. */
    uint8_t SRC_LNDPRT : 1;	/*!< Orientation interrupt status. */
    uint8_t SRC_TRANS  : 1;	/*!< Transient interrupt status. */
    uint8_t SRC_FIFO   : 1;	/*!< FIFO interrupt status. */
    uint8_t SRC_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt status. */
  } bits;			/*!< The INT_SOURCE bits accessed individually. */
} INT_SOURCE_Union;

#define INT_SOURCE     		INT_SOURCE_Union.byte
#define INT_SOURCE_SRC_DRDY	INT_SOURCE_Union.bits.SRC_DRDY
#define INT_SOURCE_SRC_FF_MT	CTRL_REG4_Union.bits.SRC_FF_MT
#define INT_SOURCE_SRC_PULSE	CTRL_REG4_Union.bits.SRC_PULSE
#define INT_SOURCE_SRC_LNDPRT	CTRL_REG4_Union.bits.SRC_LNDPRT
#define INT_SOURCE_SRC_TRANS	CTRL_REG4_Union.bits.SRC_TRANS
#define INT_SOURCE_SRC_FIFO	CTRL_REG4_Union.bits.SRC_FIFO
#define INT_SOURCE_SRC_ASLP	CTRL_REG4_Union.bits.SRC_ASLP

#define ADDRESS_CTRL_REG1 0x2A

typedef enum
{
  DATE_RATE_800_HZ,
  DATE_RATE_400_HZ,
  DATE_RATE_200_HZ,
  DATE_RATE_100_HZ,
  DATE_RATE_50_HZ,
  DATE_RATE_12_5_HZ,
  DATE_RATE_6_25_HZ,
  DATE_RATE_1_56_HZ
} TOutputDataRate;

typedef enum
{
  SLEEP_MODE_RATE_50_HZ,
  SLEEP_MODE_RATE_12_5_HZ,
  SLEEP_MODE_RATE_6_25_HZ,
  SLEEP_MODE_RATE_1_56_HZ
} TSLEEPModeRate;

static union
{
  uint8_t byte;			/*!< The CTRL_REG1 bits accessed as a byte. */
  struct
  {
    uint8_t ACTIVE    : 1;	/*!< Mode selection. */
    uint8_t F_READ    : 1;	/*!< Fast read mode. */
    uint8_t LNOISE    : 1;	/*!< Reduced noise mode. */
    uint8_t DR        : 3;	/*!< Data rate selection. */
    uint8_t ASLP_RATE : 2;	/*!< Auto-WAKE sample frequency. */
  } bits;			/*!< The CTRL_REG1 bits accessed individually. */
} CTRL_REG1_Union;

#define CTRL_REG1     		    CTRL_REG1_Union.byte
#define CTRL_REG1_ACTIVE	    CTRL_REG1_Union.bits.ACTIVE
#define CTRL_REG1_F_READ  	  CTRL_REG1_Union.bits.F_READ
#define CTRL_REG1_LNOISE  	  CTRL_REG1_Union.bits.LNOISE
#define CTRL_REG1_DR	    	  CTRL_REG1_Union.bits.DR
#define CTRL_REG1_ASLP_RATE	  CTRL_REG1_Union.bits.ASLP_RATE

#define ADDRESS_CTRL_REG2 0x2B

#define ADDRESS_CTRL_REG3 0x2C

static union
{
  uint8_t byte;			/*!< The CTRL_REG3 bits accessed as a byte. */
  struct
  {
    uint8_t PP_OD       : 1;	/*!< Push-pull/open drain selection. */
    uint8_t IPOL        : 1;	/*!< Interrupt polarity. */
    uint8_t WAKE_FF_MT  : 1;	/*!< Freefall/motion function in SLEEP mode. */
    uint8_t WAKE_PULSE  : 1;	/*!< Pulse function in SLEEP mode. */
    uint8_t WAKE_LNDPRT : 1;	/*!< Orientation function in SLEEP mode. */
    uint8_t WAKE_TRANS  : 1;	/*!< Transient function in SLEEP mode. */
    uint8_t FIFO_GATE   : 1;	/*!< FIFO gate bypass. */
  } bits;			/*!< The CTRL_REG3 bits accessed individually. */
} CTRL_REG3_Union;

#define CTRL_REG3     		    CTRL_REG3_Union.byte
#define CTRL_REG3_PP_OD		    CTRL_REG3_Union.bits.PP_OD
#define CTRL_REG3_IPOL		    CTRL_REG3_Union.bits.IPOL
#define CTRL_REG3_WAKE_FF_MT	CTRL_REG3_Union.bits.WAKE_FF_MT
#define CTRL_REG3_WAKE_PULSE	CTRL_REG3_Union.bits.WAKE_PULSE
#define CTRL_REG3_WAKE_LNDPRT	CTRL_REG3_Union.bits.WAKE_LNDPRT
#define CTRL_REG3_WAKE_TRANS	CTRL_REG3_Union.bits.WAKE_TRANS
#define CTRL_REG3_FIFO_GATE	  CTRL_REG3_Union.bits.FIFO_GATE

#define ADDRESS_CTRL_REG4 0x2D

static union
{
  uint8_t byte;			/*!< The CTRL_REG4 bits accessed as a byte. */
  struct
  {
    uint8_t INT_EN_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t               : 1;
    uint8_t INT_EN_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_EN_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_EN_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_EN_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_EN_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_EN_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG4 bits accessed individually. */
} CTRL_REG4_Union;

#define CTRL_REG4            		CTRL_REG4_Union.byte
#define CTRL_REG4_INT_EN_DRDY	  CTRL_REG4_Union.bits.INT_EN_DRDY
#define CTRL_REG4_INT_EN_FF_MT	CTRL_REG4_Union.bits.INT_EN_FF_MT
#define CTRL_REG4_INT_EN_PULSE	CTRL_REG4_Union.bits.INT_EN_PULSE
#define CTRL_REG4_INT_EN_LNDPRT	CTRL_REG4_Union.bits.INT_EN_LNDPRT
#define CTRL_REG4_INT_EN_TRANS	CTRL_REG4_Union.bits.INT_EN_TRANS
#define CTRL_REG4_INT_EN_FIFO	  CTRL_REG4_Union.bits.INT_EN_FIFO
#define CTRL_REG4_INT_EN_ASLP	  CTRL_REG4_Union.bits.INT_EN_ASLP

#define ADDRESS_CTRL_REG5 0x2E

static union
{
  uint8_t byte;			/*!< The CTRL_REG5 bits accessed as a byte. */
  struct
  {
    uint8_t INT_CFG_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t                : 1;
    uint8_t INT_CFG_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_CFG_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_CFG_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_CFG_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_CFG_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_CFG_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG5 bits accessed individually. */
} CTRL_REG5_Union;

#define CTRL_REG5     		      	CTRL_REG5_Union.byte
#define CTRL_REG5_INT_CFG_DRDY		CTRL_REG5_Union.bits.INT_CFG_DRDY
#define CTRL_REG5_INT_CFG_FF_MT		CTRL_REG5_Union.bits.INT_CFG_FF_MT
#define CTRL_REG5_INT_CFG_PULSE		CTRL_REG5_Union.bits.INT_CFG_PULSE
#define CTRL_REG5_INT_CFG_LNDPRT	CTRL_REG5_Union.bits.INT_CFG_LNDPRT
#define CTRL_REG5_INT_CFG_TRANS		CTRL_REG5_Union.bits.INT_CFG_TRANS
#define CTRL_REG5_INT_CFG_FIFO		CTRL_REG5_Union.bits.INT_CFG_FIFO
#define CTRL_REG5_INT_CFG_ASLP		CTRL_REG5_Union.bits.INT_CFG_ASLP

// Accelerometer I2C bus address
// SA0 is pulled high so address is based on default values
static const uint8_t ACCEL_ADDRESS = 0x1D;

//Local global for callback
static void (*UserFunction)(void*);
static void* UserArguments;

//Global constant for I2C baud rate
static const uint32_t I2C_BAUD_RATE = 100000;

bool Accel_Init(const TAccelSetup* const accelSetup)
{
  //Struct to configure I2C module
  TI2CModule i2cSetup;
  i2cSetup.baudRate = I2C_BAUD_RATE;
  i2cSetup.primarySlaveAddress = ACCEL_ADDRESS;
  i2cSetup.readCompleteCallbackArguments = accelSetup->readCompleteCallbackArguments;
  i2cSetup.readCompleteCallbackFunction = accelSetup->readCompleteCallbackFunction;

  //Initialise I2C module
  I2C_Init(&i2cSetup,accelSetup->moduleClk);

  //Place Accelerometer in standby mode so that registers can be modified.
  CTRL_REG1_ACTIVE = 0;//Modify reg1 union for standby
  I2C_Write(ADDRESS_CTRL_REG1,CTRL_REG1);//write to accelerometer

  //Control Register 1 setup
  CTRL_REG1_F_READ = 1;//configure for fast read (single byte)
  CTRL_REG1_DR = DATE_RATE_1_56_HZ;//Data rate select 1.56Hz
  I2C_Write(ADDRESS_CTRL_REG1,CTRL_REG1);//write to accelerometer

  //Control Register 5 setup
  CTRL_REG5_INT_CFG_DRDY = 1;//Route Data ready interrupt to INT1 (PTB4)
  I2C_Write(ADDRESS_CTRL_REG5,CTRL_REG5);//write to accelerometer

  //Store callback into local global
  UserArguments = accelSetup->dataReadyCallbackArguments;
  UserFunction = accelSetup->dataReadyCallbackFunction;

  //Enable portB clk gate
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

  //interrupt not enabled for pin intentionally
  //configured in Accel_SetMode

  //Mux port PTB Alt1
  PORTB_PCR4 |= PORT_PCR_MUX(1);

  //Initialise PORTB NVIC
  //Non-IPR=2  IRQ=88
  //clear any pending interrupts at PORTB
  NVICICPR2 = (1 << 24);
  //Enable interrupts from PORTB
  NVICISER2 = (1 << 24);


  //Activate Accelerometer
  CTRL_REG1_ACTIVE = 1;//Modify reg1 union for active
  I2C_Write(ADDRESS_CTRL_REG1,CTRL_REG1);//write to accelerometer

  return true;
}

void Accel_ReadXYZ(uint8_t data[3])
{
  I2C_IntRead(ADDRESS_OUT_X_MSB, data, (sizeof (data)/sizeof (data[0])) );
}

void Accel_SetMode(const TAccelMode mode)
{
  //Entering Critical Section
  EnterCritical();

  //Place Accelerometer in standby mode so that registers can be modified.
  CTRL_REG1_ACTIVE = 0;//Modify reg1 union for standby
  I2C_Write(ADDRESS_CTRL_REG1,CTRL_REG1);//write to accelerometer

  switch (mode)
  {
    //Synchronous mode (use PIT interrupt)
    case ACCEL_POLL:
      //Disable PortB interrupt (0b0000)
      PORTB_PCR4 |= PORT_PCR_IRQC(0);

      //Disable Accel data ready interrupt
      //Control Register 4
      CTRL_REG4_INT_EN_DRDY = 0;//Disable Data ready interrupt trigger
      I2C_Write(ADDRESS_CTRL_REG4,CTRL_REG4);//write to accelerometer
    break;

    //Asynchronous mode (use PORTB interrupt)
    case ACCEL_INT:
      //Enable PortB interrupt on falling edge (0b1010)
      PORTB_PCR4 |= PORT_PCR_IRQC(10);

      //Enable Accel data ready interrupt
      //Control Register 4
      CTRL_REG4_INT_EN_DRDY = 1;//Enable Data ready interrupt trigger
      I2C_Write(ADDRESS_CTRL_REG4,CTRL_REG4);//write to accelerometer
    break;
    default:
    break;
  }

  //Activate Accelerometer
  CTRL_REG1_ACTIVE = 1;//Modify reg1 union for active
  I2C_Write(ADDRESS_CTRL_REG1,CTRL_REG1);//write to accelerometer

  ExitCritical(); //End critical section
}

void __attribute__ ((interrupt)) AccelDataReady_ISR(void)
{
  //Clear Flag
  PORTB_PCR4 |= PORT_PCR_ISF_MASK;

  if (UserFunction)
    (*UserFunction)(UserArguments);
}

/*!
 * @}
*/

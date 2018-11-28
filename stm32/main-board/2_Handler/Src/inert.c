////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      inert.c
//!  \brief     Inertial sensor handler
//!  \details   TODO 2 interrupt mode (needs some soldering)
//!             TODO data processing (averaging etc.)
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Idea for 2 interrupt mode:
 * - INT1 for accelerometer
 * - INT2 for gyroscope
 * - Each one sets a flag (in IT handler routine)
 * - If both are set the reading starts (from IT handler routine)
 * - @ the end of the reading both flags are reset.
 */

// Includes ------------------------------------------------------------------------------------------------------------

#include "inert.h"
#include "LSM6DS3.h"
#include "bsp_i2c.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define REGS_TO_READ (12u)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static uint8_t regSequence[REGS_TO_READ] =
{
	OUTX_L_XL,  OUTX_H_XL,  // Accelerometer X
	OUTY_L_XL,  OUTY_H_XL,  // Accelerometer Y
	OUTZ_L_XL,  OUTZ_H_XL,  // Accelerometer Z
	OUTX_L_G,   OUTX_H_G,   // Gyroscope X
	OUTY_L_G,   OUTY_H_G,   // Gyroscope Y
	OUTZ_L_G,   OUTZ_H_G    // Gyroscope Z
};

static uint8_t regResult[REGS_TO_READ];

static Accel  ret_accel;
static AngVel ret_angvel;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data);

// Global function definitions -----------------------------------------------------------------------------------------

void inertInit()
{
	HAL_GPIO_WritePin(INRT_SD0_GPIO_Port, INRT_SD0_Pin, GPIO_PIN_RESET); // Pull down SD0
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_SET); // CS (Stays active 4ever)

	// Enable accelerometer
	WriteRegBlocking(CTRL9_XL, 0x38); // Acc X, Y, Z axes enabled
	WriteRegBlocking(CTRL1_XL, 0x60); // Acc = 416Hz (High-Performance mode)

	// Enable gyroscope
	WriteRegBlocking(CTRL10_C, 0x38); // Gyro X, Y, Z axes enabled
	WriteRegBlocking(CTRL2_G, 0x60); // Gyro = 416Hz (High-Performance mode)

	// TODO enable interrupt
}

Accel inertGetAccel()
{
	Accel* ptr = (Accel*) &regResult[0];
    return *ptr;
}

AngVel inertGetAngVel()
{
    AngVel* ptr = (AngVel*) &regResult[6];
    return *ptr;
}

// Sensor reading ------------------------------------------

int i;

void inertTriggerMeasurement()
{
	i = 0;

	HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSequence[i], 1, &regResult[i], 1);
}

void i2cInertialSensorMemRxCallback()
{
	i++;

	if (i < REGS_TO_READ)
	{
		HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSequence[i], 1, &regResult[i], 1);
	}
	else // Sensor reading finished
	{
	    Accel*  acc_ptr =  (Accel*) &regResult[0];
	    AngVel* ang_ptr = (AngVel*) &regResult[6];

	    __disable_irq(); // Cheap thread safety

		// Copy the temporary values to their final location
		ret_accel  = *acc_ptr;
		ret_angvel = *ang_ptr;

		__enable_irq();
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data)
{
	// Assuming SD0 is pulled down
	HAL_I2C_Mem_Write(INERTIAL_I2C, LSM6DS3_ADDR0, regAddr, 1, &data, 1, HAL_MAX_DELAY);
}

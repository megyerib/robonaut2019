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

#define G 9.81f

#define REGS_TO_READ (12u)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static int16_t tmp_accel[3];
static int16_t tmp_angvel[3];

static int16_t ret_accel[3];
static int16_t ret_angvel[3];

static uint8_t regSource[REGS_TO_READ];
static uint8_t* regDest[REGS_TO_READ];

static int i;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data);
static void fillRegDest();

// Global function definitions -----------------------------------------------------------------------------------------

void inertInit()
{
	fillRegDest();

	HAL_GPIO_WritePin(INRT_SD0_GPIO_Port, INRT_SD0_Pin, GPIO_PIN_RESET); // Pull down SD0
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_SET); // CS (Stays active 4ever)

	// Enable accelerometer
	WriteRegBlocking(CTRL9_XL, 0x38); // Acc X, Y, Z axes enabled
	WriteRegBlocking(CTRL1_XL, 0x6C); // Acc = 416Hz (High-Performance mode); Full scale: +-8g

	// Enable gyroscope
	WriteRegBlocking(CTRL10_C, 0x38); // Gyro X, Y, Z axes enabled
	WriteRegBlocking(CTRL2_G, 0x60);  // Gyro = 416Hz (High-Performance mode)

	// TODO enable interrupt

}

ACCEL inertGetAccel()
{
    ACCEL ret;

    ret.a_x = ret_accel[0] / -4096.0f * G;
    ret.a_y = ret_accel[1] / -4096.0f * G;
    ret.a_z = ret_accel[2] / -4096.0f * G;

	return ret;
}

ANGVEL inertGetAngVel()
{
	ANGVEL ret;
	return ret;
}

// Sensor reading ------------------------------------------

void inertTriggerMeasurement()
{
	i = 0;
	HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSource[i], 1, regDest[i], 1);
}

void i2cInertialSensorMemRxCallback()
{
	i++;

	if (i < REGS_TO_READ)
	{
		HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSource[i], 1, regDest[i], 1);
	}
	else // Sensor reading finished
	{
	    __disable_irq(); // Cheap thread safety

		// Copy the temporary values to their final location
		int j;

		for (j = 0; j < 3; j++)
		{
			ret_accel[j]  = tmp_accel[j];
			ret_angvel[j] = tmp_angvel[j];
		}

		__enable_irq();
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data)
{
	// Assuming SD0 is pulled down
	HAL_I2C_Mem_Write(INERTIAL_I2C, LSM6DS3_ADDR0, regAddr, 1, &data, 1, HAL_MAX_DELAY);
}

static void fillRegDest()
{
	regSource[0] = OUTX_L_XL; // Accelerometer X
	regSource[1] = OUTX_H_XL;
	regSource[2] = OUTY_L_XL; // Accelerometer Y
	regSource[3] = OUTY_H_XL;
	regSource[4] = OUTZ_L_XL; // Accelerometer Z
	regSource[5] = OUTZ_H_XL;

	regSource[6] = OUTX_L_G;  // Gyroscope X
	regSource[7] = OUTX_H_G;
	regSource[8] = OUTY_L_G;  // Gyroscope Y
	regSource[9] = OUTY_H_G;
	regSource[10] = OUTZ_L_G; // Gyroscope Z
	regSource[11] = OUTZ_H_G;

	regDest[0]  = &((uint8_t*) &tmp_accel)[2]; // Accelerometer Y
	regDest[1]  = &((uint8_t*) &tmp_accel)[3];
	regDest[2]  = &((uint8_t*) &tmp_accel)[0]; // Accelerometer X
	regDest[3]  = &((uint8_t*) &tmp_accel)[1];
	regDest[4]  = &((uint8_t*) &tmp_accel)[4]; // Accelerometer Z
	regDest[5]  = &((uint8_t*) &tmp_accel)[5];

	regDest[6]  = &((uint8_t*) &tmp_angvel)[2]; // Gyroscope Y
	regDest[7]  = &((uint8_t*) &tmp_angvel)[3];
	regDest[8]  = &((uint8_t*) &tmp_angvel)[0]; // Gyroscope X
	regDest[9]  = &((uint8_t*) &tmp_angvel)[1];
	regDest[10] = &((uint8_t*) &tmp_angvel)[4]; // Gyroscope Z
	regDest[11] = &((uint8_t*) &tmp_angvel)[5];
}

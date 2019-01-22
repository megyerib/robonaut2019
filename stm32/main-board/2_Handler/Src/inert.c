////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      inert.c
//!  \brief     Inertial sensor handler
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// XL: Accelerometer
//  G: Gyroscope

// Includes ------------------------------------------------------------------------------------------------------------

#include "inert.h"
#include "LSM6DS3.h"
#include "bsp_i2c.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define G         (9.81f)
#define XL_CAL    (9.81f / 9.92f) /* Rough calibration */
#define XL_1G_VAL (4096.0f)
#define XL_C      (G * XL_CAL / XL_1G_VAL)

#define G_C       (1.0f)

#define REGS_TO_READ (12u)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static const uint8_t regSource[REGS_TO_READ] =
{
	OUTX_L_XL, OUTX_H_XL, // Accelerometer X
	OUTY_L_XL, OUTY_H_XL, // Accelerometer Y
	OUTZ_L_XL, OUTZ_H_XL, // Accelerometer Z
	OUTX_L_G,  OUTX_H_G,  // Gyroscope X
	OUTY_L_G,  OUTY_H_G,  // Gyroscope Y
	OUTZ_L_G,  OUTZ_H_G   // Gyroscope Z
};

static int16_t tmp_reading[6];

static int16_t ret_accel[3];
static int16_t ret_angvel[3];

static int i_reg;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data);

// Global function definitions -----------------------------------------------------------------------------------------

void inertInit()
{
	HAL_GPIO_WritePin(INRT_SD0_GPIO_Port, INRT_SD0_Pin, GPIO_PIN_RESET); // Pull down SD0
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_SET); // CS (Stays active 4ever)

	// Enable accelerometer
	WriteRegBlocking(CTRL9_XL, 0x38); // Acc X, Y, Z axes enabled
	WriteRegBlocking(CTRL1_XL, 0x6C); // Acc = 416Hz (High-Performance mode); Full scale: +-8g

	// Enable gyroscope
	WriteRegBlocking(CTRL10_C, 0x38); // Gyro X, Y, Z axes enabled
	WriteRegBlocking(CTRL2_G, 0x60);  // Gyro = 416Hz (High-Performance mode)
}

ACCEL inertGetAccel()
{
    ACCEL ret;

    ret.a_x = ret_accel[1] * -XL_C; // X = -Y
    ret.a_y = ret_accel[0] *  XL_C; // Y =  X
    ret.a_z = ret_accel[2] * -XL_C; // Z = -Z

	return ret;
}

ANGVEL inertGetAngVel()
{
	ANGVEL ret;

	// TODO
	ret.omega_x = ret_angvel[1] * -G_C;
	ret.omega_y = ret_angvel[0] *  G_C;
	ret.omega_z = ret_angvel[2] * -G_C;

	return ret;
}

void inertTriggerMeasurement()
{
	i_reg = 0;
	HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSource[i_reg], 1, &((uint8_t*) tmp_reading)[i_reg], 1);
}

// Callback functions --------------------------------------------------------------------------------------------------

void i2cInertialSensorMemRxCallback()
{
	i_reg++;

	if (i_reg < REGS_TO_READ)
	{
		HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSource[i_reg], 1, &((uint8_t*) tmp_reading)[i_reg], 1);
	}
	else // Sensor reading finished
	{
		// Copy the temporary values to their final location
		int j;

		for (j = 0; j < 3; j++)
		{
			ret_accel[j]  = tmp_reading[j];
			ret_angvel[j] = tmp_reading[j+3];
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data)
{
	// Assuming SD0 is pulled down
	HAL_I2C_Mem_Write(INERTIAL_I2C, LSM6DS3_ADDR0, regAddr, 1, &data, 1, HAL_MAX_DELAY);
}

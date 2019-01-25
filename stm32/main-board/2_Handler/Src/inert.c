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

#include "handler_common.h"
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

static float Xgain = 1.0;
static float Xofs  = 0.0;
static float XtoY  = 0.0;
static float XtoZ  = 0.0;
static float Ygain = 1.0;
static float Yofs  = 0.0;
static float YtoX  = 0.0;
static float YtoZ  = 0.0;
static float Zgain = 1.0;
static float Zofs  = 0.0;
static float ZtoX  = 0.0;
static float ZtoY  = 0.0;

static cMATRIX_3X3 invParams;

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

	invParams.a1[0] = 1.0;
	invParams.a1[1] = 0.0;
	invParams.a1[2] = 0.0;
	invParams.a2[0] = 0.0;
	invParams.a2[1] = 1.0;
	invParams.a2[2] = 0.0;
	invParams.a3[0] = 0.0;
	invParams.a3[1] = 0.0;
	invParams.a3[2] = 1.0;

}

ACCEL inertGetAccel()
{
    ACCEL measuredAcc;
    ACCEL trueAcc;

    // Changing directions to match the car's orientation.
    measuredAcc.a_x = ret_accel[1] * -XL_C; // X = -Y
    measuredAcc.a_y = ret_accel[0] *  XL_C; // Y =  X
    measuredAcc.a_z = ret_accel[2] * -XL_C; // Z = -Z

    trueAcc.a_x =   invParams.a1[0] * (measuredAcc.a_x - Xofs)
    			  + invParams.a1[1] * (measuredAcc.a_y - Yofs)
				  + invParams.a1[2] * (measuredAcc.a_z - Zofs);
    trueAcc.a_y =   invParams.a2[0] * (measuredAcc.a_x - Xofs)
    			  + invParams.a2[1] * (measuredAcc.a_y - Yofs)
				  + invParams.a2[2] * (measuredAcc.a_z - Zofs);
    trueAcc.a_z = 	invParams.a3[0] * (measuredAcc.a_x - Xofs)
    		      + invParams.a3[1] * (measuredAcc.a_y - Yofs)
				  + invParams.a3[2] * (measuredAcc.a_z - Zofs);

	return trueAcc;
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

void inert6PointCalibration(
								const float pXgain,
								const float pXofs,
								const float pXtoY,
								const float pXtoZ,
								const float pYgain,
								const float pYofs,
								const float pYtoX,
								const float pYtoZ,
								const float pZgain,
								const float pZofs,
								const float pZtoX,
								const float pZtoY
							)
{
	cMATRIX_3X3 Params;

	Xgain = pXgain;
	Xofs  = pXofs;
	XtoY  = pXtoY;
	XtoZ  = pXtoZ;
	Ygain = pYgain;
	Yofs  = pYofs;
	YtoX  = pYtoX;
	YtoZ  = pYtoZ;
	Zgain = pZgain;
	Zofs  = pZofs;
	ZtoX  = pZtoX;
	ZtoY  = pZtoY;

	Params.a1[0] = pXgain;
	Params.a1[1] = pYtoX;
	Params.a1[2] = pZtoX;
	Params.a2[0] = pXtoY;
	Params.a2[1] = pYgain;
	Params.a2[2] = pZtoY;
	Params.a3[0] = pXtoZ;
	Params.a3[1] = pYtoZ;
	Params.a3[2] = pZgain;

	invParams = hndlMatrixInversion(Params);
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

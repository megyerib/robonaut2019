////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      inert.c
//!  \brief     Inertial sensor handler
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// XL: Accelerometer
//  G: Gyroscope

// Gyro:
// 100 dps = +11428
// Magic constant = 114.28f

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

#define G_C       (0.00875043752f) /* 1/114.28 */
#define G_C_d     (0.00875043752) /* 1/114.28 */

//#define REGS_TO_READ (12u)
#define REGS_TO_READ (2u)

#define AVG_BUFF_SIZE 	(10)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static const uint8_t regSource[REGS_TO_READ] =
{
	/*OUTX_L_XL, OUTX_H_XL, // Accelerometer X
	OUTY_L_XL, OUTY_H_XL, // Accelerometer Y
	OUTZ_L_XL, OUTZ_H_XL, // Accelerometer Z
	OUTX_L_G,  OUTX_H_G,  // Gyroscope X
	OUTY_L_G,  OUTY_H_G,  // Gyroscope Y*/
	OUTZ_L_G,  OUTZ_H_G   // Gyroscope Z
};

//static int16_t tmp_reading[6];
static int16_t angvel_z;

static int i_reg = 0;

// Calibration parameters for the acceleration measurement.
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

// Calibration data (inverted) to calculate the true acceleration from the measured ones.
static cMATRIX_3X3 invParamsAcc;
static cMATRIX_3X3 invParamsAng;

// Gyroscope offsets.
static double WxOfs;
static double WyOfs;
static double WzOfs;

//
static ANGVELd bufferPsi[AVG_BUFF_SIZE];
static uint8_t load = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data);
static ANGVELd inertPutInAvgBuffer (ANGVELd new, uint8_t size);

// Global function definitions -----------------------------------------------------------------------------------------

void inertInit()
{
	HAL_GPIO_WritePin(INRT_SD0_GPIO_Port, INRT_SD0_Pin, GPIO_PIN_RESET); // Pull down SD0
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_SET); // CS (Stays active 4ever)

	/*// Enable accelerometer
	WriteRegBlocking(CTRL9_XL, 0x38); // Acc X, Y, Z axes enabled
	WriteRegBlocking(CTRL1_XL, 0x6C); // Acc = 416Hz (High-Performance mode); Full scale: +-8g*/

	// Enable gyroscope
	WriteRegBlocking(CTRL10_C, 0x38); // Gyro X, Y, Z axes enabled
	WriteRegBlocking(CTRL2_G, 0x80);  // Gyro = 1.66kHz (High-Performance mode), Full scale: +-250dps

	// Last valid calibration:
	//               - 2019.01.25. 12:23 by Joci
	// acceleration:
	invParamsAcc.a1[0] = 0.103275834;
	invParamsAcc.a1[1] = -0.00021038;
	invParamsAcc.a1[2] = 0.000890583;
	invParamsAcc.a2[0] = 0.000563969;
	invParamsAcc.a2[1] = 0.104217378;
	invParamsAcc.a2[2] = 0.00234628;
	invParamsAcc.a3[0] = 0.002363388;
	invParamsAcc.a3[1] = -0.003682367;
	invParamsAcc.a3[2] = 0.103042739;
	//
	// gyroscope:
	invParamsAng.a1[0] = 1;
	invParamsAng.a1[1] = 0;
	invParamsAng.a1[2] = 0;
	invParamsAng.a2[0] = 0;
	invParamsAng.a2[1] = 1;
	invParamsAng.a2[2] = 0;
	invParamsAng.a3[0] = 0;
	invParamsAng.a3[1] = 0;
	invParamsAng.a3[2] = 1;

	WxOfs = 0;
	WyOfs = 0;
	WzOfs = 0;
}

ACCEL inertGetAccel()
{
    ACCEL trueAcc;
    /*ACCEL measuredAcc;

    // Changing directions to match the car's orientation.
    measuredAcc.a_x = ret_accel[1] * -XL_C; // X =  Y
    measuredAcc.a_y = ret_accel[0] * -XL_C; // Y =  X
    measuredAcc.a_z = ret_accel[2] * XL_C;  // Z =  Z

    // Calculate the true acceleration from the measured ones with the calibration parameters.
    trueAcc.a_x =   invParamsAcc.a1[0] * (measuredAcc.a_x - Xofs)
    			  + invParamsAcc.a1[1] * (measuredAcc.a_y - Yofs)
				  + invParamsAcc.a1[2] * (measuredAcc.a_z - Zofs);
    trueAcc.a_y =   invParamsAcc.a2[0] * (measuredAcc.a_x - Xofs)
    			  + invParamsAcc.a2[1] * (measuredAcc.a_y - Yofs)
				  + invParamsAcc.a2[2] * (measuredAcc.a_z - Zofs);
    trueAcc.a_z = 	invParamsAcc.a3[0] * (measuredAcc.a_x - Xofs)
    		      + invParamsAcc.a3[1] * (measuredAcc.a_y - Yofs)
				  + invParamsAcc.a3[2] * (measuredAcc.a_z - Zofs);*/

	return trueAcc;
}

ANGVELd inertGetAngVel()
{
	ANGVELd measuredAngVel;
	ANGVELd trueAngVel;

	/*measuredAngVel.omega_x = ret_angvel[1] * -G_C_d;
	measuredAngVel.omega_y = ret_angvel[0] *  G_C_d;
	measuredAngVel.omega_z = ret_angvel[2] * -G_C_d;*/

	measuredAngVel.omega_z = angvel_z * -G_C_d;

	// Get rid of the offset error. Calibration was made in standing-still only.
	/*trueAngVel.omega_x =    invParamsAng.a1[0] * (measuredAngVel.omega_x - WxOfs)
    					  + invParamsAng.a1[1] * (measuredAngVel.omega_y - WyOfs)
						  + invParamsAng.a1[2] * (measuredAngVel.omega_z - WzOfs);
	trueAngVel.omega_y =    invParamsAng.a2[0] * (measuredAngVel.omega_x - WxOfs)
    					  + invParamsAng.a2[1] * (measuredAngVel.omega_y - WyOfs)
						  + invParamsAng.a2[2] * (measuredAngVel.omega_z - Zofs);
	trueAngVel.omega_z = 	invParamsAng.a3[0] * (measuredAngVel.omega_x - WxOfs)
    		    		  + invParamsAng.a3[1] * (measuredAngVel.omega_y - WyOfs)
						  + invParamsAng.a3[2] * (measuredAngVel.omega_z - WzOfs);*/

	//trueAngVel.omega_x = measuredAngVel.omega_x - WxOfs;
	//trueAngVel.omega_y = measuredAngVel.omega_y - WyOfs;
	trueAngVel.omega_z = measuredAngVel.omega_z - WzOfs;


	trueAngVel = inertPutInAvgBuffer(trueAngVel, AVG_BUFF_SIZE);

	return trueAngVel; //TODO
}

int inertTriggerMeasurement()
{
	if (i_reg == 0) // Have we finished the last read sequence?
	{
		HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSource[i_reg], 1, &((uint8_t*) &angvel_z)[i_reg], 1);
		return 1;
	}
	else
	{
		return 0;
	}
}

void inertGyroOffsetCalibration (const ANGVELd ofs)
{
	WxOfs += ofs.omega_x;
	WyOfs += ofs.omega_y;
	WzOfs += ofs.omega_z;
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

	invParamsAcc = hndlMatrixInversion(Params);
}

// Callback functions --------------------------------------------------------------------------------------------------

void i2cInertialSensorMemRxCallback()
{
	i_reg++;

	if (i_reg < REGS_TO_READ)
	{
		HAL_I2C_Mem_Read_IT(INERTIAL_I2C, LSM6DS3_ADDR0, regSource[i_reg], 1, &((uint8_t*) &angvel_z)[i_reg], 1);
	}
	else // Sensor reading finished
	{
		// Copy the temporary values to their final location
		/*int j;

		for (j = 0; j < 3; j++)
		{
			ret_accel[j]  = tmp_reading[j];
			ret_angvel[j] = tmp_reading[j+3];
		}*/

		i_reg = 0;
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void WriteRegBlocking(uint8_t regAddr, uint8_t data)
{
	// Assuming SD0 is pulled down
	HAL_I2C_Mem_Write(INERTIAL_I2C, LSM6DS3_ADDR0, regAddr, 1, &data, 1, HAL_MAX_DELAY);
}

static ANGVELd inertPutInAvgBuffer (ANGVELd new, uint8_t size)
{
	ANGVELd avg;
	int i;
	//float threshold = 0.8f;

	avg.omega_x = 0;
	avg.omega_y = 0;
	avg.omega_z = 0;

	if (load < size)
	{
		load++;
	}

	for (int i = 0; i < load-1; i++)
	{
		bufferPsi[i+1].omega_x = bufferPsi[i].omega_x;
		bufferPsi[i+1].omega_y = bufferPsi[i].omega_y;
		bufferPsi[i+1].omega_z = bufferPsi[i].omega_z;
	}

	if (load == size)
	{
		/*if (new.omega_x < threshold * avg.omega_x)
		{
			bufferPsi[0].omega_x = new.omega_x;
		}
		else
		{
			bufferPsi[0].omega_x = avg.omega_x;
		}

		if (new.omega_y < threshold * avg.omega_y)
		{
			bufferPsi[0].omega_y = new.omega_y;
		}
		else
		{
			bufferPsi[0].omega_y = avg.omega_y;
		}

		if (new.omega_z < threshold * avg.omega_z)
		{
			bufferPsi[0].omega_z = new.omega_z;
		}
		else
		{
			bufferPsi[0].omega_z = avg.omega_z;
		}*/

		bufferPsi[0].omega_x = new.omega_x;
		bufferPsi[0].omega_y = new.omega_y;
		bufferPsi[0].omega_z = new.omega_z;
	}
	else
	{
		bufferPsi[0].omega_x = new.omega_x;
		bufferPsi[0].omega_y = new.omega_y;
		bufferPsi[0].omega_z = new.omega_z;
	}


	for (i = 0; i < load; i++)
	{
		avg.omega_x += bufferPsi[i].omega_x;
		avg.omega_y += bufferPsi[i].omega_y;
		avg.omega_z += bufferPsi[i].omega_z;
	}

	avg.omega_x /= load;
	avg.omega_y /= load;
	avg.omega_z /= load;

	return avg;
}

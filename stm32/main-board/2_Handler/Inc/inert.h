////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      inert.h
//!  \brief     Inertial sensor functions
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	float a_x;
	float a_y;
	float a_z;
}
ACCEL;

typedef struct
{
	float omega_x;
	float omega_y;
	float omega_z;
}
ANGVEL;

typedef struct
{
	double omega_x;
	double omega_y;
	double omega_z;
}
ANGVELd;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void inertInit();
ACCEL inertGetAccel();
ANGVELd inertGetAngVel();

int inertTriggerMeasurement();

void inertGyroOffsetCalibration (const ANGVELd ofs);

void inert6PointCalibration(
								const float Xgain,
								const float Xofs,
								const float XtoY,
								const float XtoZ,
								const float Ygain,
								const float Yofs,
								const float YtoX,
								const float YtoZ,
								const float Zgain,
								const float Zofs,
								const float ZtoX,
								const float ZtoY
							);

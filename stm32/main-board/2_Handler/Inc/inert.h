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

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void inertInit();
ACCEL inertGetAccel();
ANGVEL inertGetAngVel();
void inertTriggerMeasurement();

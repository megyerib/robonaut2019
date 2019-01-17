////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		navigation.h
//!  \brief
//!  \details  	This module is responsible for the navigation of the car. This module implements the simple Dead
//! 			Reckoning algorithm.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "semphr.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	float u;
	float v;
} cVEC_ACCEL;

typedef struct
{
	float u;
	float v;
} cVelocityVector;

typedef struct
{
	float n;
	float e;
} cNedParameters;

typedef struct
{
	cNedParameters p;
	float phi;
} cNAVI_STATE;

// Variables -----------------------------------------------------------------------------------------------------------

SemaphoreHandle_t semDrNavi;

// Function prototypes -------------------------------------------------------------------------------------------------

///*********************************************************************************************************************
//! Initializes the navigation module.
//!
//! Resets all of the module variables.
//!
//! @return -
///*********************************************************************************************************************
void naviDRInit (void);

///*********************************************************************************************************************
//! Calculates the actual navigation state from the sensor data.
//!
//! This function executes all of the calculations that are need to determine the actual navigation state from the
//! previous one. The function updates the current acceleration vector, the angular acceleration and the time quantum.
//! From these, the function calculates the orientation change and updates the previous value. With the integration of
//! the acceleration the previous velocity vector can be updated. From the new velocity vector and the new orientation,
//! the position can be updated. The function collects the new position and orientation value and returns them.
//!
//! @param a				[m^2/s]
//! @param omega			[rad/s]
//! @param dt				[ms]
//!
//! @return
///*********************************************************************************************************************
cNAVI_STATE naviDRNaviProcess (const cVEC_ACCEL a, const float omega, const uint32_t dt);


/* TODO Do we need this?
cNedParameters naviDRGetNedCoordinates (void);

void naviDRSetNedCoordinates (const cNedParameters coords);
*/

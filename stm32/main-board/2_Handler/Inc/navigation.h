////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		naviDeadReckoning.h
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
	double x;
	double y;
} cVelocityVector;

//TODO _Joci_ needs update
typedef struct
{
	double omega;
	double time;
} cAngularVelocity;

typedef struct
{
	double n;
	double e;
} cNedParameters;

// Variables -----------------------------------------------------------------------------------------------------------

SemaphoreHandle_t semDrNavi;

// Function prototypes -------------------------------------------------------------------------------------------------

void naviDRInit (void);

cNedParameters naviDRGetNedCoordinates (void);

void naviDRSetNedCoordinates (const cNedParameters coords);

//									[m/s]							[rad/s]					[ms]
cNedParameters naviDRNavigate (const cVelocityVector v, const cAngularVelocity w, const uint32_t dt);

double naviDRNumIntegTrapezoidal (const double a, const double b, const double fa, const double fb);



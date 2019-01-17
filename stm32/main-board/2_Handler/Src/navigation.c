////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		navigation.c
//!  \brief
//!  \details	This module is responsible for the navigation of the car. This module implements the simple Dead
//! 			Reckoning algorithm.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------


#include "navigation.h"
#include "math.h"
#include "handler_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

// TODO move to hndlCommon.h and include to sharp too.
#define 	WAIT_SEMAPHORE			100		//!< Ticks [ms]

// Typedefs ------------------------------------------------------------------------------------------------------------



// Local (static) & extern variables -----------------------------------------------------------------------------------

static cVEC_ACCEL prevA;
static cVelocityVector prevV;
static cNedParameters prevP;
static float prevOmega;
static float prevPhi;

static cVEC_ACCEL currA;
static cVelocityVector currV;
static cNedParameters currP;
static float currOmega;
static float currPhi;

static float dt_s;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void naviUpdateOrientation (void);
static void naviUpdateVelocity 	  (void);
static void naviUpdatePosition    (void);

// Global function definitions -----------------------------------------------------------------------------------------

void naviDRInit (void)
{
	prevA.u = 0;
	prevA.v = 0;
	prevV.u = 0;
	prevV.v = 0;
	prevP.n = 0;
	prevP.e = 0;
	prevOmega = 0;
	prevPhi = 0;

	currA.u = 0;
	currA.v = 0;
	currV.u = 0;
	currV.v = 0;
	currP.n = 0;
	currP.e = 0;
	currOmega = 0;
	currPhi = 0;

	dt_s = 1;
}

cNAVI_STATE naviDRNaviProcess (const cVEC_ACCEL a, const float omega, const uint32_t dt)
{
	cNAVI_STATE retval;

	// Save the current sensor data.
	currA = a;
	currOmega = omega;

	// Convert time to seconds.
	dt_s = (float)dt / 1000;

	// Calculate orientation.
	naviUpdateOrientation();

	// Calculate velocity.
	naviUpdateVelocity();

	// Calculate position.
	naviUpdatePosition();

	// Construct navigation state vector.
	retval.p = currP;
	retval.phi = currPhi;

	return retval;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// omega = [rad/s], dt = [s] -> phi = [rad]
static void naviUpdateOrientation ()
{
	float dPhi;

	// Calculate the derivative of the new theta form the current and the last omega value
	dPhi = hndlNumIntegTrapezoidal(0, dt_s, prevOmega, currOmega);

	// Determine the current theta value
	currPhi = prevPhi + dPhi;

	// Save the current parameters as previous measurement and state for the next iteration
	prevOmega = currOmega;
	prevPhi   = currPhi;
}

// u, v = [m/s]
static void naviUpdateVelocity ()
{
	cVelocityVector dv;

	// Calculate the velocity change since the last value.
	dv.u = hndlNumIntegTrapezoidal(0, dt_s, prevA.u, currA.u);
	dv.v = hndlNumIntegTrapezoidal(0, dt_s, prevA.v, currA.v);

	// Calculate the new velocity.
	currV.u = prevV.u + dv.u;
	currV.v = prevV.v + dv.v;

	// Save the result for the next calculation.
	prevA = currA;
	prevV = currV;
}

// dt = [s] -> n = [m], e = [m]
static void naviUpdatePosition (void)
{
	float grad_n;
	float grad_e;

	// Determine the derivative of the NED coordinations.
	grad_n = cos(currPhi) * currV.u - sin(currPhi) * currV.v;
	grad_e = sin(currPhi) * currV.u + cos(currPhi) * currV.v;

	// Calculate the new NED coordinates with the previous results.
	currP.n = prevP.n + dt_s * grad_n;
	currP.e = prevP.e + dt_s * grad_e;

	// Save the new result
	prevP = currP;
}


/* TODO Do we need this?
cNedParameters naviDRGetNedCoordinates (void)
{
	cNedParameters ned;

	if (xSemaphoreTake(semDrNavi, WAIT_SEMAPHORE) == pdTRUE)
	{
		ned.n = nedPosition.n;
		ned.e = nedPosition.e;
		xSemaphoreGive(semDrNavi);
	}

	return ned;
}

void naviDRSetNedCoordinates (const cNedParameters coords)
{
	if (xSemaphoreTake(semDrNavi, WAIT_SEMAPHORE) == pdTRUE)
	{
		nedPosition.n = coords.n;
		nedPosition.e = coords.e;
		xSemaphoreGive(semDrNavi);
	}
}
*/

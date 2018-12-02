////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		naviDeadReckoning.c
//!  \brief
//!  \details	This module is responsible for the navigation of the car. This module implements the simple Dead
//! 			Reckoning algorithm.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------


#include "naviDeadReckoning.h"
#include "math.h"

// Defines -------------------------------------------------------------------------------------------------------------

// TODO move to hndlCommon.h and include to sharp too.
#define 	WAIT_SEMAPHORE			100		//!< Ticks [ms]

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	double theta;
	cVelocityVector v;
} cStrapDownNavigation;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static cStrapDownNavigation naviOffset;

static cStrapDownNavigation naviState;
static cStrapDownNavigation prevNaviState;

static cAngularVelocity prevMeasure;

static cNedParameters nedPosition;
static cNedParameters prevNedPosition;

// Local (static) function prototypes ----------------------------------------------------------------------------------

//static const double drn_NumInteg_Trapezoidal (const double a, const double b, const double fa, const double fb);

//static void drn_UpdateOrientation (const double omega, const double time);

static void drnUpdateOrientation (const double omega, const double dt);

static void drnUpdateVelocity (const double u, const double v);

static void drnCalculateNedParameters (const double dt);

// Global function definitions -----------------------------------------------------------------------------------------

void naviDRInit (void)
{
	naviOffset.theta = 0;
	naviOffset.v.x = 0;
	naviOffset.v.y = 0;

	//drn_SetOrientation(naviOffset.theta);

	prevMeasure.omega = 0;
	prevMeasure.time = 0;

	naviState.theta = 0;
	naviState.v.x = 0;
	naviState.v.y = 0;

	prevNaviState.theta = 0;
	prevNaviState.v.x = 0;
	prevNaviState.v.y = 0;

	nedPosition.n = 0;
	nedPosition.e = 0;

	prevNedPosition.n = 0;
	prevNedPosition.e = 0;
}

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

// v = [m/s], w = [rad/s], dt = [ms]
cNedParameters naviDRNavigate (const cVelocityVector v, const cAngularVelocity w, const uint32_t dt)
{
	cNedParameters retval;

	//Convert time to sec
	double dt_s = (double)dt / 1000;

	drnUpdateOrientation(w.omega, dt_s);
	drnUpdateVelocity(v.x, v.y);
	drnCalculateNedParameters(dt_s);

	retval = naviDRGetNedCoordinates();

	return retval;
}

double naviDRNumIntegTrapezoidal (const double a, const double b, const double fa, const double fb)
{
	double integral;

	integral = (b - a) * (fb + fa) / 2;

	return integral;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

/*static void drn_UpdateOrientation (const double omega, const double time)
{
	double lastOmega;
	double lastTime;
	double theta;

	// Get previous values
	lastOmega = prevMeasure.omega;
	lastTime = prevMeasure.time;

	// Calculate current theta and save it
	theta = drn_NumInteg_Trapezoidal(lastTime, time, lastOmega, omega);
	naviState.theta = theta;

	// Save the current parameters as previous measurement
	prevMeasure.omega = omega;
	prevMeasure.time = time;
}*/

// omega = [rad/s], dt = [s] -> theta = [rad]
static void drnUpdateOrientation (const double omega, const double dt)
{
	volatile double dtheta;

	// Calculate the derivative of the new theta form the current and the last omega value
	dtheta = naviDRNumIntegTrapezoidal(0, dt, prevMeasure.omega, omega);

	// Determine the current theta value
	naviState.theta = prevNaviState.theta + dtheta;

	// Save the current parameters as previous measurement and state for the next iteration
	prevMeasure.omega = omega;
	prevNaviState.theta = naviState.theta;
}

// u, v = [m/s]
static void drnUpdateVelocity (const double u, const double v)
{
	naviState.v.x = u;
	naviState.v.y = v;
}

// dt = [s] -> n = [m], e = [m]
static void drnCalculateNedParameters (const double dt)
{
	volatile double delta_n;
	volatile double delta_e;
	volatile cNedParameters coords;

	// Determine the derivative of the NED coordinations
	delta_n = naviState.v.x*cos(naviState.theta) - naviState.v.y*sin(naviState.theta);
	delta_e = naviState.v.x*sin(naviState.theta) + naviState.v.y*cos(naviState.theta);

	// Calculate the new NED coordinates with the previous results
	coords.n = prevNedPosition.n + dt * delta_n;
	coords.e = prevNedPosition.e + dt * delta_e;

	// Save the new result as previous for the next iteration
	prevNedPosition.n = coords.n;
	prevNedPosition.e = coords.e;

	// Save the new result
	naviDRSetNedCoordinates(coords);
}

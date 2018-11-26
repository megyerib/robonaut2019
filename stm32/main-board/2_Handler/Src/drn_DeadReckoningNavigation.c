/*
 * drn_DeadReckoningNavigation.c
 *
 *  Created on: 2018. nov. 9.
 *      Author: Joci
 */

// ------------------------------- Includes -------------------------------- //

#include "../../2_Handler/Inc/drn_DeadReckoningNavigation.h"

#include "math.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

typedef struct
{
	double theta;
	cVelocityVector v;
} StrapDownNavigation;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

//static const double drn_NumInteg_Trapezoidal (const double a, const double b, const double fa, const double fb);

//static void drn_UpdateOrientation (const double omega, const double time);

static void drnUpdateOrientation (const double omega, const double dt);

static void drnUpdateVelocity (const double u, const double v);

static void drnCalculateNedParameters (const double dt);

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

StrapDownNavigation naviOffset;

StrapDownNavigation naviState;
StrapDownNavigation prevNaviState;

cAngularVelocity		prevMeasure;

cNedParameters		nedPosition;
cNedParameters		prevNedPosition;

// --------------------------------------------------------------------------//

// ------------------------------ Functions ---------------------------------//

void drnInit (void)
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

cNedParameters drnGetNedCoordinates (void)
{
	cNedParameters ned;

	xSemaphoreTake(semDrNavi, portMAX_DELAY);
	ned.n = nedPosition.n;
	ned.e = nedPosition.e;
	xSemaphoreGive(semDrNavi);

	return ned;
}

void drnSetNedCoordinates (const cNedParameters coords)
{
	xSemaphoreTake(semDrNavi, portMAX_DELAY);
	nedPosition.n = coords.n;
	nedPosition.e = coords.e;
	xSemaphoreGive(semDrNavi);
}

// v = [m/s], w = [rad/s], dt = [ms]
cNedParameters drnReckonNavigation (const cVelocityVector v, const cAngularVelocity w, const uint32_t dt)
{
	cNedParameters retval;

	//Convert time to sec
	double dt_s = (double)dt / 1000;

	drnUpdateOrientation(w.omega, dt_s);
	drnUpdateVelocity(v.x, v.y);
	drnCalculateNedParameters(dt_s);

	retval = drnGetNedCoordinates();

	return retval;
}

double drnNumIntegTrapezoidal (const double a, const double b, const double fa, const double fb)
{
	double integral;

	integral = (b - a) * (fb + fa) / 2;

	return integral;
}

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
	dtheta = drnNumIntegTrapezoidal(0, dt, prevMeasure.omega, omega);

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
	drnSetNedCoordinates(coords);
}

// --------------------------------------------------------------------------//

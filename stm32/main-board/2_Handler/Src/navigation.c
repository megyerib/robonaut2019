////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		navigation.c
//!  \brief		This module calculates the navigation informations from the inertial sensor and from the encoder.
//!  \details	This module is responsible for the navigation of the car. This module implements the simple Dead
//! 			Reckoning algorithm.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "navigation.h"
#include "handler_common.h"
#include "bsp_common.h"
#include <math.h>

// Defines -------------------------------------------------------------------------------------------------------------

#define   	NAVI_PI					(3.14159265359)
//! Constant used to convert the gravitational acceleration given in gravitational force [g] to SI [m2/s].
#define     NAVI_F_GRAVY_TO_SI      (9.80665)
//! Constant used to convert the angular velocity given in degree per seconds [dps] to SI [rad/s].
#define     NAVI_DPS_TO_SI          (double)(NAVI_PI/180.0)

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

static cVEC_ACCEL prevA;
static cVEC_VEL prevV;
static cNED_COORD prevP;
static double prevOmega;
static double prevPhi;

static cVEC_ACCEL currA;
static cVEC_VEL currV;
static cNED_COORD currP;
static double currOmega;
static double currPhi;

static double dt_s;

static ANGVELd W;
static double Phi;
static double Theta;
static double Psi;
static double dPhi;
static double dTheta;
static double dPsi;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static cNAVI_STATE naviDRProcessInertial      (const cVEC_ACCEL a, const double omega, const uint32_t dt);
static cNAVI_STATE naviDRProcessInertialSTM32 (const cVEC_ACCEL a, const ANGVELd w, const uint32_t dt);

static cNAVI_STATE naviDRProcessIncremental      (const double v, const double omega, const uint32_t dt);
static cNAVI_STATE naviDRProcessIncrementalSTM32 (const double v, const ANGVELd w, const uint32_t dt);

static void  naviUpdateOrientationSTM32 (void);
static void  naviUpdateOrientation 	    (void);
static void  naviUpdateVelocity 	  	(void);
static void  naviUpdatePosition         (void);


// Global function definitions -----------------------------------------------------------------------------------------

void naviDRInit (void)
{
	prevA.u = 0.0;
	prevA.v = 0.0;
	prevV.u = 0.0;
	prevV.v = 0.0;
	prevP.n = 0.0;
	prevP.e = 0.0;
	prevOmega = 0.0;
	prevPhi = 0.0;

	currA.u = 0.0;
	currA.v = 0.0;
	currV.u = 0.0;
	currV.v = 0.0;
	currP.n = 0.0;
	currP.e = 0.0;
	currOmega = 0.0;
	currPhi = 0.0;

	dt_s = 1.0;

	Phi    = 0.0;
	Theta  = 0.0;
	Psi    = 0.0;
	dPhi   = 0.0;
	dTheta = 0.0;
	dPsi   = 0.0;
}

cNAVI_STATE naviGetNaviDataEnc (const double v, const ANGVELd w, const uint32_t dt, const eNAVI_ENC_MODE mode)
{
	cNAVI_STATE retVal;

	switch (mode)
	{
		case eNAVI_ENC:
		{
			retVal = naviDRProcessIncremental(v, w.omega_z, dt);
			break;
		}
		case eNAVI_ENC_GRAVY_CORR:
		{
			retVal = naviDRProcessIncrementalSTM32(v, w, dt);
			break;
		}
		default:
		{
			break;
		}
	}

	return retVal;
}

cNAVI_STATE naviGetNaviDataInrt (const cVEC_ACCEL a, const ANGVELd w, const uint32_t dt, const eNAVI_INERT_MODE mode)
{
	cNAVI_STATE retVal;

	switch (mode)
	{
		case eNAVI_INERT:
		{
			retVal = naviDRProcessInertial(a, w.omega_z, dt);
			break;
		}
		case eNAVI_INERT_GRAVY_CORR:
		{
			retVal = naviDRProcessInertialSTM32(a, w, dt);
			break;
		}
		default:
		{
			break;
		}
	}

	return retVal;
}

double naviConvertDpsToSI (const double ang_dps)
{
	return ang_dps * NAVI_DPS_TO_SI;
}

double naviConvertGToSI (const double accel_g)
{
	return accel_g * NAVI_F_GRAVY_TO_SI;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
double naviNormaliseOrientation (const double psi)
{
	double normOri;
	double temp = psi;

	if (temp >= 0.0)
	{
		// Positive.

		// Convert into [0; 2PI)
		while (temp >= 2.0 * NAVI_PI)
		{
			temp -= 2.0 * NAVI_PI;
		}

		normOri = temp;
	}
	else
	{
		// Negative.

		// Convert into (-2PI; 0]
		while (temp <= -2.0 * NAVI_PI)
		{
			temp += 2.0 * NAVI_PI;
		}

		normOri = temp + 2.0 * NAVI_PI;
	}

	return normOri;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//**********************************************************************************************************************
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
//**********************************************************************************************************************
static cNAVI_STATE naviDRProcessInertial (const cVEC_ACCEL a, const double omega, const uint32_t dt)
{
	cNAVI_STATE retVal;

	// Save the current sensor data.
	currA = a;
	currOmega = omega;

	// Convert time to seconds.
	dt_s = (double)dt / 1000.0;

	// Calculate orientation.
	naviUpdateOrientation();

	// Calculate velocity.
	naviUpdateVelocity();

	// Calculate position.
	naviUpdatePosition();

	// Construct navigation state vector.
	retVal.p   = currP;
	retVal.psi = currPhi;

	return retVal;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static cNAVI_STATE naviDRProcessInertialSTM32 (const cVEC_ACCEL a, const ANGVELd w, const uint32_t dt)
{
	cNAVI_STATE retVal;

	// Save the current sensor data.
	currA = a;
	W = w;

	// Convert time to seconds.
	dt_s = (double)dt / 1000;

	// Calculate orientation.
	naviUpdateOrientationSTM32();

	// Calculate velocity.
	naviUpdateVelocity();

	// Calculate position.
	naviUpdatePosition();

	// Construct navigation state vector.
	retVal.p   = currP;
	retVal.psi = currPhi;

	return retVal;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static cNAVI_STATE naviDRProcessIncremental (const double v, const double omega, const uint32_t dt)
{
	cNAVI_STATE retVal;

	// Save the current sensor data.
	currV.u = v;
	currV.v = 0.;
	currOmega = omega;

	// Convert time to seconds.
	dt_s = (double)dt / 1000.0;

	// Calculate orientation.
	naviUpdateOrientation();

	// Calculate position();
	naviUpdatePosition();

	// Construct navigation state vector.
	retVal.p   = currP;
	retVal.psi = currPhi;

	return retVal;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static cNAVI_STATE naviDRProcessIncrementalSTM32 (const double v, const ANGVELd w, const uint32_t dt)
{
	cNAVI_STATE retVal;

	// Save the current sensor data.
	currV.u = v;
	currV.v = 0.0;
	W = w;

	// Convert time to seconds.
	dt_s = (double)dt / 1000.0;

	// Calculate orientation.
	naviUpdateOrientationSTM32();

	// Calculate position();
	naviUpdatePosition();

	// Construct navigation state vector.
	retVal.p   = currP;
	retVal.psi = currPhi;

	return retVal;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static void naviUpdateOrientationSTM32 (void)
{
	////STEP 1
	// Calculate the derivatives of the angles.
	// NOTE: if Theta = +- 90deg, then tan(Theta) = +- inf (singularity points)

	// Roll derivative: Phi’ = Wx + Wy * Sin(Phi) * Tan(Theta) + Wz * Cos(Phi) * Tan(Theta)
	dPhi = W.omega_x + W.omega_y * sinf(Phi) * tanf(Theta) + W.omega_z * cosf(Phi) * tanf(Theta);

	// Pitch derivative: Theta’ = Wy * Cos(Phi) – Wz * Sin(Phi)
	dTheta = W.omega_y * cosf(Phi) - W.omega_z * sinf(Phi);

	// Yaw derivative: Psi’ = Wy * Sin(Phi) / Cos(Theta) + Wz * Cos(Phi) / Cos(Theta)
	dPsi = W.omega_y * sinf(Phi) / cosf(Theta) + W.omega_z * cosf(Phi) / cosf(Theta);

	////STEP 2
	// Update the angles.
	// Roll: Phi(t+Ts) = Phi(t) + Phi’ * Ts
	Phi = Phi + dPhi * dt_s;

	// Pitch: Theta(t+Ts) = Theta(t) + Theta’ * Ts
	Theta =  Theta + dTheta * dt_s;

	// Yaw: Psi(t+Ts) = Psi(t) + Psi’ * Ts
	Psi = Psi + dPsi * dt_s;

	////STEP 3
	// Get the Yaw angle and save for further use.
	prevPhi = naviNormaliseOrientation(Psi);
	currPhi = naviNormaliseOrientation(Psi);
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
// omega = [rad/s], dt = [s] -> phi = [rad]
static void naviUpdateOrientation (void)
{
	double dPhi;

	// Calculate the derivative of the new theta form the current and the last omega value
	dPhi = hndlNumIntegTrapezoidalDouble(0.0, dt_s, prevOmega, currOmega);

	// Determine the current theta value
	currPhi = prevPhi + dPhi;

	// Save the current parameters as previous measurement and state for the next iteration
	//prevOmega = naviNormaliseOrientation(currOmega);
	//prevPhi   = naviNormaliseOrientation(currPhi);
	prevOmega = currOmega;
	prevPhi = currPhi;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
// u, v = [m/s]
static void naviUpdateVelocity (void)
{
	cVEC_VEL dv;

	// Calculate the velocity change since the last value.
	dv.u = hndlNumIntegTrapezoidalDouble(0.0, dt_s, prevA.u, currA.u);
	dv.v = hndlNumIntegTrapezoidalDouble(0.0, dt_s, prevA.v, currA.v);

	// Calculate the new velocity.
	currV.u = prevV.u + dv.u;
	currV.v = prevV.v + dv.v;

	// Save the result for the next calculation.
	prevA = currA;
	prevV = currV;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
// dt = [s] -> n = [m], e = [m]
static void naviUpdatePosition (void)
{
	double grad_n;
	double grad_e;

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

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

//! Constant used to convert the gravitational acceleration given in gravitational force [g] to SI [m2/s].
#define     NAVI_F_GRAVY_TO_SI      (9.80665f)
//! Constant used to convert the angular velocity given in degree per seconds [dps] to SI [rad/s].
#define     NAVI_DPS_TO_SI          (float)(PI/180)

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

static cVEC_ACCEL prevA;
static cVEC_VEL prevV;
static cNED_COORD prevP;
static float prevOmega;
static float prevPhi;

static cVEC_ACCEL currA;
static cVEC_VEL currV;
static cNED_COORD currP;
static float currOmega;
static float currPhi;

static float dt_s;

static ANGVEL W;
static float Phi;
static float Theta;
static float Psi;
static float dPhi;
static float dTheta;
static float dPsi;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static cNAVI_STATE naviDRProcessInertial      (const cVEC_ACCEL a, const float omega, const uint32_t dt);
static cNAVI_STATE naviDRProcessInertialSTM32 (const cVEC_ACCEL a, const ANGVEL w, const uint32_t dt);

static cNAVI_STATE naviDRProcessIncremental      (const float v, const float omega, const uint32_t dt);
static cNAVI_STATE naviDRProcessIncrementalSTM32 (const float v, const ANGVEL w, const uint32_t dt);

static void  naviUpdateOrientationSTM32 (void);
static void  naviUpdateOrientation 	    (void);
static void  naviUpdateVelocity 	  	(void);
static void  naviUpdatePosition         (void);
static float naviNormaliseOrientation   (const float psi);

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

	Phi    = 0;
	Theta  = 0;
	Psi    = 0;
	dPhi   = 0;
	dTheta = 0;
	dPsi   = 0;
}

cNAVI_STATE naviGetNaviDataEnc (const float v, const ANGVEL w, const uint32_t dt, const eNAVI_ENC_MODE mode)
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

cNAVI_STATE naviGetNaviDataInrt (const cVEC_ACCEL a, const ANGVEL w, const uint32_t dt, const eNAVI_INERT_MODE mode)
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

float naviConvertDpsToSI (const float ang_dps)
{
	return ang_dps * NAVI_DPS_TO_SI;
}

float naviConvertGToSI (const float accel_g)
{
	return accel_g * NAVI_F_GRAVY_TO_SI;
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
static cNAVI_STATE naviDRProcessInertial (const cVEC_ACCEL a, const float omega, const uint32_t dt)
{
	cNAVI_STATE retVal;

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
	retVal.p   = currP;
	retVal.phi = currPhi;

	return retVal;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static cNAVI_STATE naviDRProcessInertialSTM32 (const cVEC_ACCEL a, const ANGVEL w, const uint32_t dt)
{
	cNAVI_STATE retVal;

	// Save the current sensor data.
	currA = a;
	W = w;

	// Convert time to seconds.
	dt_s = (float)dt / 1000;

	// Calculate orientation.
	naviUpdateOrientationSTM32();

	// Calculate velocity.
	naviUpdateVelocity();

	// Calculate position.
	naviUpdatePosition();

	// Construct navigation state vector.
	retVal.p   = currP;
	retVal.phi = currPhi;

	return retVal;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static cNAVI_STATE naviDRProcessIncremental (const float v, const float omega, const uint32_t dt)
{
	cNAVI_STATE retVal;

	// Save the current sensor data.
	currV.u = v;
	currV.v = 0;
	currOmega = omega;

	// Convert time to seconds.
	dt_s = (float)dt / 1000;

	// Calculate orientation.
	naviUpdateOrientation();

	// Calculate position();
	naviUpdatePosition();

	// Construct navigation state vector.
	retVal.p   = currP;
	retVal.phi = currPhi;

	return retVal;
}

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static cNAVI_STATE naviDRProcessIncrementalSTM32 (const float v, const ANGVEL w, const uint32_t dt)
{
	cNAVI_STATE retVal;

	// Save the current sensor data.
	currV.u = v;
	currV.v = 0;
	W = w;

	// Convert time to seconds.
	dt_s = (float)dt / 1000;

	// Calculate orientation.
	naviUpdateOrientationSTM32();

	// Calculate position();
	naviUpdatePosition();

	// Construct navigation state vector.
	retVal.p   = currP;
	retVal.phi = currPhi;

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
	float dPhi;

	// Calculate the derivative of the new theta form the current and the last omega value
	dPhi = hndlNumIntegTrapezoidal(0, dt_s, prevOmega, currOmega);

	// Determine the current theta value
	currPhi = prevPhi + dPhi;

	// Save the current parameters as previous measurement and state for the next iteration
	prevOmega = naviNormaliseOrientation(currOmega);
	prevPhi   = naviNormaliseOrientation(currPhi);
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
	dv.u = hndlNumIntegTrapezoidal(0, dt_s, prevA.u, currA.u);
	dv.v = hndlNumIntegTrapezoidal(0, dt_s, prevA.v, currA.v);

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

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
static float naviNormaliseOrientation (const float psi)
{
	float normOri;
	float temp = psi;

	if (temp >= 0.0f)
	{
		// Positive.

		// Convert into [0; 2PI)
		while (temp >= 2.0f*PI)
		{
			temp -= 2.0f*PI;
		}

		normOri = temp;
	}
	else
	{
		// Negative.

		// Convert into (-2PI; 0]
		while (temp <= -2.0f*PI)
		{
			temp += 2.0f*PI;
		}

		normOri = temp + 2.0f*PI;
	}

	return normOri;
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

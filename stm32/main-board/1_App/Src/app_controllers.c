////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_controllers.c
//!  \brief		This module contains all of the available controllers.
//!  \details	See in app_controllers.h.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include <math.h>	// TODO added

// Defines -------------------------------------------------------------------------------------------------------------

#define SPEED_TS				(5.0f)	//! Ts sampling time in ms (task period).
#define SPEED_CONTROL_VAR_MIN	(0.0f)	//!< u(t) control variable minimal value.
#define SPEED_CONTROL_VAR_MAX	(0.85f) //!< u(t) control variable maximal value.

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

//! Function: cntrlLineFollow
float cntrlLineFollow (const float actLine, const float prevLine, const float P, const float kP, const float kD)
{
	float line_diff;
	float servoAngle;

	// Detect line.
	line_diff = actLine - prevLine;

	// Control the servo.
	servoAngle = -0.75f * (actLine  * kP + line_diff * kD);

	return servoAngle;
}

//! Function: cntrSpeed
uint32_t cntrSpeed (const float r_speed, const float prevSpeed, const float actSpeed, const float Ti, float* fk, const float kc)
{
	float e_speed;
	float beta = expf(-SPEED_TS / Ti);	// TODO expf, SPEED_TS
	float uk;

	// v diff = v wanted - v actual
	e_speed = r_speed - actSpeed;

	// Calculate control variable.
	uk = kc * e_speed + *fk;

	// Saturation.
	if(uk < SPEED_CONTROL_VAR_MIN)
	{
		uk = SPEED_CONTROL_VAR_MIN;
	}
	if(uk > SPEED_CONTROL_VAR_MAX)
	{
		uk = SPEED_CONTROL_VAR_MAX;
	}

	// Update fk FOXBORO parameter.
	*fk = beta * (*fk) + ( 1 - beta * uk);

	return (uint32_t)(uk);
}

//! Function: cntrDistance
float cntrDistance (const float actDist, const float followDist, const float p, const float minSpeed, const float maxSpeed)
{
	float distDiff;
	float speed;

	// Calculate the error.
	distDiff = actDist - followDist;

	// Calculate process variable (new speed).
	speed = p * distDiff;

	// Saturate the result.
	if (minSpeed < 0)
	{
		speed = minSpeed;
	}
	else if (speed > maxSpeed)
	{
		speed = maxSpeed;
	}

	return speed;
}
// Local (static) function definitions ---------------------------------------------------------------------------------

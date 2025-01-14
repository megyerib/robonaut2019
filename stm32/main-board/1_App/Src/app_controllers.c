////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_controllers.c
//!  \brief		This module contains all of the available controllers.
//!  \details	See in app_controllers.h.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include <math.h>

// Defines -------------------------------------------------------------------------------------------------------------

#define SPEED_TS				(5.0f)	//! Ts sampling time in ms (task period).
#define SPEED_CONTROL_VAR_MIN	(-0.85f)//!< u(t) control variable minimal value.
#define SPEED_CONTROL_VAR_MAX	(0.85f) //!< u(t) control variable maximal value.
#define DELTA_UK_MAX            (0.05f)

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
float uk_prev = 0;
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
float cntrSpeed (const float r_speed, const float prevSpeed, const float actSpeed, const float Ti, float* fk, const float kc)
{
	float e_speed;
	float beta = expf(-SPEED_TS / Ti);
	float uk;

	// v diff = v wanted - v actual
	e_speed = r_speed - actSpeed;

	// Calculate control variable.
	uk = kc * e_speed + (*fk);

	/*if (uk > uk_prev + DELTA_UK_MAX)
	{
		uk = uk_prev + DELTA_UK_MAX;
	}
	else if (uk < uk_prev - DELTA_UK_MAX)
	{
		uk = uk_prev - DELTA_UK_MAX;
	}*/

	// Saturation.
	if(uk < SPEED_CONTROL_VAR_MIN)
	{
		uk = SPEED_CONTROL_VAR_MIN;
	}
	if(uk > SPEED_CONTROL_VAR_MAX)
	{
		uk = SPEED_CONTROL_VAR_MAX;
	}

	if(r_speed < 0.1 && r_speed > -0.1)
	{
		uk = 0.0;
	}

	uk_prev = uk;

	// Update fk FOXBORO parameter.
	*fk = beta * (*fk) + ( 1 - beta * uk);

	return uk*100; // Nem sz�zal�k
}

//! Function: cntrDistance
float cntrDistance (const uint32_t followDist,
					   const float prevSpeed,
					   const uint32_t actDist,
					   const float kP,
					   const uint32_t speedMin,
					   const uint32_t speedMax,
					   const bool extraSpeed)
{
	float diff;
	float speed;
	float distAct = actDist;

	// d diff = d wanted - d actual
	diff = ((float)distAct) - ((float)followDist);

	// Calculate control variable.
	speed = diff  * kP;

	if (actDist > 220)
	{
		speed = 0.8f;
	}
	if (extraSpeed == true)
	{
		speed = 1.0f;
	}

	// Saturation.
	if(speed < speedMin)
	{
		speed = speedMin;
	}
	if(speed > speedMax)
	{
		speed = speedMax;
	}

	return speed;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

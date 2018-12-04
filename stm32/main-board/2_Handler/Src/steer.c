////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      steer.c
//!  \brief
//!  \details	This module controls the steering wheel through the servo.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "servo.h"
#include "bsp_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define	 wheelTransmission			2

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------

static double steerCalculateServoAngle ( double steerAngle );
static double steerCalculateSteerAngle ( double servoAngle );

// Global function definitions -----------------------------------------------------------------------------------------

void steerInit()
{
	servoInit();
}

//TODO
void steerSetAngle(double steerAngle)
{
	double servoAngle = PI/2;
	int8_t correctedAnlge = steerAngle * wheelTransmission;

	servoAngle = steerCalculateServoAngle(correctedAnlge);

	// Saturation
	if (servoAngle > PI/2)
	{
		servoAngle = PI/2;
	}
	else if (servoAngle < -PI/2)
	{
		servoAngle = -PI/2;
	}

	servoSetAngle(servoAngle);
}

//! Steering wheel:
//!
//!                0°
//!         +30°   |   -30°
//!            \   |   /
//!             \  |  /
//!              \ | /
//!    +90________\|/________-90°
//!  Left end              Right end
//!
int8_t steerGetAngle()
{
	int8_t steerAngle = 0;
	int8_t correctedAngle = 0;
	double servoAngle = PI/2;

	servoAngle = servoGetAngle();
	steerAngle = steerCalculateSteerAngle(servoAngle/wheelTransmission);

	correctedAngle = steerAngle / wheelTransmission;

	return correctedAngle;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

												// DEG TODO
static double steerCalculateServoAngle ( double steerAngle )
{
	double srvAngle = PI/2;

	srvAngle = PI/180 * ( -1 * (double)steerAngle + PI/2 );
			// GRAD
	return srvAngle;
}
												// GRAD
static double steerCalculateSteerAngle ( double servoAngle )
{
	double steerAngle = 90;

	steerAngle = (double)(180/PI * (-1 * servoAngle + PI/2));

			// DEG TODO
	return steerAngle;
}

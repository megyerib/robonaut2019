////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      steer.c
//!  \brief
//!  \details	This module controls the steering wheel through the servo.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_common.h"
#include "steer.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define	 wheelTransmission				2
#define  STEERING_WHEEL_RIGHT_END		-PI/6		// -30° kormány,  60° szervo
#define  STEERING_WHEEL_LEFT_END		PI/6		// +30° kormány, 120° szervo

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------

static double steerCalculateServoAngle ( double steerAngle );
static double steerCalculateSteerAngle ( double servoAngle );

// Global function definitions -----------------------------------------------------------------------------------------

void steerInit(eServoModel myServo)
{
	servoInit(myServo);
}

void steerSetAngle(double steerAngle)
{
	double servoAngle = PI/2;
	double locSteerAngle = steerAngle;
	double correctedservoAnlge;

	// Saturation TODO nem jó  a szaturálás
	if (locSteerAngle > STEERING_WHEEL_LEFT_END)
	{
		locSteerAngle = STEERING_WHEEL_LEFT_END;
	}
	else if (locSteerAngle < STEERING_WHEEL_RIGHT_END)
	{
		locSteerAngle = STEERING_WHEEL_RIGHT_END;
	}

	correctedservoAnlge = locSteerAngle * wheelTransmission;
	servoAngle = steerCalculateServoAngle(correctedservoAnlge);

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
double steerGetAngle()
{
	double steerAngle = 0;
	double correctedAngle = 0;
	double servoAngle = PI/2;

	servoAngle = servoGetAngle();
	steerAngle = steerCalculateSteerAngle(servoAngle/wheelTransmission);

	correctedAngle = steerAngle / wheelTransmission;

	return correctedAngle;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//! @brief Converts rad to rad.
static double steerCalculateServoAngle ( double steerAngle )
{
	double srvAngle = PI/2;

	srvAngle = steerAngle + PI/2;

	return srvAngle;
}

static double steerCalculateSteerAngle ( double servoAngle )
{
	double steerAngle = 90;

	steerAngle = (double)(servoAngle + PI/2);

	return steerAngle;
}

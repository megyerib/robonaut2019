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

#define	 wheelTransmission				2
#define  STEERING_WHEEL_RIGHT_END		PI/3		// -30� korm�ny,  60� szervo
#define  STEERING_WHEEL_LEFT_END		2*PI/3		// +30� korm�ny, 120� szervo

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

void steerSetAngle(double steerAngle)
{
	double servoAngle = PI/2;
	double correctedAnlge = steerAngle * wheelTransmission;

	servoAngle = steerCalculateServoAngle(correctedAnlge);

	// Saturation TODO nem j�  a szatur�l�s
	if (servoAngle > STEERING_WHEEL_LEFT_END)
	{
		servoAngle = STEERING_WHEEL_LEFT_END;
	}
	else if (servoAngle < STEERING_WHEEL_RIGHT_END)
	{
		servoAngle = STEERING_WHEEL_RIGHT_END;
	}

	servoSetAngle(servoAngle);
}

//! Steering wheel:
//!
//!                0�
//!         +30�   |   -30�
//!            \   |   /
//!             \  |  /
//!              \ | /
//!    +90________\|/________-90�
//!  Left end              Right end
//!
int8_t steerGetAngle()
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

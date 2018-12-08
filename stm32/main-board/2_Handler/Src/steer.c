////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      steer.c
//!  \brief
//!  \details
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

static double steerCalculateServoAngle ( int8_t steerAngle );
static int8_t steerCalculateSteerAngle ( double srvAgnel );

// Global function definitions -----------------------------------------------------------------------------------------

void steerInit()
{
	servoInit();
}

void steerSetAngle(int8_t steerAngle)
{
	double servoAngle = PI/2;
	int8_t correctedAnlge = steerAngle * wheelTransmission;

	servoAngle = steerCalculateServoAngle(correctedAnlge);

	servoSetAngle(servoAngle);
}

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

												// DEG
static double steerCalculateServoAngle ( int8_t steerAngle )
{
	double srvAngle = PI/2;

	srvAngle = PI/180 * ( -1 * (double)steerAngle + PI/2 );
			// GRAD
	return srvAngle;
}
												// GRAD
static int8_t steerCalculateSteerAngle ( double srvAgnel )
{
	int8_t steerAngle = 90;

	steerAngle = (int8_t)(180/PI * (-1 * srvAgnel + PI/2));

			// DEG
	return steerAngle;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      trace.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "trace.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

extern QueueHandle_t qSharpDistance_u16;
extern QueueHandle_t qSharpCollWarn_x;
extern QueueHandle_t qServoAngle_d;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static bool traceWrapInteger (uint32_t* const value, const eBluetoothLogMember member, const uint32_t length);

static bool traceWrapBool (bool* const value, const eBluetoothLogMember member);

static bool traceWrapDouble (
								double* const 			  value,
								const uint32_t 			  decimals,
								const eBluetoothLogMember member,
								const uint32_t 			  length
							);

static uint32_t traceGetUpperBound (const uint32_t digits);

// Global function definitions -----------------------------------------------------------------------------------------

void traceInit (void)
{
	bcmInit();
}

void traceBluetooth(const eBluetoothLogMember destination, void* const data)
{
	switch (destination)
	{
		case BCM_LOG_SHARP_DISTANCE:
			//traceTraceSharpDistance((uint32_t* const)data);
			xQueueOverwrite(qSharpDistance_u16, (uint16_t* const)data);
			break;
		case BCM_LOG_SHARP_COLLISION_WARNING:
			//traceTraceSharpCollisionWarning((bool* const)data);
			xQueueOverwrite(qSharpCollWarn_x, (bool* const)data);
			break;
		case BCM_LOG_SERVO_ANGLE:
			//traceServoAngle((double* const)data);
			xQueueOverwrite(qServoAngle_d, (double* const)data);
			break;
		case BCM_LOG_INERT_ACCEL_X:

			break;
		default:
			break;
	}
}


void traceFlushData (void)
{
	uint32_t sharpDist;
	bool sharpColWarn;
	double srvAngle;

	xQueueReceive(qSharpDistance_u16, &sharpDist, 0);
	traceWrapInteger(&sharpDist, BCM_LOG_SHARP_DISTANCE, BCM_LOG_LENGHT_SHARP_DISTANCE);

	xQueueReceive(qSharpCollWarn_x, &sharpColWarn, 0);
	traceWrapBool(&sharpColWarn, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);

	xQueueReceive(qServoAngle_d, &srvAngle, 0);
	traceWrapDouble(&srvAngle, TRACE_DECIMALS_SERVO_ANGLE, BCM_LOG_SERVO_ANGLE, BCM_LOG_LENGHT_SERVO_ANGLE);

	bcmBtBufferFlush();
}


// Local (static) function definitions ---------------------------------------------------------------------------------

static bool traceWrapInteger (uint32_t* const value, const eBluetoothLogMember member, const uint32_t length)
{
	bool traced = false;
	uint8_t buffer[length];
	uint32_t upBound;
	uint32_t locValue = *value;

	memset(buffer, 0, sizeof(buffer));

	upBound = traceGetUpperBound(length);

	// Saturation
	if (locValue >= upBound)
	{
		locValue = upBound - 1;
	}

	hndlPlaceIntegerToAsciiMsg(buffer, locValue, length);

	traced = bcmLogMemberUpdate(member, buffer, length);

	return traced;
}

static bool traceWrapBool (bool* const value, const eBluetoothLogMember member)
{
	bool traced = false;
	uint8_t buffer;

	buffer = (uint8_t)*value + 0x30;

	traced = bcmLogMemberUpdate(member, &buffer, 1);

	return traced;
}

static bool traceWrapDouble (
								double* const 			  value,
								const uint32_t 			  decimals,
								const eBluetoothLogMember member,
								const uint32_t 			  length
							)
{
	bool traced = false;
	uint8_t  buffer[length];
	uint32_t mul;
	uint32_t upBound;
	uint32_t temp;
	double   locValue = *value;

	memset(buffer, 0, sizeof(buffer));

	mul = traceGetUpperBound(decimals);
	upBound = traceGetUpperBound(length);

	temp = (uint32_t)(mul*(*value));

	// Saturation
	if (temp >= upBound)
	{
		locValue = upBound - 1;
	}

	hndlPlaceFractionToAsciiMsg(buffer, locValue, length, decimals);

	traced = bcmLogMemberUpdate(member, buffer, length);

	return traced;
}

static uint32_t traceGetUpperBound (const uint32_t digits)
{
	uint32_t upBound = 1;
	uint32_t i = 0;

	for (i = 0; i < digits; i++)
	{
		upBound *= 10;
	}

	return upBound;
}

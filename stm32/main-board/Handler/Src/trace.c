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

static bool traceWrapSharpDistance (uint32_t* const dist);

static bool traceWrapSharpCollisionWarning (bool* const collision);

static bool traceWrapServoAngle (double* const theta, const uint32_t decimals);

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
	traceWrapSharpDistance(&sharpDist);

	xQueueReceive(qSharpCollWarn_x, &sharpColWarn, 0);
	traceWrapSharpCollisionWarning(&sharpColWarn);

	xQueueReceive(qServoAngle_d, &srvAngle, 0);
	traceWrapServoAngle(&srvAngle, TRACE_DECIMALS_SERVO_ANGLE);

	bcmBtBufferFlush();
}


// Local (static) function definitions ---------------------------------------------------------------------------------

static bool traceWrapSharpDistance (uint32_t* const dist)
{
	bool traced = false;
	uint8_t buffer[BCM_LOG_LENGHT_SHARP_DISTANCE];

	memset(buffer, 0, sizeof(buffer));

	hndlPlaceIntegerToAsciiMsg(buffer, *dist, BCM_LOG_LENGHT_SHARP_DISTANCE);

	traced = bcmLogMemberUpdate(BCM_LOG_SHARP_DISTANCE, buffer, BCM_LOG_LENGHT_SHARP_DISTANCE);

	return traced;
}

static bool traceWrapSharpCollisionWarning (bool* const collision)
{
	bool traced = false;
	uint8_t buffer;

	buffer = (uint8_t)*collision + 0x30;

	traced = bcmLogMemberUpdate(BCM_LOG_SHARP_COLLISION_WARNING, &buffer, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);

	return traced;
}

static bool traceWrapServoAngle (double* const theta, const uint32_t decimals)
{
	bool traced = false;
	uint8_t buffer[BCM_LOG_LENGHT_SERVO_ANGLE];

	memset(buffer, 0, sizeof(buffer));

	hndlPlaceFractionToAsciiMsg(buffer, *theta, BCM_LOG_LENGHT_SERVO_ANGLE, decimals);

	traced = bcmLogMemberUpdate(BCM_LOG_SERVO_ANGLE, buffer, BCM_LOG_LENGHT_SERVO_ANGLE);

	return traced;
}

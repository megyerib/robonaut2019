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

//! Handles of the available message queues
extern QueueHandle_t qSharpDistance_u32;	// 1
extern QueueHandle_t qSharpCollWarn_x;
extern QueueHandle_t qServoAngle_d;
extern QueueHandle_t qInertAccelX_d;
extern QueueHandle_t qInertAccelY_d;
extern QueueHandle_t qInertAccelZ_d;
extern QueueHandle_t qInertAngVelX_d;
extern QueueHandle_t qInertAngVelY_d;
extern QueueHandle_t qInertAngVelZ_d;
extern QueueHandle_t qNaviN_d;				// 10
extern QueueHandle_t qNaviE_d;
extern QueueHandle_t qNaviTheta_d;
extern QueueHandle_t qEncVel_d;
extern QueueHandle_t qTof1Distance_u32;
extern QueueHandle_t qTof2Distance_u32;
extern QueueHandle_t qTof3Distance_u32;
extern QueueHandle_t qMtrMainBatVolt_d;
extern QueueHandle_t qMtrSecBatVolt_d;
extern QueueHandle_t qMtrCurr_d;
extern QueueHandle_t qMtrSysCurr_u32;		// 20
extern QueueHandle_t qMtrSrvCurr_u32;
extern QueueHandle_t qMtrCmdStopEngine_x;
extern QueueHandle_t qCtrlMtrCurr_d;

// Local (static) function prototypes ----------------------------------------------------------------------------------

//! Converts a integer value to a unit8_t array so it can be sent out on bluetooth.
//!
//! @param value		value that will be sent out
//! @param member		determines which how sent and what type of data
//! @param length		how many character has to be sent out
//! @return				was the conversion successful or not
static bool traceWrapInteger (uint32_t* const value, const eBluetoothLogMember member, const uint32_t length);

//! Converts a bool value to a unit8_t array so it can be sent out on bluetooth.
//!
//! @param value		value that will be sent out
//! @param member		determines which how sent and what type of data
//! @return				was the conversion successful or not
static bool traceWrapBool (bool* const value, const eBluetoothLogMember member);

//!	Converts a bool value to a unit8_t array so it can be sent out on bluetooth.
//!
//! @param value		value that will be sent out
//! @param decimals		how many decimals of the double has to be sent out
//! @param member		determines which how sent and what type of data
//! @param length		how many character has to be sent out
//! @return				was the conversion successful or not
static bool traceWrapDouble (
								double* const 			  value,
								const uint32_t 			  decimals,
								const eBluetoothLogMember member,
								const uint32_t 			  length
							);

//! Determines a power of 10 that can be used as a bound. With this the vlaue can be satured so it can be sent out in a
//! given lenght and decimals value.
//!
//! @param digits		how many digit the value consist of
//! @return				the calculated bound
static uint32_t traceGetBoundCharNum (const uint32_t digits);

// Global function definitions -----------------------------------------------------------------------------------------

void traceInit (void)
{
	bcmInit();
}

void traceBluetooth(const eBluetoothLogMember destination, void* const data)
{
	// Select who sent the log request.
	switch (destination)
	{
		case BCM_LOG_SHARP_DISTANCE:		// 1
			xQueueOverwrite(qSharpDistance_u32, (uint32_t* const)data);
			break;
		case BCM_LOG_SHARP_COLLISION_WARNING:
			xQueueOverwrite(qSharpCollWarn_x, (bool* const)data);
			break;
		case BCM_LOG_SERVO_ANGLE:
			xQueueOverwrite(qServoAngle_d, (double* const)data);
			break;
		case BCM_LOG_INERT_ACCEL_X:
			xQueueOverwrite(qInertAccelX_d, (double* const)data);
			break;
		case BCM_LOG_INERT_ACCEL_Y:
			xQueueOverwrite(qInertAccelY_d, (double* const)data);
			break;
		case BCM_LOG_INERT_ACCEL_Z:
			xQueueOverwrite(qInertAccelZ_d, (double* const)data);
			break;
		case BCM_LOG_INERT_ANG_VEL_X:
			xQueueOverwrite(qInertAngVelX_d, (double* const)data);
			break;
		case BCM_LOG_INERT_ANG_VEL_Y:
			xQueueOverwrite(qInertAngVelY_d, (double* const)data);
			break;
		case BCM_LOG_INERT_ANG_VEL_Z:
			xQueueOverwrite(qInertAngVelZ_d, (double* const)data);
			break;
		case BCM_LOG_NAVI_N:				// 10
			xQueueOverwrite(qNaviN_d, (double* const)data);
			break;
		case BCM_LOG_NAVI_E:
			xQueueOverwrite(qNaviE_d, (double* const)data);
			break;
		case BCM_LOG_NAVI_THETA:
			xQueueOverwrite(qNaviTheta_d, (double* const)data);
			break;
		case BCM_LOG_ENC_VEL:
			xQueueOverwrite(qEncVel_d, (double* const)data);
			break;
		case BCM_LOG_TOF_1_DISTANCE:
			xQueueOverwrite(qTof1Distance_u32, (uint32_t* const)data);
			break;
		case BCM_LOG_TOF_2_DISTANCE:
			xQueueOverwrite(qTof2Distance_u32, (uint32_t* const)data);
			break;
		case BCM_LOG_TOF_3_DISTANCE:
			xQueueOverwrite(qTof3Distance_u32, (uint32_t* const)data);
			break;
		case BCM_LOG_MTR_MAIN_BAT_VOLT:
			xQueueOverwrite(qMtrMainBatVolt_d, (double* const)data);
			break;
		case BCM_LOG_MTR_SEC_BAT_VOLT:
			xQueueOverwrite(qMtrSecBatVolt_d, (double* const)data);
			break;
		case BCM_LOG_MTR_CURR:
			xQueueOverwrite(qMtrCurr_d, (double* const)data);
			break;
		case BCM_LOG_MTR_SYS_CURR:			// 20
			xQueueOverwrite(qMtrSysCurr_u32, (uint32_t* const)data);
			break;
		case BCM_LOG_MTR_SRV_CURR:
			xQueueOverwrite(qMtrSrvCurr_u32, (uint32_t* const)data);
			break;
		case BCM_LOG_MTR_CMD_STOP_ENGINE:
			xQueueOverwrite(qMtrCmdStopEngine_x, (bool* const)data);
			break;
		case BCM_LOG_CTR_MTR_CURR:
			xQueueOverwrite(qCtrlMtrCurr_d, (double* const)data);
			break;
		default:
			break;
	}
}

void traceFlushData (void)
{
	uint32_t sharpDist        = 0;
	bool     sharpColWarn     = 0;
	double   srvAngle         = 0;
	double   inertAccelX      = 0;
	double   inertAccelY      = 0;
	double   inertAccelZ      = 0;
	double   inertAngVelX     = 0;
	double   inertAngVelY     = 0;
	double   inertAngVelZ     = 0;
	double   naviN            = 0;
	double   naviE            = 0;
	double   naviTheta        = 0;
	double   encVel           = 0;
	uint32_t tof1Distance     = 0;
	uint32_t tof2Distance     = 0;
	uint32_t tof3Distance     = 0;
	double   mtrMainBatVolt   = 0;
	double   mtrSecBatVolt    = 0;
	double   mtrCurr          = 0;
	uint32_t mtrSysCurr       = 0;
	uint32_t mtrSrvCurr       = 0;
	bool     mtrCmdStopEngine = 0;
	double   ctrlMtrCurr      = 0;

	// Collect and save the values from the queue into the bluetooth log structure.
	xQueueReceive(qSharpDistance_u32, &sharpDist, 0);
	traceWrapInteger(&sharpDist, BCM_LOG_SHARP_DISTANCE, BCM_LOG_LENGHT_SHARP_DISTANCE);

	xQueueReceive(qSharpCollWarn_x, &sharpColWarn, 0);
	traceWrapBool(&sharpColWarn, BCM_LOG_SHARP_COLLISION_WARNING);

	xQueueReceive(qServoAngle_d, &srvAngle, 0);
	traceWrapDouble(&srvAngle, TRACE_DECIMALS_SERVO_ANGLE, BCM_LOG_SERVO_ANGLE, BCM_LOG_LENGHT_SERVO_ANGLE);

	xQueueReceive(qInertAccelX_d, &inertAccelX, 0);
	traceWrapDouble(&inertAccelX, TRACE_DECIMALS_INERT_ACCEL_X, BCM_LOG_INERT_ACCEL_X, BCM_LOG_LENGHT_INERT_ACCEL_X);

	xQueueReceive(qInertAccelY_d, &inertAccelY, 0);
	traceWrapDouble(&inertAccelY, TRACE_DECIMALS_INERT_ACCEL_Y, BCM_LOG_INERT_ACCEL_Y, BCM_LOG_LENGHT_INERT_ACCEL_Y);

	xQueueReceive(qInertAccelZ_d, &inertAccelZ, 0);
	traceWrapDouble(&inertAccelZ, TRACE_DECIMALS_INERT_ACCEL_Z, BCM_LOG_INERT_ACCEL_Z, BCM_LOG_LENGHT_INERT_ACCEL_Z);

	xQueueReceive(qInertAngVelX_d, &inertAngVelX, 0);
	traceWrapDouble(&inertAngVelX, TRACE_DECIMALS_INERT_ANG_VEL_X, BCM_LOG_INERT_ANG_VEL_X, BCM_LOG_LENGHT_INERT_ANG_VEL_X);

	xQueueReceive(qInertAngVelY_d, &inertAngVelY, 0);
	traceWrapDouble(&inertAngVelY, TRACE_DECIMALS_INERT_ANG_VEL_Y, BCM_LOG_INERT_ANG_VEL_Y, BCM_LOG_LENGHT_INERT_ANG_VEL_Y);

	xQueueReceive(qInertAngVelZ_d, &inertAngVelZ, 0);
	traceWrapDouble(&inertAngVelZ, TRACE_DECIMALS_INERT_ANG_VEL_Z, BCM_LOG_INERT_ANG_VEL_Z, BCM_LOG_LENGHT_INERT_ANG_VEL_Z);

	xQueueReceive(qNaviN_d, &naviN, 0);
	traceWrapDouble(&naviN, TRACE_DECIMALS_NAVI_N, BCM_LOG_NAVI_N, BCM_LOG_LENGHT_NAVI_N);

	xQueueReceive(qNaviE_d, &naviE, 0);
	traceWrapDouble(&naviE, TRACE_DECIMALS_NAVI_E, BCM_LOG_NAVI_E, BCM_LOG_LENGHT_NAVI_E);

	xQueueReceive(qNaviTheta_d, &naviTheta, 0);
	traceWrapDouble(&naviTheta, TRACE_DECIMALS_NAVI_THETA, BCM_LOG_NAVI_THETA, BCM_LOG_LENGHT_NAVI_THETA);

	xQueueReceive(qEncVel_d, &encVel, 0);
	traceWrapDouble(&encVel, TRACE_DECIMALS_ENC_VEL, BCM_LOG_ENC_VEL, BCM_LOG_LENGHT_ENC_VEL);

	xQueueReceive(qTof1Distance_u32, &tof1Distance, 0);
	traceWrapInteger(&tof1Distance, BCM_LOG_TOF_1_DISTANCE, BCM_LOG_LENGHT_TOF_1_DISTANCE);

	xQueueReceive(qTof2Distance_u32, &tof2Distance, 0);
	traceWrapInteger(&tof2Distance, BCM_LOG_TOF_2_DISTANCE, BCM_LOG_LENGHT_TOF_2_DISTANCE);

	xQueueReceive(qTof3Distance_u32, &tof3Distance, 0);
	traceWrapInteger(&tof3Distance, BCM_LOG_TOF_3_DISTANCE, BCM_LOG_LENGHT_TOF_3_DISTANCE);

	xQueueReceive(qMtrMainBatVolt_d, &mtrMainBatVolt, 0);
	traceWrapDouble(&mtrMainBatVolt, TRACE_DECIMALS_MTR_MAIN_BAT_VOLT, BCM_LOG_MTR_MAIN_BAT_VOLT, BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT);

	xQueueReceive(qMtrSecBatVolt_d, &mtrSecBatVolt, 0);
	traceWrapDouble(&mtrSecBatVolt, TRACE_DECIMALS_MTR_SEC_BAT_VOLT, BCM_LOG_MTR_SEC_BAT_VOLT, BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT);

	xQueueReceive(qMtrCurr_d, &mtrCurr, 0);
	traceWrapDouble(&mtrCurr, TRACE_DECIMALS_MTR_CURR, BCM_LOG_MTR_CURR, BCM_LOG_LENGHT_MTR_CURR);

	xQueueReceive(qMtrSysCurr_u32, &mtrSysCurr, 0);
	traceWrapInteger(&mtrSysCurr, BCM_LOG_MTR_SYS_CURR, BCM_LOG_LENGHT_MTR_SYS_CURR);

	xQueueReceive(qMtrSrvCurr_u32, &mtrSrvCurr, 0);
	traceWrapInteger(&mtrSrvCurr, BCM_LOG_MTR_SRV_CURR, BCM_LOG_LENGHT_MTR_SRV_CURR);

	xQueueReceive(qMtrCmdStopEngine_x, &mtrCmdStopEngine, 0);
	traceWrapBool(&mtrCmdStopEngine, BCM_LOG_MTR_CMD_STOP_ENGINE);

	xQueueReceive(qCtrlMtrCurr_d, &ctrlMtrCurr, 0);
	traceWrapDouble(&ctrlMtrCurr, TRACE_DECIMALS_CTRL_MTR_CURR, BCM_LOG_CTR_MTR_CURR, BCM_LOG_LENGHT_CTRL_MTR_CURR);

	// Send out the bluetooth log.
	bcmBtBufferFlush();
}


// Local (static) function definitions ---------------------------------------------------------------------------------

static bool traceWrapInteger (uint32_t* const value, const eBluetoothLogMember member, const uint32_t length)
{
	bool traced = false;
	uint8_t buffer[length];
	uint32_t bound;
	uint32_t locValue = *value;
	bool isNegative = false;

	// Empty the buffer.
	memset(buffer, 0, sizeof(buffer));

	// Check the sign.
	if(locValue < 0)
	{
		// negative
		isNegative = true;

		// The negative sign is the first char
		bound = traceGetBoundCharNum(length-1);

		// Check the bound.
		if (locValue <= bound)
		{
			// Under the bound, saturate over the bound.
			locValue = bound + 1;
		}
	}
	else
	{
		// positive
		bound = traceGetBoundCharNum(length);

		// Check the bound.
		if (locValue >= bound)
		{
			// Over the bound, saturate under the bound.
			locValue = bound - 1;
		}
	}

	// Save the value into a buffer.
	hndlPlaceIntegerToAsciiMsg(buffer, locValue, length, isNegative);

	// Save the buffer into the bluetooth log structure.
	traced = bcmLogMemberUpdate(member, buffer, length);

	return traced;
}

static bool traceWrapBool (bool* const value, const eBluetoothLogMember member)
{
	bool traced = false;
	uint8_t buffer;

	// Convert the bool value to ASCII character.
	buffer = (uint8_t)(*value) + 0x30;

	// Save the value to the bluetooth log structure.
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
	uint32_t bound;
	double   locValue = *value;

	// At least one integer digit is need, the value can't be only decimals.
	if(length > decimals)
	{
		// Empty the buffer;
		memset(buffer, 0, sizeof(buffer));

		// Calculate how many shifts will be needed according to the decimals.
		mul = traceGetBoundCharNum(decimals);

		// Shift the value to save the observe the decimals too.
		locValue *= mul;

		// Determine the value that will be placed into the trace buffer.
		// Check the sign.
		if(value < 0)
		{
			// Negative sign.

			// Determine lower bound for the lowest available value.
			bound = traceGetBoundCharNum(length-1);

			// Check the bound.
			if(locValue <= bound)
			{
				// Under the bound, saturate over the bound.
				locValue = bound + 1;
			}
		}
		else
		{
			// Positive sign.

			// Determine upper bound for the highest available value.
			bound = traceGetBoundCharNum(length);

			// Check the bound.
			if(locValue >= bound)
			{
				// Over the bound, saturate under the bound.
				locValue = bound - 1;
			}
		}

		// Shift back the value.
		locValue /= mul;

		// Plcace the value into a byte buffer.
		hndlPlaceFractionToAsciiMsg(buffer, locValue, length, decimals);

		// Save the buffer into the trace buffer.
		traced = bcmLogMemberUpdate(member, buffer, length);
	}

	return traced;
}

static uint32_t traceGetBoundCharNum (const uint32_t digits)
{
	uint32_t upBound = 1;
	uint32_t i = 0;

	for (i = 0; i < digits; i++)
	{
		upBound *= 10;
	}

	return upBound;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      trace.c
//!  \brief		Send diagnostics data.
//!  \details	This module collects data and send them through a communication port.
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
extern QueueHandle_t qServoAngle_f;
extern QueueHandle_t qInertAccelX_f;
extern QueueHandle_t qInertAccelY_f;
extern QueueHandle_t qInertAccelZ_f;
extern QueueHandle_t qInertAngVelX_f;
extern QueueHandle_t qInertAngVelY_f;
extern QueueHandle_t qInertAngVelZ_f;
extern QueueHandle_t qNaviN_f;				// 10
extern QueueHandle_t qNaviE_f;
extern QueueHandle_t qNaviPhi_f;
extern QueueHandle_t qEncVel_f;
extern QueueHandle_t qTof1Distance_u32;
extern QueueHandle_t qTof2Distance_u32;
extern QueueHandle_t qTof3Distance_u32;
extern QueueHandle_t qMtrMainBatVolt_f;
extern QueueHandle_t qMtrSecBatVolt_f;
extern QueueHandle_t qMtrCurr_f;
extern QueueHandle_t qMtrSysCurr_u32;		// 20
extern QueueHandle_t qMtrSrvCurr_u32;
extern QueueHandle_t qMtrCmdStopEngine_x;
extern QueueHandle_t qCtrlMtrCurr_f;
extern QueueHandle_t qLineD_u32;
extern QueueHandle_t qLineTheta_u32;

extern QueueHandle_t qRecData;
static cTraceRxBluetoothStruct btRxData;

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
static bool traceWrapFloat (
								float* const 			  value,
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

static uint32_t traceUnwrapInteger (uint8_t* const buffer, uint32_t begin, uint32_t size);

static bool traceUnwrapBool (uint8_t* const buffer, uint32_t begin);

static float traceUnwrapFloat (uint8_t* const buffer, uint32_t begin, uint32_t size, uint32_t decimals);

// Global function definitions -----------------------------------------------------------------------------------------

void traceInit (void)
{
	bcmInit();

	/* TODO DEBUG
	uint8_t buff[12] = "0010-1053809";

	uint32_t i = traceUnwrapInteger(buff, 1, 3);
	bool x = traceUnwrapBool(buff, 0);
	double d = traceUnwrapDouble(buff, 4, 8, 4);*/
}

void traceBluetooth (const eBluetoothLogMember destination, void* const data)
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
			xQueueOverwrite(qServoAngle_f, (float* const)data);
			break;
		case BCM_LOG_INERT_ACCEL_X:
			xQueueOverwrite(qInertAccelX_f, (float* const)data);
			break;
		case BCM_LOG_INERT_ACCEL_Y:
			xQueueOverwrite(qInertAccelY_f, (float* const)data);
			break;
		case BCM_LOG_INERT_ACCEL_Z:
			xQueueOverwrite(qInertAccelZ_f, (float* const)data);
			break;
		case BCM_LOG_INERT_ANG_VEL_X:
			xQueueOverwrite(qInertAngVelX_f, (float* const)data);
			break;
		case BCM_LOG_INERT_ANG_VEL_Y:
			xQueueOverwrite(qInertAngVelY_f, (float* const)data);
			break;
		case BCM_LOG_INERT_ANG_VEL_Z:
			xQueueOverwrite(qInertAngVelZ_f, (float* const)data);
			break;
		case BCM_LOG_NAVI_N:				// 10
			xQueueOverwrite(qNaviN_f, (float* const)data);
			break;
		case BCM_LOG_NAVI_E:
			xQueueOverwrite(qNaviE_f, (float* const)data);
			break;
		case BCM_LOG_NAVI_THETA:
			xQueueOverwrite(qNaviPhi_f, (float* const)data);
			break;
		case BCM_LOG_ENC_VEL:
			xQueueOverwrite(qEncVel_f, (float* const)data);
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
			xQueueOverwrite(qMtrMainBatVolt_f, (float* const)data);
			break;
		case BCM_LOG_MTR_SEC_BAT_VOLT:
			xQueueOverwrite(qMtrSecBatVolt_f, (float* const)data);
			break;
		case BCM_LOG_MTR_CURR:
			xQueueOverwrite(qMtrCurr_f, (float* const)data);
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
			xQueueOverwrite(qCtrlMtrCurr_f, (float* const)data);
			break;
		case BCM_LOG_LINE_D:
			xQueueOverwrite(qLineD_u32, (uint32_t* const)data);
			break;
		case BCM_LOG_LINE_THETA:
			xQueueOverwrite(qLineTheta_u32, (uint32_t* const)data);
			break;
		default:
			break;
	}
}

void traceFlushData (void)
{
	uint32_t sharpDist        = 0;
	bool     sharpColWarn     = 0;
	float   srvAngle          = 0;
	float   inertAccelX       = 0;
	float   inertAccelY       = 0;
	float   inertAccelZ       = 0;
	float   inertAngVelX      = 0;
	float   inertAngVelY      = 0;
	float   inertAngVelZ      = 0;
	float   naviN             = 0;
	float   naviE             = 0;
	float   naviTheta         = 0;
	float   encVel            = 0;
	uint32_t tof1Distance     = 0;
	uint32_t tof2Distance     = 0;
	uint32_t tof3Distance     = 0;
	float   mtrMainBatVolt    = 0;
	float   mtrSecBatVolt     = 0;
	float   mtrCurr           = 0;
	uint32_t mtrSysCurr       = 0;
	uint32_t mtrSrvCurr       = 0;
	bool     mtrCmdStopEngine = 0;
	float   ctrlMtrCurr       = 0;
	uint32_t lineD			  = 0;
	uint32_t lineTheta		  = 0;

	// Collect and save the values from the queue into the bluetooth log structure.
	xQueueReceive(qSharpDistance_u32, &sharpDist, 0);
	traceWrapInteger(&sharpDist, BCM_LOG_SHARP_DISTANCE, BCM_LOG_LENGHT_SHARP_DISTANCE);

	xQueueReceive(qSharpCollWarn_x, &sharpColWarn, 0);
	traceWrapBool(&sharpColWarn, BCM_LOG_SHARP_COLLISION_WARNING);

	xQueueReceive(qServoAngle_f, &srvAngle, 0);
	traceWrapFloat(&srvAngle, TRACE_DECIMALS_SERVO_ANGLE, BCM_LOG_SERVO_ANGLE, BCM_LOG_LENGHT_SERVO_ANGLE);

	xQueueReceive(qInertAccelX_f, &inertAccelX, 0);
	traceWrapFloat(&inertAccelX, TRACE_DECIMALS_INERT_ACCEL_X, BCM_LOG_INERT_ACCEL_X, BCM_LOG_LENGHT_INERT_ACCEL_X);

	xQueueReceive(qInertAccelY_f, &inertAccelY, 0);
	traceWrapFloat(&inertAccelY, TRACE_DECIMALS_INERT_ACCEL_Y, BCM_LOG_INERT_ACCEL_Y, BCM_LOG_LENGHT_INERT_ACCEL_Y);

	xQueueReceive(qInertAccelZ_f, &inertAccelZ, 0);
	traceWrapFloat(&inertAccelZ, TRACE_DECIMALS_INERT_ACCEL_Z, BCM_LOG_INERT_ACCEL_Z, BCM_LOG_LENGHT_INERT_ACCEL_Z);

	xQueueReceive(qInertAngVelX_f, &inertAngVelX, 0);
	traceWrapFloat(&inertAngVelX, TRACE_DECIMALS_INERT_ANG_VEL_X, BCM_LOG_INERT_ANG_VEL_X, BCM_LOG_LENGHT_INERT_ANG_VEL_X);

	xQueueReceive(qInertAngVelY_f, &inertAngVelY, 0);
	traceWrapFloat(&inertAngVelY, TRACE_DECIMALS_INERT_ANG_VEL_Y, BCM_LOG_INERT_ANG_VEL_Y, BCM_LOG_LENGHT_INERT_ANG_VEL_Y);

	xQueueReceive(qInertAngVelZ_f, &inertAngVelZ, 0);
	traceWrapFloat(&inertAngVelZ, TRACE_DECIMALS_INERT_ANG_VEL_Z, BCM_LOG_INERT_ANG_VEL_Z, BCM_LOG_LENGHT_INERT_ANG_VEL_Z);

	xQueueReceive(qNaviN_f, &naviN, 0);
	traceWrapFloat(&naviN, TRACE_DECIMALS_NAVI_N, BCM_LOG_NAVI_N, BCM_LOG_LENGHT_NAVI_N);

	xQueueReceive(qNaviE_f, &naviE, 0);
	traceWrapFloat(&naviE, TRACE_DECIMALS_NAVI_E, BCM_LOG_NAVI_E, BCM_LOG_LENGHT_NAVI_E);

	xQueueReceive(qNaviPhi_f, &naviTheta, 0);
	traceWrapFloat(&naviTheta, TRACE_DECIMALS_NAVI_THETA, BCM_LOG_NAVI_THETA, BCM_LOG_LENGHT_NAVI_THETA);

	xQueueReceive(qEncVel_f, &encVel, 0);
	traceWrapFloat(&encVel, TRACE_DECIMALS_ENC_VEL, BCM_LOG_ENC_VEL, BCM_LOG_LENGHT_ENC_VEL);

	xQueueReceive(qTof1Distance_u32, &tof1Distance, 0);
	traceWrapInteger(&tof1Distance, BCM_LOG_TOF_1_DISTANCE, BCM_LOG_LENGHT_TOF_1_DISTANCE);

	xQueueReceive(qTof2Distance_u32, &tof2Distance, 0);
	traceWrapInteger(&tof2Distance, BCM_LOG_TOF_2_DISTANCE, BCM_LOG_LENGHT_TOF_2_DISTANCE);

	xQueueReceive(qTof3Distance_u32, &tof3Distance, 0);
	traceWrapInteger(&tof3Distance, BCM_LOG_TOF_3_DISTANCE, BCM_LOG_LENGHT_TOF_3_DISTANCE);

	xQueueReceive(qMtrMainBatVolt_f, &mtrMainBatVolt, 0);
	traceWrapFloat(&mtrMainBatVolt, TRACE_DECIMALS_MTR_MAIN_BAT_VOLT, BCM_LOG_MTR_MAIN_BAT_VOLT, BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT);

	xQueueReceive(qMtrSecBatVolt_f, &mtrSecBatVolt, 0);
	traceWrapFloat(&mtrSecBatVolt, TRACE_DECIMALS_MTR_SEC_BAT_VOLT, BCM_LOG_MTR_SEC_BAT_VOLT, BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT);

	xQueueReceive(qMtrCurr_f, &mtrCurr, 0);
	traceWrapFloat(&mtrCurr, TRACE_DECIMALS_MTR_CURR, BCM_LOG_MTR_CURR, BCM_LOG_LENGHT_MTR_CURR);

	xQueueReceive(qMtrSysCurr_u32, &mtrSysCurr, 0);
	traceWrapInteger(&mtrSysCurr, BCM_LOG_MTR_SYS_CURR, BCM_LOG_LENGHT_MTR_SYS_CURR);

	xQueueReceive(qMtrSrvCurr_u32, &mtrSrvCurr, 0);
	traceWrapInteger(&mtrSrvCurr, BCM_LOG_MTR_SRV_CURR, BCM_LOG_LENGHT_MTR_SRV_CURR);

	xQueueReceive(qMtrCmdStopEngine_x, &mtrCmdStopEngine, 0);
	traceWrapBool(&mtrCmdStopEngine, BCM_LOG_MTR_CMD_STOP_ENGINE);

	xQueueReceive(qCtrlMtrCurr_f, &ctrlMtrCurr, 0);
	traceWrapFloat(&ctrlMtrCurr, TRACE_DECIMALS_CTRL_MTR_CURR, BCM_LOG_CTR_MTR_CURR, BCM_LOG_LENGHT_CTRL_MTR_CURR);

	xQueueReceive(qLineD_u32, &lineD, 0);
	traceWrapInteger(&lineD, BCM_LOG_LINE_D, BCM_LOG_LENGHT_LINE_D);

	xQueueReceive(qLineTheta_u32, &lineTheta, 0);
	traceWrapInteger(&lineTheta, BCM_LOG_LINE_THETA, BCM_LOG_LENGHT_LINE_THETA);

	// Send out the bluetooth log.
	bcmBtBufferFlush();
}

cTraceRxBluetoothStruct traceProcessRxData (uint8_t* const buffer)
{
	cTraceRxBluetoothStruct dataSruct;
	uint8_t rxDataSize = 0;
	uint8_t index = 0;

	rxDataSize = traceUnwrapInteger(buffer, 2, 2);
	index = 4;

	if (rxDataSize == TRACE_REC_MSG_SIZE)
	{
		dataSruct.RecCmdStop = traceUnwrapBool(buffer, index);
		index++;

		dataSruct.RecCmdFollowLine = traceUnwrapBool(buffer, index);
		index++;

		dataSruct.RecCmdSelfTest = traceUnwrapBool(buffer, index);
		index++;

		dataSruct.RecCmdAccelerate = traceUnwrapBool(buffer, index);
		index++;

		dataSruct.RecDataAccelerate = traceUnwrapInteger(buffer, index, TRACE_REC_ACCEL_SIZE);
		index += TRACE_REC_ACCEL_SIZE;

		dataSruct.RecCmdSteer = traceUnwrapBool(buffer, index);
		index++;

		dataSruct.RecDataSteer = traceUnwrapInteger(buffer, index, TRACE_REC_STEER_SIZE);
		index += TRACE_REC_STEER_SIZE;

		dataSruct.RecCmdPdTd = traceUnwrapBool(buffer, index);
		index++;

		dataSruct.RecDataPdTd_d = traceUnwrapFloat(buffer, index, TRACE_REC_PD_TD_SIZE, TRACE_REC_PD_TD_DECIMALS);
		index += TRACE_REC_PD_TD_SIZE;

		dataSruct.RecCmdPdKp_x = traceUnwrapBool(buffer, index);
		index++;

		dataSruct.RecDataPdKp_d = traceUnwrapFloat(buffer, index, TRACE_REC_PD_KP_SIZE, TRACE_REC_PD_KP_DECIMALS);
		index += TRACE_REC_PD_KP_SIZE;
	}

	return dataSruct;
}

cTraceRxBluetoothStruct traceReceiveBluetooth (void)
{
	xQueueReceive(qRecData, &btRxData, 0);

	return btRxData;
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

static bool traceWrapFloat (
								float* const 			  value,
								const uint32_t 			  decimals,
								const eBluetoothLogMember member,
								const uint32_t 			  length
							)
{
	bool traced = false;
	uint8_t  buffer[length];
	uint32_t mul;
	uint32_t bound;
	float    locValue = *value;

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

static uint32_t traceUnwrapInteger (uint8_t* const buffer, uint32_t begin, uint32_t size)
{
	uint32_t retVal = 0;
	uint8_t i = 0;

	for (i = 0; i < size; i++)
	{
		retVal *= 10;
		retVal += buffer[begin+i] - 0x30;
	}

	return retVal;
}

static bool traceUnwrapBool (uint8_t* const buffer, uint32_t begin)
{
	bool retVal = false;
	uint8_t value;

	value = buffer[begin] - 0x30;
	if (value == 0)
	{
		retVal = false;
	}
	else
	{
		retVal = true;
	}

	return retVal;
}

static float traceUnwrapFloat (uint8_t* const buffer, uint32_t begin, uint32_t size, uint32_t decimals)
{
	uint32_t digits;
	uint32_t decims;
	float    retVal;
	uint8_t i = 0;
	bool isNegative = false;

	if (buffer[begin] == '-')
	{
		isNegative = true;
		digits = traceUnwrapInteger(buffer, begin+1, size-decimals-1);
	}
	else
	{
		digits = traceUnwrapInteger(buffer, begin, size-decimals);
	}

	decims = traceUnwrapInteger(buffer, begin+size-decimals, decimals);

	retVal = (float)decims;

	for (i = 0; i < decimals; i++)
	{
		retVal /= 10;
	}

	retVal += digits;

	if (isNegative == true)
	{
		retVal *= -1;
	}

	return retVal;
}

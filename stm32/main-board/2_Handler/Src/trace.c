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

#define		TRACE_MAX_NUMBER_OF_DIGITS		15

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Handles of the available message queues
//_______________________________________________ TX SIDE
extern QueueHandle_t qNaviN_f;
extern QueueHandle_t qNaviE_f;
extern QueueHandle_t qNaviPSI_f;
extern QueueHandle_t qEncV_f;
extern QueueHandle_t qDistToF1_u32;
extern QueueHandle_t qDistToF2_u32;
extern QueueHandle_t qDistToF3_u32;
extern QueueHandle_t qDistSharp1_u32;
extern QueueHandle_t qInertAccelX_f;
extern QueueHandle_t qInertAccelY_f;
extern QueueHandle_t qInertAccelZ_f;
extern QueueHandle_t qInertAngVelX_f;
extern QueueHandle_t qInertAngVelY_f;
extern QueueHandle_t qInertAngVelZ_f;
extern QueueHandle_t qSteerWheelAngle_f;
extern QueueHandle_t qServoAngle_f;

extern QueueHandle_t qMtrMainBatVolt_f;
extern QueueHandle_t qMtrSecBatVolt_f;
extern QueueHandle_t qMtrCurr_f;
extern QueueHandle_t qMtrSysCurr_u32;
extern QueueHandle_t qMtrServoCurr_u32;

extern QueueHandle_t qLineLineNbr_u32;
extern QueueHandle_t qLineMainLinePos_f;
extern QueueHandle_t qLineSecLinePos_f;

extern QueueHandle_t qMazeMainSM_u32;
extern QueueHandle_t qMazeGetKp_f;
extern QueueHandle_t qMazeGetKd_f;
extern QueueHandle_t qMazeGetSpeed_u32;
extern QueueHandle_t qMazeSegments_u32;
extern QueueHandle_t qMazeActState_u32;
extern QueueHandle_t qMazeActKp_f;
extern QueueHandle_t qMazeActKd_f;
extern QueueHandle_t qMazeActSpeed_u32;
extern QueueHandle_t qMazeInclinSegment_u32;

extern QueueHandle_t qSRunMainSM_u32;
extern QueueHandle_t qSRunActState_u32;
extern QueueHandle_t qSRunActP_f;
extern QueueHandle_t qSRunActKp_f;
extern QueueHandle_t qSRunActKd_f;
extern QueueHandle_t qSRunActSpeed_u32;
extern QueueHandle_t qSRunGetP_f;
extern QueueHandle_t qSRunGetKp_f;
extern QueueHandle_t qSRunGetKd_f;
extern QueueHandle_t qSRunGetSpeed_u32;

//_______________________________________________ RX SIDE
extern QueueHandle_t qRecData;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static bool traceWrapInteger (uint32_t* const value, const eBluetoothLogMember member, const uint32_t length);
static bool traceWrapFloat (
								float* const 			  value,
								const uint32_t 			  decimals,
								const eBluetoothLogMember member,
								const uint32_t 			  length
							);
static uint32_t traceUnwrapInteger 	 (uint8_t* const buffer, uint32_t begin, uint32_t size);
static bool 	traceUnwrapBool 	 (uint8_t* const buffer, uint32_t begin);
static float 	traceUnwrapFloat 	 (uint8_t* const buffer, uint32_t begin, uint32_t size, uint32_t decimals);
static uint32_t traceGetBoundCharNum (const uint32_t digits);

// Global function definitions -----------------------------------------------------------------------------------------

void traceInit (void) {}

void traceBluetooth (const eBluetoothLogMember destination, void* const data)
{
	// Select who sent the log request.
	switch (destination)
	{
		case BT_LOG_NAVI_N:
			xQueueOverwrite(qNaviN_f, (float* const)data);
			break;
		case BT_LOG_NAVI_E:
			xQueueOverwrite(qNaviE_f, (float* const)data);
			break;
		case BT_LOG_NAVI_PSI:
			xQueueOverwrite(qNaviPSI_f, (float* const)data);
			break;
		case BT_LOG_ENC_V:
			xQueueOverwrite(qEncV_f, (float* const)data);
			break;
		case BT_LOG_DIST_TOF_1:
			xQueueOverwrite(qDistToF1_u32, (uint32_t* const)data);
			break;
		case BT_LOG_DIST_TOF_2:
			xQueueOverwrite(qDistToF2_u32, (uint32_t* const)data);
			break;
		case BT_LOG_DIST_TOF_3:
			xQueueOverwrite(qDistToF3_u32, (uint32_t* const)data);
			break;
		case BT_LOG_DIST_SHARP_1:
			xQueueOverwrite(qDistSharp1_u32, (uint32_t* const)data);
			break;
		case BT_LOG_INERT_ACCEL_X:
			xQueueOverwrite(qInertAccelX_f, (float* const)data);
			break;
		case BT_LOG_INERT_ACCEL_Y:
			xQueueOverwrite(qInertAccelY_f, (float* const)data);
			break;
		case BT_LOG_INERT_ACCEL_Z:
			xQueueOverwrite(qInertAccelZ_f, (float* const)data);
			break;
		case BT_LOG_INERT_ANG_VEL_X:
			xQueueOverwrite(qInertAngVelX_f, (float* const)data);
			break;
		case BT_LOG_INERT_ANG_VEL_Y:
			xQueueOverwrite(qInertAngVelY_f, (float* const)data);
			break;
		case BT_LOG_INERT_ANG_VEL_Z:
			xQueueOverwrite(qInertAngVelZ_f, (float* const)data);
			break;
		case BT_LOG_STEER_WHEEL_ANGLE:
			xQueueOverwrite(qSteerWheelAngle_f, (float* const)data);
			break;
		case BT_LOG_SERVO_ANGLE:
			xQueueOverwrite(qServoAngle_f, (float* const)data);
			break;

		case BT_LOG_MTR_MAIN_BAT_VOLT:
			xQueueOverwrite(qMtrMainBatVolt_f, (float* const)data);
			break;
		case BT_LOG_MTR_SEC_BAT_VOLT:
			xQueueOverwrite(qMtrSecBatVolt_f, (float* const)data);
			break;
		case BT_LOG_MTR_MOTOR_CURR:
			xQueueOverwrite(qMtrCurr_f, (float* const)data);
			break;
		case BT_LOG_MTR_SYS_CURR:
			xQueueOverwrite(qMtrSysCurr_u32, (uint32_t* const)data);
			break;
		case BT_LOG_MTR_SERVO_CURR:
			xQueueOverwrite(qMtrServoCurr_u32, (uint32_t* const)data);
			break;

		case BT_LOG_LINE_LINE_NBR:
			xQueueOverwrite(qLineLineNbr_u32, (uint32_t* const)data);
			break;
		case BT_LOG_LINE_MAIN_LINE_POS:
			xQueueOverwrite(qLineMainLinePos_f, (float* const)data);
			break;
		case BT_LOG_LINE_SEC_LINE_POS:
			xQueueOverwrite(qLineSecLinePos_f, (float* const)data);
			break;

		case BT_LOG_MAZE_MAIN_SM:
			xQueueOverwrite(qMazeMainSM_u32, (uint32_t* const)data);
			break;
		case BT_LOG_MAZE_GET_KP:
			xQueueOverwrite(qMazeGetKp_f, (float* const)data);
			break;
		case BT_LOG_MAZE_GET_KD:
			xQueueOverwrite(qMazeGetKd_f, (float* const)data);
			break;
		case BT_LOG_MAZE_GET_SPEED:
			xQueueOverwrite(qMazeGetSpeed_u32, (uint32_t* const)data);
			break;
		case BT_LOG_MAZE_SEGMENTS:
			xQueueOverwrite(qMazeSegments_u32, (uint32_t* const)data);
			break;
		case BT_LOG_MAZE_ACT_STATE:
			xQueueOverwrite(qMazeActState_u32, (uint32_t* const)data);
			break;
		case BT_LOG_MAZE_ACT_KP:
			xQueueOverwrite(qMazeActKp_f, (float* const)data);
			break;
		case BT_LOG_MAZE_ACT_KD:
			xQueueOverwrite(qMazeActKd_f, (float* const)data);
			break;
		case BT_LOG_MAZE_ACT_SPEED:
			xQueueOverwrite(qMazeActSpeed_u32, (uint32_t* const)data);
			break;
		case BT_LOG_MAZE_INCLIN_SEGMENT:
			xQueueOverwrite(qMazeInclinSegment_u32, (uint32_t* const)data);
			break;

		case BT_LOG_SRUN_MAIN_SM:
			xQueueOverwrite(qSRunMainSM_u32, (uint32_t* const)data);
			break;
		case BT_LOG_SRUN_ACT_STATE:
			xQueueOverwrite(qSRunActState_u32, (uint32_t* const)data);
			break;
		case BT_LOG_SRUN_ACT_P:
			xQueueOverwrite(qSRunActP_f, (float* const)data);
			break;
		case BT_LOG_SRUN_ACT_KP:
			xQueueOverwrite(qSRunActKp_f, (float* const)data);
			break;
		case BT_LOG_SRUN_ACT_KD:
			xQueueOverwrite(qSRunActKd_f, (float* const)data);
			break;
		case BT_LOG_SRUN_ACT_SPEED:
			xQueueOverwrite(qSRunActSpeed_u32, (uint32_t* const)data);
			break;
		case BT_LOG_SRUN_GET_P:
			xQueueOverwrite(qSRunGetP_f, (float* const)data);
			break;
		case BT_LOG_SRUN_GET_KP:
			xQueueOverwrite(qSRunGetKp_f, (float* const)data);
			break;
		case BT_LOG_SRUN_GET_KD:
			xQueueOverwrite(qSRunGetKd_f, (float* const)data);
			break;
		case BT_LOG_SRUN_GET_SPEED:
			xQueueOverwrite(qSRunGetSpeed_u32, (uint32_t* const)data);
			break;
		default:
			break;
	}
}

void traceFlushData (void)
{
	float    naviN             = 0;
	float    naviE             = 0;
	float    naviPsi           = 0;
	float    encV              = 0;
	uint32_t distToF1		   = 0;
	uint32_t distToF2		   = 0;
	uint32_t distToF3		   = 0;
	uint32_t distSharp1		   = 0;
	float    inertAccelX       = 0;
	float    inertAccelY       = 0;
	float    inertAccelZ       = 0;
	float    inertAngVelX      = 0;
	float    inertAngVelY      = 0;
	float    inertAngVelZ      = 0;
	float    steerWheelAngle   = 0;
	float    servoAngle        = 0;

	float    mtrMainBatVolt    = 0;
	float    mtrSecBatVolt     = 0;
	float    mtrCurr           = 0;
	uint32_t mtrSysCurr        = 0;
	uint32_t mtrSrvCurr        = 0;

	uint32_t lineLineNbr	   = 0;
	float    lineMainLinePos   = 0;
	float    lineSecLinePos	   = 0;

	uint32_t mazeMainSm		   = 0;
	float    mazeGetKp		   = 0;
	float    mazeGetKd		   = 0;
	uint32_t mazeGetSpeed      = 0;
	uint32_t mazeSegments      = 0;
	uint32_t mazeActState	   = 0;
	float    mazeActKp		   = 0;
	float    mazeActKd         = 0;
	uint32_t mazeActSpeed	   = 0;
	uint32_t mazeInclinSegment = 0;

	uint32_t sRunMainSM        = 0;
	uint32_t sRunActState	   = 0;
	float    sRunActP		   = 0;
	float    sRunActKp		   = 0;
	float    sRunActKd		   = 0;
	uint32_t sRunActSpeed      = 0;
	float    sRunGetP		   = 0;
	float    sRunGetKp		   = 0;
	float    sRunGetKd         = 0;
	uint32_t sRunGetSpeed      = 0;

	// Collect and save the values from the queue into the bluetooth log structure.
	xQueueReceive(qNaviN_f, &naviN, 0);
	traceWrapFloat(&naviN, TRACE_DECIMALS_NAVI_N, BT_LOG_NAVI_N, BT_LOG_LEN_NAVI_N);

	xQueueReceive(qNaviE_f, &naviE, 0);
	traceWrapFloat(&naviE, TRACE_DECIMALS_NAVI_E, BT_LOG_NAVI_E, BT_LOG_LEN_NAVI_E);

	xQueueReceive(qNaviPSI_f, &naviPsi, 0);
	traceWrapFloat(&naviPsi, TRACE_DECIMALS_NAVI_PSI, BT_LOG_NAVI_PSI, BT_LOG_LEN_NAVI_PSI);

	xQueueReceive(qEncV_f, &encV, 0);
	traceWrapFloat(&encV, TRACE_DECIMALS_ENC_V, BT_LOG_ENC_V, BT_LOG_LEN_ENC_V);

	xQueueReceive(qDistToF1_u32, &distToF1, 0);
	traceWrapInteger(&distToF1, BT_LOG_DIST_TOF_1, BT_LOG_LEN_DIST_TOF_1);

	xQueueReceive(qDistToF2_u32, &distToF2, 0);
	traceWrapInteger(&distToF2, BT_LOG_DIST_TOF_2, BT_LOG_LEN_DIST_TOF_2);

	xQueueReceive(qDistToF3_u32, &distToF3, 0);
	traceWrapInteger(&distToF3, BT_LOG_DIST_TOF_3, BT_LOG_LEN_DIST_TOF_3);

	xQueueReceive(qDistSharp1_u32, &distSharp1, 0);
	traceWrapInteger(&distSharp1, BT_LOG_DIST_SHARP_1, BT_LOG_LEN_DIST_SHARP_1);

	xQueueReceive(qInertAccelX_f, &inertAccelX, 0);
	traceWrapFloat(&inertAccelX, TRACE_DECIMALS_INERT_ACCEL_X, BT_LOG_INERT_ACCEL_X, BT_LOG_LEN_INERT_ACCEL_X);

	xQueueReceive(qInertAccelY_f, &inertAccelY, 0);
	traceWrapFloat(&inertAccelY, TRACE_DECIMALS_INERT_ACCEL_Y, BT_LOG_INERT_ACCEL_Y, BT_LOG_LEN_INERT_ACCEL_Y);

	xQueueReceive(qInertAccelZ_f, &inertAccelZ, 0);
	traceWrapFloat(&inertAccelZ, TRACE_DECIMALS_INERT_ACCEL_Z, BT_LOG_INERT_ACCEL_Z, BT_LOG_LEN_INERT_ACCEL_Z);

	xQueueReceive(qInertAngVelX_f, &inertAngVelX, 0);
	traceWrapFloat(&inertAngVelX, TRACE_DECIMALS_INERT_ANG_VEL_X, BT_LOG_INERT_ANG_VEL_X, BT_LOG_LEN_INERT_ANG_VEL_X);

	xQueueReceive(qInertAngVelY_f, &inertAngVelY, 0);
	traceWrapFloat(&inertAngVelY, TRACE_DECIMALS_INERT_ANG_VEL_Y, BT_LOG_INERT_ANG_VEL_Y, BT_LOG_LEN_INERT_ANG_VEL_Y);

	xQueueReceive(qInertAngVelZ_f, &inertAngVelZ, 0);
	traceWrapFloat(&inertAngVelZ, TRACE_DECIMALS_INERT_ANG_VEL_Z, BT_LOG_INERT_ANG_VEL_Z, BT_LOG_LEN_INERT_ANG_VEL_Z);

	xQueueReceive(qSteerWheelAngle_f, &steerWheelAngle, 0);
	traceWrapFloat(&steerWheelAngle, TRACE_DECIMALS_STEER_WHEEL_ANGLE, BT_LOG_STEER_WHEEL_ANGLE, BT_LOG_LEN_STEER_WHEEL_ANGLE);

	xQueueReceive(qServoAngle_f, &servoAngle, 0);
	traceWrapFloat(&servoAngle, TRACE_DECIMALS_SERVO_ANGLE, BT_LOG_SERVO_ANGLE, BT_LOG_LEN_SERVO_ANGLE);


	xQueueReceive(qMtrMainBatVolt_f, &mtrMainBatVolt, 0);
	traceWrapFloat(&mtrMainBatVolt, TRACE_DECIMALS_MTR_MAIN_BAT_VOLT, BT_LOG_MTR_MAIN_BAT_VOLT, BT_LOG_LEN_MTR_MAIN_BAT_VOLT);

	xQueueReceive(qMtrSecBatVolt_f, &mtrSecBatVolt, 0);
	traceWrapFloat(&mtrSecBatVolt, TRACE_DECIMALS_MTR_SEC_BAT_VOLT, BT_LOG_MTR_SEC_BAT_VOLT, BT_LOG_LEN_MTR_SEC_BAT_VOLT);

	xQueueReceive(qMtrCurr_f, &mtrCurr, 0);
	traceWrapFloat(&mtrCurr, TRACE_DECIMALS_MTR_CURR, BT_LOG_MTR_MOTOR_CURR, BT_LOG_LEN_MTR_MOTOR_CURR);

	xQueueReceive(qMtrSysCurr_u32, &mtrSysCurr, 0);
	traceWrapInteger(&mtrSysCurr, BT_LOG_MTR_SYS_CURR, BT_LOG_LEN_MTR_SYS_CURR);

	xQueueReceive(qMtrServoCurr_u32, &mtrSrvCurr, 0);
	traceWrapInteger(&mtrSrvCurr, BT_LOG_MTR_SERVO_CURR, BT_LOG_LEN_MTR_SERVO_CURR);


	xQueueReceive(qLineLineNbr_u32, &lineLineNbr, 0);
	traceWrapInteger(&lineLineNbr, BT_LOG_LINE_LINE_NBR, BT_LOG_LEN_LINE_LINE_NBR);

	xQueueReceive(qLineMainLinePos_f, &lineMainLinePos, 0);
	traceWrapFloat(&lineMainLinePos, TRACE_DECIMALS_LINE_MAIN_LINE_POS, BT_LOG_LINE_MAIN_LINE_POS, BT_LOG_LEN_LINE_MAIN_LINE_POS);

	xQueueReceive(qLineSecLinePos_f, &lineSecLinePos, 0);
	traceWrapFloat(&lineSecLinePos, TRACE_DECIMALS_LINE_SEC_LINE_POS, BT_LOG_LINE_SEC_LINE_POS, BT_LOG_LEN_LINE_SEC_LINE_POS);


	xQueueReceive(qMazeMainSM_u32, &mazeMainSm, 0);
	traceWrapInteger(&mazeMainSm, BT_LOG_MAZE_MAIN_SM, BT_LOG_LEN_MAZE_MAIN_SM);

	xQueueReceive(qMazeGetKp_f, &mazeGetKp, 0);
	traceWrapFloat(&mazeGetKp, TRACE_DECIMALS_MAZE_GET_KP, BT_LOG_MAZE_GET_KP, BT_LOG_LEN_MAZE_GET_KP);

	xQueueReceive(qMazeGetKd_f, &mazeGetKd, 0);
	traceWrapFloat(&mazeGetKd, TRACE_DECIMALS_MAZE_GET_KD, BT_LOG_MAZE_GET_KD, BT_LOG_LEN_MAZE_GET_KD);

	xQueueReceive(qMazeGetSpeed_u32, &mazeGetSpeed, 0);
	traceWrapInteger(&mazeGetSpeed, BT_LOG_MAZE_GET_SPEED, BT_LOG_LEN_MAZE_GET_SPEED);

	xQueueReceive(qMazeSegments_u32, &mazeSegments, 0);
	traceWrapInteger(&mazeSegments, BT_LOG_MAZE_SEGMENTS, BT_LOG_LEN_MAZE_SEGENTS);

	xQueueReceive(qMazeActState_u32, &mazeActState, 0);
	traceWrapInteger(&mazeActState, BT_LOG_MAZE_ACT_STATE, BT_LOG_LEN_MAZE_ACT_STATE);

	xQueueReceive(qMazeActKp_f, &mazeActKp, 0);
	traceWrapFloat(&mazeActKp, TRACE_DECIMALS_MAZE_ACT_KP, BT_LOG_MAZE_ACT_KP, BT_LOG_LEN_MAZE_ACT_KP);

	xQueueReceive(qMazeActKd_f, &mazeActKd, 0);
	traceWrapFloat(&mazeActKd, TRACE_DECIMALS_MAZE_ACT_KD, BT_LOG_MAZE_ACT_KD, BT_LOG_LEN_MAZE_ACT_KD);

	xQueueReceive(qMazeActSpeed_u32, &mazeActSpeed, 0);
	traceWrapInteger(&mazeActSpeed, BT_LOG_MAZE_ACT_SPEED, BT_LOG_LEN_MAZE_ACT_SPEED);

	xQueueReceive(qMazeInclinSegment_u32, &mazeInclinSegment, 0);
	traceWrapInteger(&mazeInclinSegment, BT_LOG_MAZE_INCLIN_SEGMENT, BT_LOG_LEN_MAZE_INCLIN_SEGMENT);


	xQueueReceive(qSRunMainSM_u32, &sRunMainSM, 0);
	traceWrapInteger(&sRunMainSM, BT_LOG_SRUN_MAIN_SM, BT_LOG_LEN_SRUN_MAIN_SM);

	xQueueReceive(qSRunActState_u32, &sRunActState, 0);
	traceWrapInteger(&sRunActState, BT_LOG_SRUN_ACT_STATE, BT_LOG_LEN_SRUN_ACT_STATE);

	xQueueReceive(qSRunActP_f, &sRunActP, 0);
	traceWrapFloat(&sRunActP, TRACE_DECIMALS_SRUN_ACT_P, BT_LOG_SRUN_ACT_P, BT_LOG_LEN_SRUN_ACT_P);

	xQueueReceive(qSRunActKp_f, &sRunActKp, 0);
	traceWrapFloat(&sRunActKp, TRACE_DECIMALS_SRUN_ACT_KP, BT_LOG_SRUN_ACT_KP, BT_LOG_LEN_SRUN_ACT_KP);

	xQueueReceive(qSRunActKd_f, &sRunActKd, 0);
	traceWrapFloat(&sRunActKd, TRACE_DECIMALS_SRUN_ACT_KD, BT_LOG_SRUN_ACT_KD, BT_LOG_LEN_SRUN_ACT_KD);

	xQueueReceive(qSRunActSpeed_u32, &sRunActSpeed, 0);
	traceWrapInteger(&sRunActSpeed, BT_LOG_SRUN_ACT_SPEED, BT_LOG_LEN_SRUN_ACT_SPEED);

	xQueueReceive(qSRunGetP_f, &sRunGetP, 0);
	traceWrapFloat(&sRunGetP, TRACE_DECIMALS_SRUN_GET_P, BT_LOG_SRUN_GET_P, BT_LOG_LEN_SRUN_GET_P);

	xQueueReceive(qSRunGetKp_f, &sRunGetKp, 0);
	traceWrapFloat(&sRunGetKp, TRACE_DECIMALS_SRUN_GET_KP, BT_LOG_SRUN_GET_KP, BT_LOG_LEN_SRUN_GET_KP);

	xQueueReceive(qSRunGetKd_f, &sRunGetKd, 0);
	traceWrapFloat(&sRunGetKd, TRACE_DECIMALS_SRUN_GET_KD, BT_LOG_SRUN_GET_KD, BT_LOG_LEN_SRUN_GET_KD);

	xQueueReceive(qSRunGetSpeed_u32, &sRunGetSpeed, 0);
	traceWrapInteger(&sRunGetSpeed, BT_LOG_SRUN_GET_SPEED, BT_LOG_LEN_SRUN_GET_SPEED);

	// Send out the bluetooth log.
	bspBtBufferFlush();
}

void traceProcessRxData (uint8_t* const buffer)
{
	cTRACE_RX_DATA rxData;
	uint8_t rxDataSize = 0;
	uint8_t index = 0;

	index += TRACE_REC_HEADER;

	rxDataSize = traceUnwrapInteger(buffer, index, TRACE_REC_SIZE);
	index += TRACE_REC_SIZE;

	if (buffer[0] == 'B' && buffer[1] == 'B' && TRACE_REC_MSG_SIZE == rxDataSize + TRACE_REC_HEADER + TRACE_REC_SIZE)
	{
		rxData.StopCar = traceUnwrapBool(buffer, index);
		index += TRACE_REC_STOP_CAR;


		rxData.MazeMainSMReset = traceUnwrapBool(buffer, index);
		index += TRACE_REC_MAZE_MAIN_SM_RESET;

		rxData.MazeMainSMResetTo = traceUnwrapInteger(buffer, index, TRACE_REC_MAZE_MAIN_SM_RESET_TO);
		index += TRACE_REC_MAZE_MAIN_SM_RESET_TO;

		rxData.MazeGetState = traceUnwrapInteger(buffer, index, TRACE_REC_MAZE_GET_STATE);
		index += TRACE_REC_MAZE_GET_STATE;

		rxData.MazeSetState = traceUnwrapInteger(buffer, index, TRACE_REC_MAZE_SET_STATE);
		index += TRACE_REC_MAZE_SET_STATE;

		rxData.MazeSetKp = traceUnwrapFloat(buffer, index, TRACE_REC_MAZE_SET_KP, TRACE_DECIMALS_MAZE_SET_KP);
		index += TRACE_REC_MAZE_SET_KP;

		rxData.MazeSetKd = traceUnwrapFloat(buffer, index, TRACE_REC_MAZE_SET_KD, TRACE_DECIMALS_MAZE_SET_KD);
		index += TRACE_REC_MAZE_SET_KD;

		rxData.MazeSetSpeed = traceUnwrapInteger(buffer, index, TRACE_REC_MAZE_SET_SPEED);
		index += TRACE_REC_MAZE_SET_SPEED;


		rxData.SRunTryOvertake = traceUnwrapBool(buffer, index);
		index += TRACE_REC_SRUN_TRY_OVERTAKE;

		rxData.SRunHardReset = traceUnwrapBool(buffer, index);
		index += TRACE_REC_SRUN_HARD_RESET;

		rxData.SRunSoftReset = traceUnwrapBool(buffer, index);
		index += TRACE_REC_SRUN_SOFT_RESET;

		rxData.SRunSoftResetTo = traceUnwrapInteger(buffer, index, TRACE_REC_SRUN_SOFT_RESET_TO);
		index += TRACE_REC_SRUN_SOFT_RESET_TO;

		rxData.SRunGetState = traceUnwrapInteger(buffer, index, TRACE_REC_SRUN_GET_STATE);
		index += TRACE_REC_SRUN_GET_STATE;

		rxData.SRunSetState = traceUnwrapInteger(buffer, index, TRACE_REC_SRUN_SET_STATE);
		index += TRACE_REC_SRUN_SET_STATE;

		rxData.SRunSetP = traceUnwrapFloat(buffer, index, TRACE_REC_SRUN_SET_P, TRACE_DECIMALS_SRUN_SET_P);
		index += TRACE_REC_SRUN_SET_P;

		rxData.SRunSetKp = traceUnwrapFloat(buffer, index, TRACE_REC_SRUN_SET_KP, TRACE_DECIMALS_SRUN_SET_KP);
		index += TRACE_REC_SRUN_SET_KP;

		rxData.SRunSetKd = traceUnwrapFloat(buffer, index, TRACE_REC_SRUN_SET_KD, TRACE_DECIMALS_SRUN_SET_KD);
		index += TRACE_DECIMALS_SRUN_SET_KD;

		rxData.SRunSetSpeed = traceUnwrapInteger(buffer, index, TRACE_REC_SRUN_SET_SPEED);
		index += TRACE_REC_SRUN_SET_SPEED;
	}

	xQueueOverwrite(qRecData, &rxData);
}

cTRACE_RX_DATA traceGetRxData (void)
{
	cTRACE_RX_DATA retStruct;

	xQueuePeek(qRecData, &retStruct, 0);

	return retStruct;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//**********************************************************************************************************************
//! Converts a integer value to a unit8_t array so it can be sent out on bluetooth.
//!
//! @param value		value that will be sent out
//! @param member		determines which how sent and what type of data
//! @param length		how many character has to be sent out
//! @return				was the conversion successful or not
//**********************************************************************************************************************
static bool traceWrapInteger (uint32_t* const value, const eBluetoothLogMember member, const uint32_t length)
{
	bool traced = false;
	uint8_t buffer[TRACE_MAX_NUMBER_OF_DIGITS];
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
	traced = bspLogMemberUpdate(member, buffer, length);

	return traced;
}

//**********************************************************************************************************************
//!	Converts a bool value to a unit8_t array so it can be sent out on bluetooth.
//!
//! @param value		value that will be sent out
//! @param decimals		how many decimals of the double has to be sent out
//! @param member		determines which how sent and what type of data
//! @param length		how many character has to be sent out
//! @return				was the conversion successful or not
//**********************************************************************************************************************
static bool traceWrapFloat (
								float* const 			  value,
								const uint32_t 			  decimals,
								const eBluetoothLogMember member,
								const uint32_t 			  length
							)
{
	bool traced = false;
	uint8_t  buffer[TRACE_MAX_NUMBER_OF_DIGITS];
	uint32_t mul;
	int32_t  bound;
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
		if (*value < 0.0f)
		{
			// Negative sign.

			// Determine lower bound for the lowest available value.
			bound = traceGetBoundCharNum(length-1) * -1;

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
		traced = bspLogMemberUpdate(member, buffer, length);
	}

	return traced;
}

//**********************************************************************************************************************
//! Determines a power of 10 that can be used as a bound. With this the vlaue can be satured so it can be sent out in a
//! given lenght and decimals value.
//!
//! @param digits		how many digit the value consist of
//! @return				the calculated bound
//**********************************************************************************************************************
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

//**********************************************************************************************************************
//! TODO
//! @param buffer
//! @param begin
//! @param size
//! @return
//**********************************************************************************************************************
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

//**********************************************************************************************************************
//! TODO
//! @param buffer
//! @param begin
//! @return
//**********************************************************************************************************************
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

//**********************************************************************************************************************
//! TODO
//! @param buffer
//! @param begin
//! @param size
//! @param decimals
//! @return
//**********************************************************************************************************************
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

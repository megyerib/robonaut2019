////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      trace.h
//!  \brief     Send diagnostics data.
//!  \details   This module collects data and send them through a communication port.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_bluetooth.h"
#include "handler_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Defines -------------------------------------------------------------------------------------------------------------

//TODO Collect trace defines.

//!	@brief	Defines for the structure member that how many decimals need to be stored.
//______________________________________________________________________ TX SIDE
#define TRACE_DECIMALS_NAVI_N					2
#define TRACE_DECIMALS_NAVI_E					2
#define TRACE_DECIMALS_NAVI_PSI					4
#define TRACE_DECIMALS_ENC_V					2
#define TRACE_DECIMALS_INERT_ACCEL_X			3
#define TRACE_DECIMALS_INERT_ACCEL_Y			3
#define TRACE_DECIMALS_INERT_ACCEL_Z			3
#define TRACE_DECIMALS_INERT_ANG_VEL_X			4
#define TRACE_DECIMALS_INERT_ANG_VEL_Y			4
#define TRACE_DECIMALS_INERT_ANG_VEL_Z			4
#define TRACE_DECIMALS_STEER_WHEEL_ANGLE		4
#define TRACE_DECIMALS_SERVO_ANGLE				4

#define TRACE_DECIMALS_MTR_MAIN_BAT_VOLT		3
#define TRACE_DECIMALS_MTR_SEC_BAT_VOLT			3
#define TRACE_DECIMALS_MTR_CURR					3

#define TRACE_DECIMALS_LINE_MAIN_LINE_POS		2
#define TRACE_DECIMALS_LINE_SEC_LINE_POS		2

#define TRACE_DECIMALS_MAZE_GET_KP				2
#define TRACE_DECIMALS_MAZE_GET_KD				2
#define TRACE_DECIMALS_MAZE_ACT_KP				2
#define TRACE_DECIMALS_MAZE_ACT_KD				2

#define TRACE_DECIMALS_SRUN_ACT_P				3
#define TRACE_DECIMALS_SRUN_ACT_KP				2
#define TRACE_DECIMALS_SRUN_ACT_KD				2
#define TRACE_DECIMALS_SRUN_GET_P				3
#define TRACE_DECIMALS_SRUN_GET_KP				2
#define TRACE_DECIMALS_SRUN_GET_KD				2


//______________________________________________________________________ RX SIDE
#define TRACE_REC_HEADER						2
#define TRACE_REC_SIZE							3

#define TRACE_REC_STOP_CAR						1

#define TRACE_REC_MAZE_MAIN_SM_RESET			1
#define TRACE_REC_MAZE_MAIN_SM_RESET_TO			1
#define TRACE_REC_MAZE_GET_STATE				1
#define TRACE_REC_MAZE_SET_STATE				1
#define TRACE_REC_MAZE_SET_KP					5
#define TRACE_REC_MAZE_SET_KD					5
#define TRACE_REC_MAZE_SET_SPEED				2

#define TRACE_REC_SRUN_TRY_OVERTAKE				1
#define TRACE_REC_SRUN_HARD_RESET				1
#define TRACE_REC_SRUN_SOFT_RESET				1
#define TRACE_REC_SRUN_SOFT_RESET_TO			2
#define TRACE_REC_SRUN_GET_STATE				2
#define TRACE_REC_SRUN_SET_STATE				2
#define TRACE_REC_SRUN_SET_P					6
#define TRACE_REC_SRUN_SET_KP					5
#define TRACE_REC_SRUN_SET_KD					5
#define TRACE_REC_SRUN_SET_SPEED				2

#define TRACE_REC_MSG_SIZE						(	TRACE_REC_HEADER +						\
													TRACE_REC_SIZE +						\
													TRACE_REC_STOP_CAR +					\
													TRACE_REC_MAZE_MAIN_SM_RESET +			\
													TRACE_REC_MAZE_MAIN_SM_RESET_TO +		\
													TRACE_REC_MAZE_GET_STATE +				\
													TRACE_REC_MAZE_SET_STATE +				\
													TRACE_REC_MAZE_SET_KP +					\
													TRACE_REC_MAZE_SET_KD +					\
													TRACE_REC_MAZE_SET_SPEED +				\
													TRACE_REC_SRUN_TRY_OVERTAKE +			\
													TRACE_REC_SRUN_HARD_RESET +				\
													TRACE_REC_SRUN_SOFT_RESET +				\
													TRACE_REC_SRUN_SOFT_RESET_TO +			\
													TRACE_REC_SRUN_GET_STATE +				\
													TRACE_REC_SRUN_SET_STATE +				\
													TRACE_REC_SRUN_SET_P +					\
													TRACE_REC_SRUN_SET_KP +					\
													TRACE_REC_SRUN_SET_KD +					\
													TRACE_REC_SRUN_SET_SPEED				\
												)

#define TRACE_DECIMALS_MAZE_SET_KP				2
#define TRACE_DECIMALS_MAZE_SET_KD				2

#define TRACE_DECIMALS_SRUN_SET_P				2
#define TRACE_DECIMALS_SRUN_SET_KP				2
#define TRACE_DECIMALS_SRUN_SET_KD				2


// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	bool     StopCar;			// 1

	bool     MazeMainSMReset;	// 2
	uint32_t MazeMainSMResetTo;
	uint32_t MazeGetState;
	uint32_t MazeSetState;
	float    MazeSetKp;
	float    MazeSetKd;
	uint32_t MazeSetSpeed;		// 8

	bool     SRunTryOvertake;	// 9
	bool     SRunHardReset;
	bool	 SRunSoftReset;
	uint32_t SRunSoftResetTo;
	uint32_t SRunGetState;
	uint32_t SRunSetState;
	float	 SRunSetP;
	float	 SRunSetKp;
	float 	 SRunSetKd;
	uint32_t SRunSetSpeed;		// 18
} cTRACE_RX_DATA;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//!	Initializes the trace module.
//**********************************************************************************************************************
void traceInit (void);

//**********************************************************************************************************************
//! Saves a data for sending out on bluetooth into a queue.
//! Recalling of this function will override the previous saved value.
//!
//! @param destination		determines the type of the data
//! @param data				value that needs to be traced
//**********************************************************************************************************************
void traceBluetooth (const eBluetoothLogMember destination, void* const data);

//**********************************************************************************************************************
//! Sends out the bluetooth log message. The latest data in the queues will be sent out.
//**********************************************************************************************************************
void traceFlushData (void);

//**********************************************************************************************************************
//! @return	TODO comment
//**********************************************************************************************************************
cTRACE_RX_DATA traceGetRxData (void);

//**********************************************************************************************************************
//! @param buffer
//! @return	TODO comment
//**********************************************************************************************************************
void traceProcessRxData (uint8_t* const buffer);

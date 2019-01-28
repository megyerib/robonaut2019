////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      trace.h
//!  \brief     Send diagnostics data.
//!  \details   This module collects data and send them through a communication port.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include <bsp_bluetooth.h>
#include <handler_common.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


// Defines -------------------------------------------------------------------------------------------------------------

//!	@brief	Defines for the structure member that how many decimals need to be stored.

#define TRACE_DECIMALS_NAVI_N					2
#define TRACE_DECIMALS_NAVI_E					2
#define TRACE_DECIMALS_NAVI_PSI					4
#define TRACE_DECIMALS_ENC_V					2
#define TRACE_DECIMALS_INERT_ACCEL_X			2
#define TRACE_DECIMALS_INERT_ACCEL_Y			2
#define TRACE_DECIMALS_INERT_ACCEL_Z			2
#define TRACE_DECIMALS_INERT_ANG_VEL_X			2
#define TRACE_DECIMALS_INERT_ANG_VEL_Y			2
#define TRACE_DECIMALS_INERT_ANG_VEL_Z			2
#define TRACE_DECIMALS_STEER_WHEEL_ANGLE		4
#define TRACE_DECIMALS_SERVO_ANGLE				4

#define TRACE_DECIMALS_MTR_MAIN_BAT_VOLT		3
#define TRACE_DECIMALS_MTR_SEC_BAT_VOLT			3
#define TRACE_DECIMALS_MTR_CURR					3

#define TRACE_DECIMALS_LINE_MAIN_LINE_POS		3
#define TRACE_DECIMALS_LINE_SEC_LINE_POS		3

#define TRACE_DECIMALS_MAZE_GET_KP				2
#define TRACE_DECIMALS_MAZE_GET_KD				2
#define TRACE_DECIMALS_MAZE_ACT_KP				2
#define TRACE_DECIMALS_MAZE_ACT_KD				2

#define TRACE_DECIMALS_SRUN_ACT_P				2
#define TRACE_DECIMALS_SRUN_ACT_KP				2
#define TRACE_DECIMALS_SRUN_ACT_KD				2
#define TRACE_DECIMALS_SRUN_GET_P				2
#define TRACE_DECIMALS_SRUN_GET_KP				2
#define TRACE_DECIMALS_SRUN_GET_KD				2


//____________ RECEVIE END_________________________________________________
#define TRACE_REC_HEADER						2
#define TRACE_REC_SIZE							2
#define TRACE_REC_ACCEL_SIZE					3
#define TRACE_REC_STEER_SIZE					3
#define TRACE_REC_PD_TD_SIZE    				8
#define TRACE_REC_PD_TD_DECIMALS				4
#define TRACE_REC_PD_KP_SIZE					8
#define TRACE_REC_PD_KP_DECIMALS				4

#define TRACE_REC_MSG_SIZE					(	TRACE_REC_HEADER +		\
												TRACE_REC_SIZE +		\
												1 +						\
												1 +				 		\
												1 +						\
												1 +						\
												TRACE_REC_ACCEL_SIZE +	\
												1 +						\
												TRACE_REC_STEER_SIZE +	\
												1 +						\
												TRACE_REC_PD_TD_SIZE +	\
												1 +						\
												TRACE_REC_PD_KP_SIZE	\
											)



// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	bool    RecCmdStop;			// 1
	bool    RecCmdFollowLine;
	bool    RecCmdSelfTest;
	bool    RecCmdAccelerate;
	uint8_t RecDataAccelerate;	// 5
	bool    RecCmdSteer;
	uint8_t RecDataSteer;
	bool    RecCmdPdTd;
	double  RecDataPdTd_d;
	bool    RecCmdPdKp_x;
	double  RecDataPdKp_d;
} cTraceRxBluetoothStruct;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

//!	Initializes the trace module.
void traceInit (void);

//! Saves a data for sending out on bluetooth into a queue.
//! Recalling of this function will override the previous saved value.
//!
//! @param destination		determines the type of the data
//! @param data				value that needs to be traced
void traceBluetooth (const eBluetoothLogMember destination, void* const data);

//! Sends out the bluetooth log message. The latest data in the queues will be sent out.
void traceFlushData (void);

cTraceRxBluetoothStruct traceReceiveBluetooth (void);

//! TODO
cTraceRxBluetoothStruct traceProcessRxData (uint8_t* const buffer);

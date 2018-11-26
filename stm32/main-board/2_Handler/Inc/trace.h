////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      trace.h
//!  \brief     Send diagnostics data
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f4xx_hal.h"

#include "../../2_Handler/Inc/bcm_BluetoothCommunication.h"

// Defines -------------------------------------------------------------------------------------------------------------

//!	@brief	Defines for the structure member that how many decimals need to be stored.
#define TRACE_DECIMALS_SERVO_ANGLE				4
#define TRACE_DECIMALS_INERT_ACCEL_X			2
#define TRACE_DECIMALS_INERT_ACCEL_Y			2
#define TRACE_DECIMALS_INERT_ACCEL_Z			2
#define TRACE_DECIMALS_INERT_ANG_VEL_X			2
#define TRACE_DECIMALS_INERT_ANG_VEL_Y			2
#define TRACE_DECIMALS_INERT_ANG_VEL_Z			2
#define TRACE_DECIMALS_NAVI_N					2
#define TRACE_DECIMALS_NAVI_E					2
#define TRACE_DECIMALS_NAVI_THETA				4
#define TRACE_DECIMALS_ENC_VEL					2
#define TRACE_DECIMALS_MTR_MAIN_BAT_VOLT		3
#define TRACE_DECIMALS_MTR_SEC_BAT_VOLT			3
#define TRACE_DECIMALS_MTR_CURR					3
#define TRACE_DECIMALS_CTRL_MTR_CURR			2


// Typedefs ------------------------------------------------------------------------------------------------------------

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

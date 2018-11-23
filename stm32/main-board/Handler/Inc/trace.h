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

#include "bcm_BluetoothCommunication.h"

// Defines -------------------------------------------------------------------------------------------------------------

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

void traceInit (void);

void traceBluetooth (const eBluetoothLogMember destination, void* const data);

void traceFlushData (void);

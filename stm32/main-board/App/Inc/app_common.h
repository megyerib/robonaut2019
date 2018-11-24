////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_common.h
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define 	DEFAULT_STACK_SIZE 				128
#define		TASK_CDT_PRIO					tskIDLE_PRIORITY+5
#define 	TASK_SC_PRIO					tskIDLE_PRIORITY+4
#define 	TASK_NAVI_PRIO					tskIDLE_PRIORITY+3
#define 	TASK_SHARP_PRIO					tskIDLE_PRIORITY+2
#define 	TASK_SRV_PRIO					tskIDLE_PRIORITY+1


#define		TASK_DELAY_16_MS				16		// Depends on the Tick
#define		TASK_DELAY_20_MS				20
#define		TASK_DELAY_40_MS				40
#define		TASK_DELAY_100_MS				100


// Typedefs ------------------------------------------------------------------------------------------------------------
// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------



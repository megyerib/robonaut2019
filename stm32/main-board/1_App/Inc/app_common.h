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

// Low priority numbers denote low priority tasks.
// https://www.freertos.org/RTOS-task-priority.html

// TODO rethink priorities
#define 	DEFAULT_STACK_SIZE 				128

#define 	TASK_INRT_CAL_PRIO				tskIDLE_PRIORITY+6
#define		TASK_CDT_PRIO					tskIDLE_PRIORITY+4
#define 	TASK_NAVI_PRIO					tskIDLE_PRIORITY+7

#define 	TASK_SHARP_PRIO					tskIDLE_PRIORITY+3
#define 	TASK_SRV_PRIO					tskIDLE_PRIORITY+2

#define 	TASK_SC_PRIO					tskIDLE_PRIORITY+5
#define     TASK_LINE_FOLLOW_PRIO           tskIDLE_PRIORITY+1
#define     TASK_INERT_PRIO                 tskIDLE_PRIORITY+7
#define 	TASK_QSM_PRIO					tskIDLE_PRIORITY+5

// Task delays
// Depends on the Tick

#define 	TASK_DELAY_5_MS					5
#define		TASK_DELAY_16_MS				16
#define		TASK_DELAY_20_MS				20
#define		TASK_DELAY_40_MS				40
#define		TASK_DELAY_100_MS				100


// Typedefs ------------------------------------------------------------------------------------------------------------
// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------



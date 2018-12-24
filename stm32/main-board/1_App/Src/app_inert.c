////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_inert.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_inert.h"
#include "app_common.h"
#include "inert.h"
#include "FreeRTOS.h"
#include "task.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_inert(void)
{
	inertInit();

	xTaskCreate(Task_inert,
				"TASK_INERT",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_INERT_PRIO,
				NULL);
}

void Task_inert(void* p)
{
	ACCEL a;

	while (1)
	{
		inertTriggerMeasurement();

		a = inertGetAccel();

		vTaskDelay(10);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END -----------------------------------------------------------------------------------------------------------------

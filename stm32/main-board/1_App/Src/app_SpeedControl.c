////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_SpeedControl.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include "app_SpeedControl.h"

#include "scm_SpeedControlModule.h"
#include "trace.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_SControl(void)
{
	scmInitControllerPI();

	xTaskCreate(Task_SControl,
				"TASK_SCNTRL",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SC_PRIO,
				NULL);
}

void Task_SControl(void* p)
{
	(void)p;

	double yn;
	double rn = 1;
	double cntr = 0;

	//UBaseType_t scmStackUsage;
	//scmStackUsage = uxTaskGetStackHighWaterMark(NULL);

	while(1)
	{
		if (cntr == 625)
		{
			rn *= -1;
			cntr = 0;
		}

		yn = scmControlLoop(rn);

		traceBluetooth(BCM_LOG_CTR_MTR_CURR, &yn);

		cntr++;
		vTaskDelay(TASK_DELAY_16_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

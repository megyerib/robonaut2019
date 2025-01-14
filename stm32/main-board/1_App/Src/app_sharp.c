////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_servo.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ------------------------------- Includes -------------------------------- //

#include "bsp_sharp.h"
#include "app_sharp.h"

#include "app_common.h"
#include "trace.h"

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

EventGroupHandle_t event_sharp;

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Sharp (void)
{
	semSharp = xSemaphoreCreateBinary();
	event_sharp = xEventGroupCreate();

	if(semSharp != NULL)
	{
		xSemaphoreGive(semSharp);
	}

	xTaskCreate(Task_Sharp,
				"TASK_SHARP",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SHARP_PRIO,
				NULL);
}

void Task_Sharp (void* p)
{
	(void)p;

	cMEASUREMENT_DIST sharp;

	sharp.Distance = 0;

	while(1)
	{
		sharpTriggerAdc();
		sharp = sharpGetMeasurement();
		/*if(sharp.Distance > 40)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			xEventGroupClearBits(event_sharp, 1);
		}
		else
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			xEventGroupSetBits(event_sharp, 1);
		}*/

		traceBluetooth(BT_LOG_DIST_SHARP_1, &sharp.Distance);

		vTaskDelay(TASK_DELAY_40_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

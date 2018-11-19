////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_servo.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ------------------------------- Includes -------------------------------- //

#include "app_sharp.h"

#include "app_common.h"
#include "bsp_servo.h"

#include "task.h"
#include "sds_SharpDistanceSensor.h"

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

EventGroupHandle_t event_sharp;

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Sharp(void * p)
{
	(void)p;

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

void Task_Sharp(void* p)
{
	(void)p;

	uint16_t sharp_distance = 0;

	while(1)
	{
		sds_ADC_Conversion();
		sharp_distance = sds_GetDistance();
		if(sharp_distance > 40)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			xEventGroupClearBits(event_sharp, 1);
		}
		else
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			xEventGroupSetBits(event_sharp, 1);
		}

		bcmTraceSharpDistance(sharp_distance);

		vTaskDelay(TASK_DELAY_40_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

/*
 * task_sharp.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

// ------------------------------- Includes -------------------------------- //

#include "../Inc/task_sharp.h"

#include "bsp_servo.h"
#include "task.h"
#include "sds_SharpDistanceSensor.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

EventGroupHandle_t event_sharp;

// --------------------------------------------------------------------------//

// ------------------------------- Functions --------------------------------//

void TaskInit_Sharp(void * p)
{
	(void)p;

	semSharp = xSemaphoreCreateBinary();
	event_sharp = xEventGroupCreate();

	if(semSharp != NULL)
	{
		xSemaphoreGive(semSharp);
		//BSP_Sharp_ADC_Init();
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
		vTaskDelay(BSP_DELAY_40_MS);
	}
}

// --------------------------------------------------------------------------//

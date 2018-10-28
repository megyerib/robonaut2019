/*
 * task_sharp.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

// ------------------------------- Includes -------------------------------- //

#include "bsp.h"
#include "task_sharp.h"
#include "sds_SharpDistanceSensore.h"
#include "task.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define		DELAY_40_MS			40		// Depends on the Tick
#define		DELAY_40_MS			40		// Depends on the Tick

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Functions --------------------------------//

void TaskInit_Sharp(void * p)
{
	(void)p;
	semSharp = xSemaphoreCreateBinary();
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

void Task_Sharp(void * p)
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
		}
		else
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
		vTaskDelay(DELAY_40_MS);
	}
}

// --------------------------------------------------------------------------//

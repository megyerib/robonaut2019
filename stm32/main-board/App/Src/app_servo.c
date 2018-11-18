////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_servo.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Includes ------------------------------------------------------------------------------------------------------------

#include <app_servo.h>
#include "bsp_common.h"
#include "app_common.h"

#include "sch_ServoControlHandler.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

// Defines -------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern EventGroupHandle_t event_sharp;

// --------------------------------------------------------------------------//

// ------------------------------- Functions --------------------------------//

void TaskInit_Servo(void)
{
	sch_Servo_Init();

	xTaskCreate(Task_Servo,
				"TASK_SERVO",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SRV_PRIO,
				NULL);
}

void Task_Servo(void* p)
{
	(void)p;

	EventBits_t bits;
	double degree;

	while(1)
	{
		bits = xEventGroupGetBits(event_sharp);

		if(bits == 1)
		{
			sch_Set_Servo_Angle(2*PI/3);
		}
		else
		{
			sch_Set_Servo_Angle(PI/3);
		}

		// Angle in degree
		degree = sch_Get_Servo_Angle() * 180 /PI;

		// No warning
		degree = degree + 1 - 1;

		vTaskDelay(TASK_DELAY_20_MS);
	}
}

// --------------------------------------------------------------------------//



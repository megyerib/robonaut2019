////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_servo.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Includes ------------------------------------------------------------------------------------------------------------

#include "app_servo.h"
#include "bsp_common.h"

#include "sch_ServoControlHandler.h"
#include "trace.h"

#include "event_groups.h"

// Typedefs ------------------------------------------------------------------------------------------------------------

#define LOG_RATE 	4

// Local (static) & extern variables -----------------------------------------------------------------------------------

extern EventGroupHandle_t event_sharp;

extern QueueHandle_t qServoAngle_d;

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

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
	double theta;
	double degree;
	uint32_t rate = LOG_RATE;

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

		theta = sch_Get_Servo_Angle();

		// Angle in degree
		degree = theta * 180 /PI;
		// No warning
		degree = degree + 1 - 1;

		if(rate == 0)
		{
			traceBluetooth(BCM_LOG_SERVO_ANGLE, &theta);
			rate = LOG_RATE;
		}
		else
		{
			rate--;
		}


		vTaskDelay(TASK_DELAY_20_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------


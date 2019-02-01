////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_speedcal.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_speedcal.h"
#include "remote.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_common.h"
#include "gpio.h"
#include "line.h"
#include "motor.h"
#include "bsp_servo.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define TASK_DELAY   (5u) /* ms */

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static int actuateEnabled;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void remoteHandle();
static void lineFollow();

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_SpeedCalibration(void)
{
	xTaskCreate(Task_SpeedCalibration,
				"TASK_SPEEDCALIBRATION",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SPEED_CAL_PRIO,
				NULL);
}

void Task_SpeedCalibration(void* p)
{


	while(1)
	{
		remoteHandle();

		if (actuateEnabled)
			lineFollow();

		// LINE MEASURING __________________________________

		// TRACE ___________________________________________

		// END DELAY _______________________________________

		vTaskDelay(TASK_DELAY);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void remoteHandle()
{
	if (remoteGetState())
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		actuateEnabled = 1;
	}
	else
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		actuateEnabled = 0;
	}
}

static void lineFollow()
{
	static float angle;
	static float line_pos  = 0;
	static float line_diff = 0;
	static float prevline  = 0;

	static float K_P       =  0.025f;
	static float K_D       =  3.68f;
	static int	 motor_d   = 19;

	static float P;
	static float D;

	// STEERING ________________________________________

	prevline = line_pos;

	line_pos = lineGetSingle() / 1000; // m -> mm

	line_diff = line_pos - prevline;

	P = line_pos  * K_P;

	D = line_diff * K_D;

	angle = -0.75f * (P + D);

	// ACTUATE _________________________________________

	// Note: Disabling must happen outside

	motorSetDutyCycle(motor_d);
	servoSetAngle(angle);
}

// END -----------------------------------------------------------------------------------------------------------------

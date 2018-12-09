////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_steeringDemo.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include "app_steeringDemo.h"
#include "trace.h"
#include "servo.h"
#include "motor.h"
#include "line.h"
#include "bsp_common.h"
#include "bsp_uart.h"
#include "controller.h"
#include "remote.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define STTERINGDEMO_TASK_DELAY 5

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

/*static double p_a;		// m
static double p_meas;	// m
static double e;		// m
static double p;		// m
static double phi_a;	// deg
static double v;		// m/s
static double L;		// m

static double Kp;		//
static double Td;		// sec
static double T;		// sec

uint8_t pwm = 10;

static cFirstOrderTF contrPD;*/

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_steeringDemo(void)
{
    servoInit();

    lineInit();

    remoteInit();

    xTaskCreate(Task_steeringDemo,
                "TASK_DEMO",
                DEFAULT_STACK_SIZE,
                NULL,
                TASK_SRV_PRIO,
                NULL);
}

float angle;
float line_pos  = 0;
float line_diff = 0;
float prevline = 0;

float P, D;

void Task_steeringDemo(void* p)
{
	servoSetAngle(0);

	for (float i = 0.0f; i > -PI/2.0f; i -= 0.01)
	{
		servoSetAngle(i);
	}

	while(1)
    {
		// REMOTE CONTROL __________________________________

		if (remoteGetState())
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			vTaskDelay(STTERINGDEMO_TASK_DELAY);
			continue; // Skip the loop iteration
		}

		// TRACTION ________________________________________

		motorSetDutyCycle(12);

		// STEERING ________________________________________

		prevline = line_pos;

		line_pos = (float) lineGet().d;

		line_diff = line_pos - prevline;

		P = line_pos  * (1.0 / 100.0f);

		D = line_diff * 1.8f;

		angle = -0.75f * (P + D);

		servoSetAngle(angle);
		traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);

		// TRACE ___________________________________________

		//traceBluetooth(BCM_LOG_LINE_D, &linepos);
		//traceBluetooth(BCM_LOG_ENC_VEL, 10);

		// END DELAY _______________________________________

		vTaskDelay(STTERINGDEMO_TASK_DELAY);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

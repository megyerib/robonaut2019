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

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_steeringDemo(void)
{
    servoInit();

    lineInit();

    xTaskCreate(Task_steeringDemo,
                "TASK_DEMO",
                DEFAULT_STACK_SIZE,
                NULL,
                TASK_SRV_PRIO,
                NULL);
}

#define LEFT  0
#define RIGHT 1

void Task_steeringDemo(void* p)
{
	LINE l;
	double angle;
	int16_t linepos;

	motorSetDutyCycle(10);
	servoSetAngle(0);

	while(1)
    {
		l = lineGet();

		if (l.theta > 0)
		{
			linepos = -1 * l.d;
		}
		else
		{
			linepos = l.d;
		}

		angle = -1.0 * (linepos / 150.0) * (PI/3);

		servoSetAngle(angle);

		traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);

		vTaskDelay(100);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

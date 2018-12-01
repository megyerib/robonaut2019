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
#include "sch_ServoControlHandler.h"
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
    sch_Servo_Init();

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

	motorSetDutyCycle(10);
	sch_Set_Servo_Angle(0);

	while(1)
    {
		l = lineGet();

		angle = -1.0 * (l.d / 150.0) * (PI/3);

		sch_Set_Servo_Angle(angle);

		traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);

		vTaskDelay(100);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

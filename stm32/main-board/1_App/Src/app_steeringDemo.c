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
    servoInit(SRV_SRT_CH6012);

    lineInit();

    bspUartInit();

    xTaskCreate(Task_steeringDemo,
                "TASK_DEMO",
                DEFAULT_STACK_SIZE,
                NULL,
                TASK_SRV_PRIO,
                NULL);
}

#define LEFT  0
#define RIGHT 1

bool enab = true;
int8_t t = 0;

void Task_steeringDemo(void* p)
{
	LINE l;
	double angle;
	int16_t linepos;

	motorSetDutyCycle(10);
	steerSetAngle(3.1415/180 * 0);

	HAL_Delay(2000);

	steerSetAngle(3.1415/180 * 30);



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

		//steerSetAngle(angle);

		uint8_t p[6];
		if (l.d > 0)
		{
			p[0] = l.d % 100 + 0x30;
			p[1] = l.d % 10 + 0x30;
			p[2] = l.d + 0x30;
			p[3] = '\r';
			p[4] = '\n';
		}
		else
		{
			p[0] = '-';
			p[1] = l.d % 100 + 0x30;
			p[2] = l.d % 10 + 0x30;
			p[3] = l.d + 0x30;
			p[4] = '\r';
			p[5] = '\n';
		}

		if( enab == true )
		{
			t++;
			steerSetAngle(t * 3.14/180);

			if( t > 15 )
			{
				t = -15;
			}
		}

		//traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);
		bspUartTransmit_IT(Uart_USB, p, sizeof(p));

		vTaskDelay(100);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

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

static int printInt(int x, uint8_t* buf);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_steeringDemo(void)
{
    servoInit(SRV_SRT_CH6012);

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

bool enab = true;
int8_t t = 0;

LINE l;
double angle;
int16_t linepos;

uint8_t buf[10];
uint8_t cnt;

void Task_steeringDemo(void* p)
{
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

		steerSetAngle(angle);

		cnt = printInt(linepos, buf);

		/*if( enab == true )
		{
			t++;
			steerSetAngle(t * 3.14/180);

			if( t > 15 )
			{
				t = -15;
			}
		}*/

		//traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);
		bspUartTransmit_IT(Uart_USB, buf, cnt);

		vTaskDelay(1000);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static int printInt(int x, uint8_t* buf)
{
	int i = 0;

	if (x < 0)
	{
		buf[0] = '-';
		x *= -1;
		i++;
	}

	buf[i+0] = x / 100 + '0';
	buf[i+1] = (x % 100) / 10 + '0';
	buf[i+2] = (x %  10) + '0';
	buf[i+3] = '\r';
	buf[i+4] = '\n';

	return i + 5;
}

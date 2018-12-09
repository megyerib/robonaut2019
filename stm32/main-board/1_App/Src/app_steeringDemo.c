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

static double p_a;		// m
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

static cFirstOrderTF contrPD;

double prev_p;
static double Kd;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int printInt(int x, uint8_t* buf);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_steeringDemo(void)
{
    servoInit();

    lineInit();

    remoteInit();

	p_a   = 0;
	e 	  = 0;
	p 	  = -0.1;
	phi_a = 0;
	v 	  = 2.5;
	L 	  = 0.275;

	Kp = -4/(v*L);
	Td = 0.001; //0.0217;
	T  = 10*Td;

	contrPD.an_past = 0;
	contrPD.bn_past = 0;
	contrPD.a1 = T;
	contrPD.b0 = Kp;
	contrPD.b1 = Kp*Td;


	Kd = 10;

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
	servoSetAngle(0);

	/*for (int i = 95; i >= 88; i--)
	{
		bspServoSetCompare(i);
	}*/

	HAL_Delay(2000);

	steerSetAngle(3.1415/180 * 30);



	while(1)
    {
		// REMOTE CONTROL __________________________________

		if (remoteGetState())      // ENABLED
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
		else                       // DISABLED
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			vTaskDelay(STTERINGDEMO_TASK_DELAY);
			continue; // Skip the loop iteration
		}

		// TRACTION ________________________________________

		motorSetDutyCycle(10);
		traceBluetooth(BCM_LOG_ENC_VEL, 10);

		// STEERING ________________________________________

		/*l = lineGet();

		angle = -1.0 * (l.d / 150.0) * (PI/3);

		servoSetAngle(angle*2);

		traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);*/


		prev_p = p_meas;

		p_meas = lineGet().d;
		traceBluetooth(BCM_LOG_LINE_D, &p_meas);

		//e = p_a - p_meas;

		//phi_a = controllerTransferFunction(&contrPD, e);

		phi_a = -1 * p_meas / 120 * 0.75 + Kd * (prev_p - p_meas);

		servoSetAngle(phi_a);
		traceBluetooth(BCM_LOG_SERVO_ANGLE, &phi_a);

		// END DELAY _______________________________________

		vTaskDelay(STTERINGDEMO_TASK_DELAY);
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

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
#include "controller.h"

// Defines -------------------------------------------------------------------------------------------------------------

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

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_steeringDemo(void)
{
    sch_Servo_Init();

    lineInit();

	p_a   = 0;
	e 	  = 0;
	p 	  = -0.1;
	phi_a = 0;
	v 	  = 2.5;
	L 	  = 0.275;

	Kp = -4/(v*L);
//	Kd = 23;
	Td = 0.001; //0.0217;
	T  = 10*Td;

	contrPD.an_past = 0;
	contrPD.bn_past = 0;
	contrPD.a1 = T;
	contrPD.b0 = Kp;
	contrPD.b1 = Kp*Td;

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

	sch_Set_Servo_Angle(3.14/180*10);
	sch_Set_Servo_Angle(3.14/180*20);
	sch_Set_Servo_Angle(3.14/180*30);
	sch_Set_Servo_Angle(3.14/180*40);
	sch_Set_Servo_Angle(3.14/180*40);

	while(1)
    {
		// REMOTE CONTROL __________________________________

		// STEERING ________________________________________

		/*l = lineGet();

		angle = -1.0 * (l.d / 150.0) * (PI/3);

		sch_Set_Servo_Angle(angle*2);

		traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);*/

		p_meas = -lineGet().d;

		e = p_a - p_meas;

		//phi_a = controllerTransferFunction(&contrPD, e);

		phi_a = e * -5;

		sch_Set_Servo_Angle(3.14159265359/180 * phi_a *2);

		// END DELAY _______________________________________

		vTaskDelay(5);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

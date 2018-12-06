////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_linefollow.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include "app_linefollow.h"
#include "line.h"
#include "steer.h"
#include "trace.h"
#include "motor.h"
#include "controller.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

//static double Tsrv;		// sec
//static double Ksrv;		//

static double p_a;		// m
static double p_meas;	// m
static double e;		// m
static double p;		// m
static double phi_a;	// deg
static double v;		// m/s
static double L;		// m

static double Kp;		//
//static double Kd;		//
static double Td;		// sec
static double T;		// sec

static cFirstOrderTF contrPD;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void Task_LineFollow (void* p);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_LineFollow (void)
{
	//Tsrv = 0.007;
	//Ksrv = 60;

	p_a   = 0;
	e 	  = 0;
	p 	  = -0.1;
	phi_a = 0;
	v 	  = 2.5;
	L 	  = 0.275;

	Kp = -4/(v*L);
//	Kd = 23;
	Td = 0.0217;
	T  = 10*Td;

	contrPD.an_past = 0;
	contrPD.bn_past = 0;
	contrPD.a1 = T;
	contrPD.b0 = Kp;
	contrPD.b1 = Kp*Td;

    xTaskCreate(Task_LineFollow,
                "TASK_LINE_FOLLOW",
                DEFAULT_STACK_SIZE,
                NULL,
                TASK_LINE_FOLLOW_PRIO,
                NULL);
}

uint8_t rxBuf[3];

static void Task_LineFollow (void* p)
{
    (void)p;

    // TODO Constant duty cycle.
    uint8_t pwm = 20;

    while (1)
    {
    	// Get new measurements of the line p distance.
    	//p_meas = lineGet().d;
    	p_meas = -0.1;
    	traceBluetooth(BCM_LOG_LINE_D, &p_meas);

    	// PD controller: Calculate control error. Give the error to the controller and receive new control variable.
    	e = p_a - p_meas;
    	phi_a = controllerTransferFunction(&contrPD, e);

    	// Give the control variable to the actuator.
    	steerSetAngle(3.14159265359/180 * phi_a);
    	traceBluetooth(BCM_LOG_SERVO_ANGLE, &phi_a);

    	// TODO Give constant speed.
    	motorSetDutyCycle(pwm);
    	traceBluetooth(BCM_LOG_CTR_MTR_CURR, &pwm);

    	vTaskDelay(TASK_DELAY_5_MS);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

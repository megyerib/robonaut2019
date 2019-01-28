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

static cTraceRxBluetoothStruct recBtData;

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
    uint8_t pwm = 10;

    while (1)
    {
    	// TODO
    	recBtData = traceReceiveBluetooth();
    	/*if (recBtData.RecCmdPdKp_x == true)
    	{
        	controllerPdConfigure(&contrPD, recBtData.RecDataPdKp_d, Td, T);
    	}

    	if (recBtData.RecCmdPdTd == true)
    	{
        	controllerPdConfigure(&contrPD, Kp, recBtData.RecDataPdTd_d, recBtData.RecDataPdTd_d*10);
    	}*/

    	// Get new measurements of the line p distance.
    	p_meas = lineGetSingle();
    	traceBluetooth(BT_LOG_LINE_MAIN_LINE_POS, &p_meas);

    	// Calculate control error.
    	e = p_a - p_meas;

    	// Give the error to the controller and receive new
    	phi_a = controllerTransferFunction(&contrPD, e);

    	//TODO
    	// Give the control variable to the actuator.
    	steerSetAngle(3.14159265359/180 * phi_a);
    	/*if (recBtData.RecCmdSteer == true)
    	{
    		steerSetAngle(3.14159265359/180 * recBtData.RecDataSteer);
    	}
    	else
    	{
    		steerSetAngle(3.14159265359/180 * phi_a);
    	}*/
    	traceBluetooth(BT_LOG_SERVO_ANGLE, &phi_a);

    	motorSetDutyCycle(pwm);
    	/*if (recBtData.RecCmdAccelerate == true)
    	{
    		uint8_t duty = 0;
    		if (recBtData.RecDataAccelerate > 100)
    		{
    			duty = 100;
    		}
    		else if (recBtData.RecDataAccelerate < 0)
    		{
    			duty = 0;
    		}
    		else
    		{
    			duty = recBtData.RecDataAccelerate;
    		}

        	motorSetDutyCycle(duty);
    	}*/
    	//traceBluetooth(BCM_LOG_CTR_MTR_CURR, &pwm);

    	//TODO const to define
    	vTaskDelay(5);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_navigation.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../1_App/Inc/app_navigation.h"

#include "FreeRTOS.h"
#include "task.h"
#include "../../1_App/Inc/app_common.h"
#include "../../2_Handler/Inc/drn_DeadReckoningNavigation.h"
#include "../../2_Handler/Inc/inert.h"
#include "../../2_Handler/Inc/trace.h"
#include "../../3_BSP/Inc/bsp_servo.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------


//! @brief	Initializes the Task_Servo task.
void TaskInit_Navigation(void)
{
	semDrNavi = xSemaphoreCreateBinary();
	if(semDrNavi != NULL)
	{
		xSemaphoreGive(semDrNavi);
		drnInit();
		inertInit();

		inertTriggerMeasurement();
	}

	xTaskCreate(Task_Navigation,
				"TASK_NAVIGATION",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_NAVI_PRIO,
				NULL);
}


//! @brief	Communicates with the Task_Sharp task and moves the  servo.
void Task_Navigation(void* p)
{
	(void)p;

	Accel prev_a;
	Accel a;
	cVelocityVector v;

	AngVel w;
	cAngularVelocity w_drn;

	cNedParameters ned;

	a.a_x = 0;
	a.a_y = 0;
	a.a_z = 0;

//	UBaseType_t naviStackUsage;
//	naviStackUsage = uxTaskGetStackHighWaterMark(NULL);

	while(1)
	{
		prev_a = a;
		a = inertGetAccel();
		w = inertGetAngVel();

		w_drn.omega = w.omega_z;

		v.x = drnNumIntegTrapezoidal(0, TASK_DELAY_16_MS, prev_a.a_x, a.a_x);
		v.y = drnNumIntegTrapezoidal(0, TASK_DELAY_16_MS, prev_a.a_y, a.a_y);

		ned = drnReckonNavigation(v, w_drn, TASK_DELAY_16_MS);

		traceBluetooth(BCM_LOG_NAVI_N, &ned.n);
		traceBluetooth(BCM_LOG_NAVI_E, &ned.e);
		//traceBluetooth(BCM_LOG_NAVI_THETA, &);

		inertTriggerMeasurement();

		vTaskDelay(TASK_DELAY_16_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------


// --------------------------------------------------------------------------//

// TEST
// Init
/*
 const double a_m[200] =
		{
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
		};

const double w_m[200] =
		{
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,PI/2,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
		};
double u[200];
double th[200];
double x;
double n_m[200];
double e_m[200];
VelocityVector v[200];
AngularVelocity w[200];

u[0] = 0;
n_m[0] = 0;
e_m[0] = 0;
v[0].x = 0;
v[0].y = 0;
w[0].omega = 0;
w[0].time = 0;

int i = 0;
for(i = 1; i < 190; i++)
{
	//x = u[i-1] + ((double)BSP_DELAY_16_MS/1000) * (a_m[i-1] + a_m[i]) / 2;
	x = u[i-1] + drn_NumInteg_Trapezoidal(0, (double)BSP_DELAY_16_MS/1000, a_m[i-1], a_m[i]);
	u[i] = x;

	v[i].x = u[i];
	v[i].y = 0;
	w[i].omega = w_m[i];
	w[i].time = 0;

	th[i] = th[i-1] + drn_NumInteg_Trapezoidal(0, (double)BSP_DELAY_16_MS/1000, w_m[i-1], w_m[i]);
}

volatile VelocityVector tv1;
volatile AngularVelocity ta2;
tv1.x = 1.016;
tv1.y = 0;
ta2.omega = PI/2;
ta2.time = 0;
volatile NED_Parameters test0 =  drn_ReckonNavigation(tv1, ta2, 16);
test0 =  drn_ReckonNavigation(tv1, ta2, 16);
 */

// TASK
/*
	NED_Parameters nedCoordinates;

	uint32_t i = 1;

	nedCoordinates.n = 0;
	nedCoordinates.e = 0;

	while(1)
	{
		if( i < 192 )
		{
			nedCoordinates =  drn_ReckonNavigation(v[i], w[i], BSP_DELAY_16_MS);
			n_m[i] = nedCoordinates.n;
			e_m[i] = nedCoordinates.e;
		}

		//printf("A has been integrated to v");

		if(i > 192)
		{
			// Car stopped
			u[199] = 10 + u[198];
			printf("Finish");
		}
		else
		{
			i++;
		}

		vTaskDelay(BSP_DELAY_16_MS);
	}
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_navigation.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "navigation.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_navigation.h"
#include "app_common.h"

#include "inert.h"
#include "speed.h"
#include "trace.h"
#include "handler_common.h"

#include "bsp_servoTimer.h"


// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

static ACCEL    acceleration;
static ANGVEL   angularVelocity;
static cVEC_VEL velocityEnc;

static ACCEL  accelOffset;
static ANGVEL angularOffset;

static cVEC_ACCEL a;
static ANGVEL w;
static float v;
static cNAVI_STATE naviState;

static uint8_t naviMethod = 0;	// DEBUG. It can only be 0,1,2 or 3.

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

//! @brief	Initializes the Task_Navigation task.
void TaskInit_Navigation(void)
{
	inertTriggerMeasurement();

	semDrNavi = xSemaphoreCreateBinary();
	if(semDrNavi != NULL)
	{
		xSemaphoreGive(semDrNavi);
	}

	// Measurements.
	accelOffset   = inertGetAccel();		// g
	angularOffset = inertGetAngVel();		// dps

	xTaskCreate(Task_Navigation,
				"TASK_NAVIGATION",
				DEFAULT_STACK_SIZE+100,
				NULL,
				TASK_NAVI_PRIO,
				NULL);
}


//! @brief	Communicates with the Task_Sharp task and moves the  servo.
void Task_Navigation(void* p)
{
	(void)p;

	// Wait for the first measurement result.
	vTaskDelay(200);
	ANGVEL ofs = inertGetAngVel();
	inertGyroOffsetCalibration(ofs);

	while(1)
	{
		// Measurements.
		acceleration    = inertGetAccel();		// g
		angularVelocity = inertGetAngVel();		// dps
		// TODO
		//velocityEnc     = speedGet();			//

		//_____________________________________________________ CONVERSION _____________________________________________
		// Parallel with the orientation of the car.
		a.u = naviConvertGToSI(acceleration.a_x);
		// Orthogonal with the orientation of the car.
		a.v = naviConvertGToSI(acceleration.a_y);
		// TODO
		a.v = naviConvertGToSI(acceleration.a_z);

		// TODO
		w.omega_x = naviConvertDpsToSI(angularVelocity.omega_x);
		// TODO
		w.omega_y = naviConvertDpsToSI(angularVelocity.omega_y);
		// Yaw direction in the RPY coordinate-system.
		w.omega_z = naviConvertDpsToSI(angularVelocity.omega_z);

		//TODO
		//v = velocityEnc;
		//______________________________________________________________________________________________________________

		//________________________________________________ NAVIGATION STATE UPDATE _____________________________________
		// Calculate the actual position and orientation.
		if (naviMethod == 0)
		{
			// OWN ALGORITHM
			naviState = naviGetNaviDataInrt(a, w, TASK_DELAY_16_MS, eNAVI_INERT);
		}
		else if (naviMethod == 1)
		{
			// STM algorithm, gravitational correction
			naviState = naviGetNaviDataInrt(a, w, TASK_DELAY_16_MS, eNAVI_INERT_GRAVY_CORR);
		}
		else if (naviMethod == 2)
		{
			// OWN ALGORITHM
			naviState = naviGetNaviDataEnc(v, w, TASK_DELAY_16_MS, eNAVI_ENC);
		}
		else if (naviMethod == 3)
		{
			// STM algorithm, gravitational correction
			naviState = naviGetNaviDataEnc(v, w, TASK_DELAY_16_MS, eNAVI_ENC_GRAVY_CORR);
		}
		else
		{
			// NOP
		}
		//______________________________________________________________________________________________________________

		//___________________________________________________ TRACE ____________________________________________________
		traceBluetooth(BCM_LOG_NAVI_N, &naviState.p.n);
		traceBluetooth(BCM_LOG_NAVI_E, &naviState.p.e);
		traceBluetooth(BCM_LOG_NAVI_THETA, &naviState.phi);

		traceBluetooth(BCM_LOG_INERT_ACCEL_X, &acceleration.a_x);
		traceBluetooth(BCM_LOG_INERT_ACCEL_Y, &acceleration.a_y);
		traceBluetooth(BCM_LOG_INERT_ACCEL_Z, &acceleration.a_z);

		traceBluetooth(BCM_LOG_INERT_ANG_VEL_X, &angularVelocity.omega_x);
		traceBluetooth(BCM_LOG_INERT_ANG_VEL_Y, &angularVelocity.omega_y);
		traceBluetooth(BCM_LOG_INERT_ANG_VEL_Z, &angularVelocity.omega_z);
		//______________________________________________________________________________________________________________

		// Trigger the next conversion.
		inertTriggerMeasurement();

		vTaskDelay(TASK_DELAY_16_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

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
#include "trace.h"
#include "handler_common.h"

#include "bsp_servoTimer.h"


// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

static ACCEL  acceleration;
static ANGVEL angularVelocity;

static ACCEL  accelOffset;
static ANGVEL angularOffset;

static cVEC_ACCEL a;
static cNAVI_STATE naviState;

static bool naviMethod = true;

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

	while(1)
	{
		// Measurements.
		acceleration    = inertGetAccel();		// g
		angularVelocity = inertGetAngVel();		// dps

		// Parallel with the orientation of the car.
		a.u = naviConvertGToSI(acceleration.a_x);
		// Orthogonal with the orientation of the car.
		a.v = naviConvertGToSI(acceleration.a_y);

		if (naviMethod == false)
		{
			// OWN ALGORITHM_______________________________________________________________
			// Yaw direction in the RPY coordinate-system.
			angularVelocity.omega_z = naviConvertDpsToSI(angularVelocity.omega_z);

			// Calculate the actual position and orientation.
			naviState = naviDRProcessInertial(a, angularVelocity.omega_z, TASK_DELAY_16_MS);
			//______________________________________________________________________________
		}
		else
		{
			// STM algorithm, gravitational correction _____________________________________
			angularVelocity.omega_x = naviConvertDpsToSI(angularVelocity.omega_x);
			angularVelocity.omega_y = naviConvertDpsToSI(angularVelocity.omega_y);
			angularVelocity.omega_z = naviConvertDpsToSI(angularVelocity.omega_z);

			// Calculate the actual position and orientation.
			naviState = naviDRProcessInertialSTM32(a, angularVelocity, TASK_DELAY_16_MS);
			//______________________________________________________________________________
		}


		//traceBluetooth(BCM_LOG_NAVI_N, &naviState.p.n);
		//traceBluetooth(BCM_LOG_NAVI_E, &naviState.p.e);
		//traceBluetooth(BCM_LOG_NAVI_THETA, &naviState.phi);

		inertTriggerMeasurement();

		vTaskDelay(TASK_DELAY_16_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

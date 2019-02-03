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

#define NAVI_TASK_DELAY		5

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

static ACCEL    acceleration;
static ANGVEL   angularVelocity;

static ACCEL  accelOffset;
static ANGVEL angularOffset;

static cVEC_ACCEL a;
static ANGVEL w;
static float v;
static cNAVI_STATE naviState;

static uint8_t naviMethod = 3;	// DEBUG. It can only be 0,1,2 or 3.

static ACCEL txAccel;
static ANGVEL txAngVel;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void naviInertOffsetCalibration (void);

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
	vTaskDelay(300);

	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();
	naviInertOffsetCalibration();

	while(1)
	{
		// Measurements.
		acceleration    = inertGetAccel();		// g
		angularVelocity = inertGetAngVel();		// dps
		// Velocity of the car (parallel with the movement) from the encoder.
		v = speedGet();							// m/s

		if (v == 0.0f)
		{
			//inertGyroOffsetCalibration(inertGetAngVel());
		}

		//_____________________________________________________ CONVERSION _____________________________________________
		// Parallel with the orientation of the car.
		a.u = naviConvertGToSI(acceleration.a_x);
		// Orthogonal with the orientation of the car.
		a.v = naviConvertGToSI(acceleration.a_y);
		// Vertical acceleration.
		//a.v = naviConvertGToSI(acceleration.a_z);

		// Roll direction in the RPY coordinate-system.
		w.omega_x = naviConvertDpsToSI(angularVelocity.omega_x);
		// Pitch direction in the RPY coordinate-system.
		w.omega_y = naviConvertDpsToSI(angularVelocity.omega_y);
		// Yaw direction in the RPY coordinate-system.
		w.omega_z = naviConvertDpsToSI(angularVelocity.omega_z);
		//______________________________________________________________________________________________________________

		//________________________________________________ NAVIGATION STATE UPDATE _____________________________________
		// Calculate the actual position and orientation.
		if (naviMethod == 0)
		{
			// OWN ALGORITHM
			naviState = naviGetNaviDataInrt(a, w, NAVI_TASK_DELAY, eNAVI_INERT);
		}
		else if (naviMethod == 1)
		{
			// STM algorithm, gravitational correction
			naviState = naviGetNaviDataInrt(a, w, NAVI_TASK_DELAY, eNAVI_INERT_GRAVY_CORR);
		}
		else if (naviMethod == 2)
		{
			// OWN ALGORITHM
			naviState = naviGetNaviDataEnc(v, w, NAVI_TASK_DELAY, eNAVI_ENC);
		}
		else if (naviMethod == 3)
		{
			// STM algorithm, gravitational correction
			naviState = naviGetNaviDataEnc(v, w, NAVI_TASK_DELAY, eNAVI_ENC_GRAVY_CORR);
		}
		else
		{
			// NOP
		}
		//______________________________________________________________________________________________________________

		//___________________________________________________ TRACE ____________________________________________________
		float psi = naviNormaliseOrientation(naviState.psi);

		traceBluetooth(BT_LOG_NAVI_N, &naviState.p.n);
		traceBluetooth(BT_LOG_NAVI_E, &naviState.p.e);
		traceBluetooth(BT_LOG_NAVI_PSI, &psi);

		traceBluetooth(BT_LOG_ENC_V, &v);

		txAccel.a_x = naviConvertGToSI(acceleration.a_x);
		txAccel.a_y = naviConvertGToSI(acceleration.a_y);
		txAccel.a_z = naviConvertGToSI(acceleration.a_z);

		txAngVel.omega_x = naviConvertDpsToSI(angularVelocity.omega_x);
		txAngVel.omega_y = naviConvertDpsToSI(angularVelocity.omega_y);
		txAngVel.omega_z = naviConvertDpsToSI(angularVelocity.omega_z);

		traceBluetooth(BT_LOG_INERT_ACCEL_X, &txAccel.a_x);
		traceBluetooth(BT_LOG_INERT_ACCEL_Y, &txAccel.a_y);
		traceBluetooth(BT_LOG_INERT_ACCEL_Z, &txAccel.a_z);

		traceBluetooth(BT_LOG_INERT_ANG_VEL_X, &txAngVel.omega_x);
		traceBluetooth(BT_LOG_INERT_ANG_VEL_Y, &txAngVel.omega_y);
		traceBluetooth(BT_LOG_INERT_ANG_VEL_Z, &txAngVel.omega_z);
		//______________________________________________________________________________________________________________

		// Trigger the next conversion.
		inertTriggerMeasurement();

		vTaskDelay(NAVI_TASK_DELAY);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void naviInertOffsetCalibration (void)
{
	uint8_t wait = 5;
	ANGVEL ofs[10];
	ANGVEL avg;
	ANGVEL sum;
	int i;
	sum.omega_x = 0;
	sum.omega_y = 0;
	sum.omega_z = 0;

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[0] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[1] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[2] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[3] = inertGetAngVel();

	vTaskDelay(wait);
	ofs[4] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[5] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[6] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[7] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[8] = inertGetAngVel();

	inertTriggerMeasurement();
	vTaskDelay(wait);
	ofs[9] = inertGetAngVel();

	for (i = 0; i < 10; i++)
	{
		sum.omega_x += ofs[i].omega_x;
		sum.omega_y += ofs[i].omega_y;
		sum.omega_z += ofs[i].omega_z;
	}

	avg.omega_x = sum.omega_x / 10.0f;
	avg.omega_y = sum.omega_y / 10.0f;
	avg.omega_z = sum.omega_z / 10.0f;

	inertGyroOffsetCalibration(avg);
}

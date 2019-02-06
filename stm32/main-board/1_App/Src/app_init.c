////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_init.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <app_encoderCalibration.h>
#include <app_servoCalibration.h>
#include "bsp_common.h"
#include "handler_common.h"

// App layer headers
#include "app_steeringDemo.h"
#include "app_sharp.h"
#include "app_navigation.h"
#include "app_cdt.h"
#include "app_inertCalibration.h"
#include "app_maze.h"
#include "app_speedRun.h"
#include "app_roadsignal.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void appInit();

// Global function definitions /////////////////////////////////////////////////////////////////////////////////////////

void Init()
{
	// Wait for the PSU init
	HAL_Delay(1000);

	bspInit();
	hndlInit();
	appInit();
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//! Initialize app layer modules
static void appInit()
{
	//______________________Race:
	/*TaskInit_CarDiagnosticsTool();
	TaskInit_Navigation();
	TaskInit_Sharp();

	TaskInit_Maze();
	TaskInit_SpeedRun();*/
	TaskInit_roadSignal();

	//_______________________Calibration:
	//TaskInit_InertialCalibration();
	//TaskInit_SpeedCalibration();


	//_______________________Debug:
	//TaskInit_steeringDemo();

	//TaskInit_Servo();
}

// END /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_init.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

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
#include "app_speedcal.h"

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
	TaskInit_CarDiagnosticsTool();
	TaskInit_Navigation();

	TaskInit_Maze();
	TaskInit_SpeedRun();

	//_______________________Calibration:
	//TaskInit_InertialCalibration();
	//TaskInit_SpeedCalibration();


	//_______________________Debug:
	//TaskInit_steeringDemo();
	//TaskInit_Sharp();
}

// END /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

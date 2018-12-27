////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_init.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

#include "bsp_common.h"

#include "app_cdt.h"
#include "app_steeringDemo.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions /////////////////////////////////////////////////////////////////////////////////////////

// Local (static) function definitions ---------------------------------------------------------------------------------

void Init()
{
	// Wait for the PSU init
	HAL_Delay(1000);

	bspInit();

	//DEBUG
    servoInit();

 //   lineInit();
    //

	TaskInit_Sharp();
//	TaskInit_Servo();
	//TaskInit_SControl();
	//TaskInit_steeringDemo();
	TaskInit_CarDiagnosticsTool();
}

// END /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

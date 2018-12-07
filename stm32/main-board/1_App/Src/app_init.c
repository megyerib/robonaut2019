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
#include "app_linefollow.h"
#include "app_servo.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions /////////////////////////////////////////////////////////////////////////////////////////

// Local (static) function definitions ---------------------------------------------------------------------------------

void Init()
{
	// Wait for the PSU init
	//TODO ez itt jó? a perifériák már inicializálva vannak, nem veszélyes az?
	HAL_Delay(2000);

	bspInit();

	// Must-Have
	TaskInit_CarDiagnosticsTool();


	// TODO DEBUG	WARNING: Don't enable all of them at the same time!
	//TaskInit_Servo();
	TaskInit_LineFollow();
	//TaskInit_steeringDemo();
	// END_DEBUG
}

// END /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_application.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------
#include "app_applicaton.h"
#include "app_commands.h"
#include "app_current_controller.h"
#include "app_init.h"
#include "app_measurements_evaluation.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void APP_TestApplication()
{
	while(1)
	{
		//APP_MessageIdentificationForTesting(ReceiveBufferUART, &ConvertedDutyCycle);
		//APP_SetDutyCycle(&ConvertedDutyCycle);	//for testing the Transistors
		//APP_SetLEDDutyClyeForTesting(&ConvertedDutyCycle);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END -----------------------------------------------------------------------------------------------------------------
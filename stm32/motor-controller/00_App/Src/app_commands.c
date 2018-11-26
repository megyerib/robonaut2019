////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_commands.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f0xx_hal.h"
#include "app_commands.h"
#include "bsp_communication.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------
unsigned char DutyCycleFromUART[24];
// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

// Local (static) function definitions ---------------------------------------------------------------------------------

void APP_MessageIdentificationForTesting(unsigned char* DutyCycleFromUART, float* ConvertedDutyCycle)
{
	int i = 0;
	int32_t Converted = 0;
	while(DutyCycleFromUART[i] != '0')
	{
		Converted *= 10;
		Converted+= DutyCycleFromUART[i] - '0';
		*ConvertedDutyCycle = Converted + 0.0;
		*ConvertedDutyCycle /= 100;
	}

}

// END -----------------------------------------------------------------------------------------------------------------
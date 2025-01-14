////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      dist.c
//!  \brief		Interface to distance sensors.
//!  \details	See in the header file...
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <bsp_sharp.h>
#include "dist.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void distInit()
{
	// Init I2C

	return;
}

uint16_t distGet(const eDIRECTION dir)
{
	uint16_t retVal = 0;

	switch (dir)
	{
		case Front:
		{
			break;
		}
		case RightFront:
		{
			break;
		}
		case RightMiddle:
		{
			break;
		}
		case Rear:
		{
			retVal = sharpGetMeasurement().Distance;
			break;
		}
		default:
		{
			// Error: Invalid direction selected.
			break;
		}
	}

	return retVal;
}


// Local (static) function definitions ---------------------------------------------------------------------------------

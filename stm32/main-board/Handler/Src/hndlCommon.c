////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      hndlCommon.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "hndlCommon.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

uint32_t hndlGetNumberLength (const uint32_t number)
{
	uint32_t lenght = 0;
	uint32_t temp = (uint32_t)number;

	if (temp == 0)
	{
		lenght = 1;
	}
	else
	{
		while (temp > 0)
		{
			lenght++;
			temp /= 10;
		}
	}

	return lenght;
}

bool hndlConvertUintToUintArray (uint8_t* const array, const uint32_t value, const uint32_t len)
{
	bool success = false;
	uint32_t i = 0;
	uint32_t temp;

	if (array != NULL)
	{
		temp = value;
		for(i = len; i > 0; i--)
		{
			array[i-1] = temp % 10;
			temp /= 10;
		}

		success = true;
	}

	return success;
}

void hndlPlaceIntegerToAsciiMsg (uint8_t* const array, const uint32_t value, const uint32_t frameSize)
{
	uint32_t i = 0;
	uint32_t length;
	uint32_t offset;

	//TODO handle saturation
	length = hndlGetNumberLength(value);

	offset = frameSize - length;

	hndlConvertUintToUintArray(array+offset, value, length);

	// Convert to ASCII characters
	for (i = 0; i < frameSize; i++)
	{
		array[i] += 0x30;
	}
}

void hndlPlaceFractionToAsciiMsg (
										uint8_t* const array,
										const double   value,
										const uint32_t frameSize,
										const uint32_t decimaldigits
								 )
{
	uint32_t i = 0;
	double temp = value;
	uint32_t convertedValue;

	// Convert the double value to a integer
	for (i = 0; i < decimaldigits; i++)
	{
		temp *= 10;
	}
	convertedValue = (uint32_t)temp;

	hndlPlaceIntegerToAsciiMsg(array, convertedValue, frameSize);
}

// Local (static) function definitions ---------------------------------------------------------------------------------


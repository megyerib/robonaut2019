////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      hndlCommon.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../2_Handler/Inc/hndlCommon.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

uint32_t hndlGetNumberLength (const uint32_t number)
{
	uint32_t lenght = 0;
	uint32_t temp = number;

	// If the number was 0, then its length is 1.
	if (temp == 0)
	{
		lenght = 1;
	}
	else
	{
		// The absolute value of the number is > 0.

		// Count the digits/length.
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

	// Check if the pointer is NULL.
	if (array != NULL)
	{
		// Pointer is not NULL.

		// Fill up the array with the digits of the value form the back.
		temp = value;
		for(i = len; i > 0; i--)
		{
			array[i-1] = temp % 10;
			temp /= 10;
		}

		// Success flag is set.
		success = true;
	}

	return success;
}

void hndlPlaceIntegerToAsciiMsg (uint8_t* const array, const uint32_t value, const uint32_t frameSize, const bool neg)
{
	uint32_t i = 0;
	uint32_t length;
	uint32_t offset;

	// Count the digits.
	length = hndlGetNumberLength(value);

	// Check if the value is negative.
	if(neg)
	{
		// The first element of the buffer will be "-" so the frame size is less for the aligned data.
		offset = frameSize - 1 - length;
	}
	else
	{
		// Align the data to the MSB.
		offset = frameSize - length;
	}

	// Convert the vale to an array with the offset.
	hndlConvertUintToUintArray(array+offset, value, length);

	// Convert the array to ASCII characters. Before the offset all will be '0'.
	for (i = 1; i < frameSize; i++)
	{
		array[i] += 0x30;
	}

	// Place "-" in the first place or convert the number to ASCII.
	if(neg)
	{
		array[0] = '-';
	}
	else
	{
		array[0] += 0x30;
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
	bool isNegative = false;

	// Convert the double value to a integer.
	for (i = 0; i < decimaldigits; i++)
	{
		temp *= 10;
	}

	// If it is a negative number take its absolute value and sign it negative.
	if(temp < 0)
	{
		temp *= -1;
		isNegative = true;
	}

	// Cast the double to unsigned int.
	convertedValue = (uint32_t)temp;

	// Call the Integer placer function with absolute value and negative indication flag.
	hndlPlaceIntegerToAsciiMsg(array, convertedValue, frameSize, isNegative);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

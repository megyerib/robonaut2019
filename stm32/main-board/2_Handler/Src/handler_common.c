////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      hndlCommon.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "handler_common.h"

#include "button.h"
#include "dist.h"
#include "inert.h"
#include "line.h"
#include "motor.h"
#include "navigation.h"
#include "remote.h"
#include "speed.h"
#include "start.h"
#include "steer.h"
#include "trace.h"


// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------
// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void hndlInit()
{
	buttonInit();
	distInit();
	inertInit();
	lineInit();
	motorInit();
	naviDRInit();
	remoteInit();
	speedInit();
	startInit();
	steerInit();
	traceInit();
}

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
										const float    value,
										const uint32_t frameSize,
										const uint32_t decimaldigits
								 )
{
	uint32_t i = 0;
	float temp = value;
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

float hndlNumIntegTrapezoidal (const float a, const float b, const float fa, const float fb)
{
	float integral;

	integral = (b - a) * (fb + fa) / 2;

	return integral;
}

cMATRIX_3X3 hndlMatrixInversion (const cMATRIX_3X3 A)
{
	cMATRIX_3X3 invA;
	cMATRIX_3X3 adjA;
	float detA;

	// det(A)
	detA =    A.a1[0] * (A.a2[1]*A.a3[2] - A.a2[2]*A.a3[1])
			- A.a2[0] * (A.a1[1]*A.a3[2] - A.a1[2]*A.a3[1])
			+ A.a3[0] * (A.a1[1]*A.a2[2] - A.a1[2]*A.a2[1]);

	// Adj A
	adjA.a1[0] = A.a2[1]*A.a3[2] - A.a2[2]*A.a3[1];
	adjA.a1[1] = A.a2[2]*A.a3[0] - A.a2[0]*A.a3[2];
	adjA.a1[2] = A.a2[0]*A.a3[1] - A.a2[1]*A.a3[0];
	adjA.a2[0] = A.a1[2]*A.a3[1] - A.a1[1]*A.a3[2];
	adjA.a2[1] = A.a1[0]*A.a3[2] - A.a1[2]*A.a3[0];
	adjA.a2[2] = A.a1[1]*A.a3[0] - A.a1[0]*A.a3[1];
	adjA.a3[0] = A.a1[1]*A.a2[2] - A.a1[2]*A.a2[1];
	adjA.a3[1] = A.a1[2]*A.a2[0] - A.a2[2]*A.a1[0];
	adjA.a3[2] = A.a1[0]*A.a2[1] - A.a1[1]*A.a2[0];

	// 1/det A *adj A
	invA.a1[0] = 1.0 / detA * adjA.a1[0];
	invA.a1[1] = 1.0 / detA * adjA.a1[1];
	invA.a1[2] = 1.0 / detA * adjA.a1[2];
	invA.a2[0] = 1.0 / detA * adjA.a2[0];
	invA.a2[1] = 1.0 / detA * adjA.a2[1];
	invA.a2[2] = 1.0 / detA * adjA.a2[2];
	invA.a3[0] = 1.0 / detA * adjA.a3[0];
	invA.a3[1] = 1.0 / detA * adjA.a3[1];
	invA.a3[2] = 1.0 / detA * adjA.a3[2];

	return invA;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

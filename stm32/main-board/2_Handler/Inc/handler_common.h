////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      hndlCommon.h
//!  \brief    
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	uint8_t  Sequence;
	uint32_t Distance;
} cMEASUREMENT_DIST;

typedef struct
{
	float a1[3];
	float a2[3];
	float a3[3];
} cMATRIX_3X3;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

//! Initialize handler layer
void hndlInit();

//! THis function determines how many digit the given value contains.
//! @param number
//! @return
uint32_t hndlGetNumberLength (const uint32_t number);

//! Converts a value to a uint8_t array that will contain the value in BCD.
//!
//! @param array			holds the characters of the value
//! @param value			value wanted to be converted
//! @param len				indicates how many digits of the value must be converted
//! @return					true indicates successful conversion, false upon failure
bool hndlConvertUintToUintArray (uint8_t* const array, const uint32_t value, const uint32_t len);

//!	Places an integer number into a uint8_t array in BCD from.
//!
//! @param array			will contain the BCD characters
//! @param value			value needs to be stored
//! @param frameSize		how many characters can be saved
//! @param neg				true indicates if the number is negative, then the first character is an '-'
void hndlPlaceIntegerToAsciiMsg (uint8_t* const array, const uint32_t value, const uint32_t frameSize, const bool neg);

//! Places a double number into a uint8_t array in BCD from.
//!
//! @param array			will contain the BCD characters
//! @param value			value needs to be stored
//! @param frameSize		how many characters can be saved
//! @param decimaldigits	how many decimals (precision) must be saved
void hndlPlaceFractionToAsciiMsg (
										uint8_t* const array,
										const float    value,
										const uint32_t frameSize,
										const uint32_t decimaldigits
									);

float hndlNumIntegTrapezoidal (const float a, const float b, const float fa, const float fb);

cMATRIX_3X3 hndlMatrixInversion (const cMATRIX_3X3 A);




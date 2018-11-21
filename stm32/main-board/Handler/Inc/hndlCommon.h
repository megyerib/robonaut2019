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
// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

uint32_t hndlGetNumberLength (const uint32_t number);

bool hndlConvertUintToUintArray (uint8_t* const array, const uint32_t value, const uint32_t len);

void hndlPlaceIntegerToAsciiMsg (uint8_t* const array, const uint32_t value, const uint32_t frameSize);

void hndlPlaceFractionToAsciiMsg (
										uint8_t* const array,
										const double   value,
										const uint32_t frameSize,
										const uint32_t decimaldigits
									);





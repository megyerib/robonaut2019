////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      mux.h
//!  \brief     Aanlog multiplexer functions
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f0xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void initMux();
void setMux(uint8_t input);

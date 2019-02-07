////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      motor.h
//!  \brief     Motor control functions
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void motorInit();
void motorSetTorque(int16_t torqe);
void motorSetDutyCycle(float d);

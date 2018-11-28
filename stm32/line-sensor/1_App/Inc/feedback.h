////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      feedback.h
//!  \brief     Let the world know about line position.
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f0xx_hal.h"

#include "../../1_App/Inc/eval.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void ledFeedback(LINE_SENSOR_OUT* line);

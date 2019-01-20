////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		sharp.h
//!  \brief
//!  \details  	This module handles the sharp distance measurements.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "handler_common.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "adc.h"


// Defines -------------------------------------------------------------------------------------------------------------

#define  	BSP_SHARP_HADC					hadc1
#define 	BSP_SHARP_ADC_CH 				4
#define 	BSP_SHARP_ADC_RESOLUTION 	    4096

// Typedefs ------------------------------------------------------------------------------------------------------------
// Variables -----------------------------------------------------------------------------------------------------------

//! @brief	Semaphore to the module's sds_distance private variable.
SemaphoreHandle_t semSharp;

// Function prototypes -------------------------------------------------------------------------------------------------

void sharpInit ();

//! @brief  Starts the ADC conversion and will generate an IT when it is ready.
void sharpTriggerAdc ();

//! @brief  Gets the calculated distance of the SHARP sensor safely.
//! @retval Distance.
cMEASUREMENT_DIST sharpGetMeasurement ();

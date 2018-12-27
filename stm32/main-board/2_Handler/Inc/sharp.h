////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		sharp.h
//!  \brief
//!  \details  	This module handles the sharp distance measurements.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "semphr.h"

#include "adc.h"

#include "hndlCommon.h"

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

//! @brief  Gets the calculated distance of the SHARP sensor safely.
//! @retval Distance.
uint16_t sharpGetDistance ();


//! @brief  Sets the calculated distance of the SHARP sensor safely
//! @param  Distance to be stored.
void sharpSetDistance(const uint16_t distance);


//! @brief  Starts the ADC conversion and will generate an IT when it is ready.
void sharpTriggerAdc ();

cMEASUREMENT_DIST sharpGetMeasurement ();

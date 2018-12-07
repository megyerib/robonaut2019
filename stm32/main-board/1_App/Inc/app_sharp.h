////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_servo.h
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "event_groups.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Variables -----------------------------------------------------------------------------------------------------------

extern EventGroupHandle_t event_sharp;

// Function prototypes -------------------------------------------------------------------------------------------------

//! @brief  Initializes Task_Sharp task. It creates semaphore for the sds module and calls the BSP_Sharp_ADC_Init()
//! 		function.
void TaskInit_Sharp (void);


//! @brief	Task function that periodically updates the the measured distance value of the sds module.
void Task_Sharp (void* p);

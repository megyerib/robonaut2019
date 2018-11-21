////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_common.h
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define 	BSP_3V3							3.3

#define 	PI								3.14159265359

// Typedefs ------------------------------------------------------------------------------------------------------------

//! @brief Status return enum for BSP functions.
typedef enum
{
	BSP_OK		= 0,
	BSP_ERROR
} eBspStatus;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------



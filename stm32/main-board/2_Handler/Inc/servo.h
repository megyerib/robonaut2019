////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		servo.h
//!  \brief
//!  \details  	This module controls the rotation of the servo.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_servo.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------


//! @brief	Initializes the module and the servo.
//! @retval	Result of the initialization (OK, FAIL).
eBSP_SrvInitStat servoInit(void);

//! @brief	Gets the servo position in radian.
//! @retval	Servo angle.
double servoGetAngle(void);

//! @brief	Rotates the servo to a given angle.
//! @param	_theta_ : The desired servo angle.
void servoSetAngle(const double theta);



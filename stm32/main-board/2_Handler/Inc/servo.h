////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		servo.h
//!  \brief
//!  \details  	This module controls the rotation of the servo.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_servoTimer.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! @brief	Available types of servos.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
	SRV_FUTABAS3003 		= 0,		// Joci's servo.		Analog
	SRV_MAVERICK_MS22,					// Car' basic servo.	Analog
	SRV_SRT_CH6012						// From model shop.		Digital
} eServoModel;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

//TODO
//! @brief	Initializes the module and the servo.
//! @retval	Result of the initialization (OK, FAIL).
eBSP_SrvTimInitStat servoInit(eServoModel myServoModel);

//! @brief	Gets the servo position in radian.
//! @retval	Servo angle.
double servoGetAngle(void);

//! @brief	Rotates the servo to a given angle.
//! @param	_theta_ : The desired servo angle.
void servoSetAngle(const double theta);

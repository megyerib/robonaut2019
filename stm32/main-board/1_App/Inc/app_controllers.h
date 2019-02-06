////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_controllers.h
//!  \brief     This module contains all of the available controllers.
//!  \details	The module encapsulates three types of controllers: line follow, speed keeper and distance keeper.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------
// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//!	PD controller that calculates a new servo angel in [rad] with which the user can follow the line.
//!
//! @param actLine		The position of the line detected by the line sensor in [mm].
//! @param prevLine		The position of the line in the previous task run in [mm].
//! @param P			The position into that the controller has to set in.
//! @param kP			The proportional constant of the controller.
//! @param kD			The derivative part of the controller.
//!
//! @return	A servo position in [rad].
//**********************************************************************************************************************
float cntrlLineFollow (const float actLine,
					   const float prevLine,
					   const float P,
					   const float kP,
					   const float kD);

//**********************************************************************************************************************
//!	PI controller that calculates a duty cycle in [%] to keep a given speed given in [m/s].
//!
//! @param r_speed		The setpoint (desired speed) of the controller.
//! @param prevSpeed	The speed of the robot that was measured in the previous task run.
//! @param actSpeed		The actual speed of the robot.
//! @param Ti			The integration tie of the controller.
//! @param fk			FOXBORO feedback constant.
//! @param kc			Gain.
//!
//! @return A duty cycle given in [%].
//**********************************************************************************************************************
uint32_t cntrSpeed (const float r_speed,
					const float prevSpeed,
					const float actSpeed,
					const float Ti,
					float* fk,
					const float kc);

//**********************************************************************************************************************
//! PI controller that calculates a TODO
//!
//! @return -
//**********************************************************************************************************************
float cntrDistance (const uint32_t followDist,
					   const float prevSpeed,
					   const uint32_t actDist,
					   const float kP,
					   const uint32_t speedMin,
					   const uint32_t speedMax);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file     app_speedRun.h
//!  \brief    Contains the logic, how to drive on the speed run path.
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

//! Determines how many segments a lap has.
#define	SRUN_LAP_SEGMENTS				16

// Typedefs ------------------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//! State of the main state machine of the Speed Run task.
//**********************************************************************************************************************
typedef enum
{
	eSTATE_MAIN_READY       = 0,	//!< The car is waiting behind the safety car to start.
	eSTATE_MAIN_PARADE_LAP,			//!< The car is following the safety car  in this lap.
	eSTATE_MAIN_OVERTAKING,			//!< The car is overtaking the safety car.
	eSTATE_MAIN_LAP_1,				//!< First lap: The safest algorithm and slowest lap (secure race points).
	eSTATE_MAIN_LAP_2,				//!< Second lap: Lap with moderate speed.
	eSTATE_MAIN_LAP_3,				//!< Third lap: The fastest lap.
	eSTATE_MAIN_STOP				//!< The car has completed 3 laps and stops.
} eSTATE_MAIN;

//**********************************************************************************************************************
//! States of the overtaking maneuver.
//**********************************************************************************************************************
typedef enum
{
	eSTATE_OVERTAKE_LEAVE_LINE = 0,			//!< Turn left and leave the race track.
	eSTATE_OVERTAKE_GET_PARALLEL,			//!< Direct back the car's direction to be parallel with the line.
	eSTATE_OVERTAKE_PASS_SAFETY_CAR,		//!< Speed up to pass the safety car.
	eSTATE_OVERTAKE_FIND_LINE,				//!< Turn right to find back to the race track.
	eSTATE_OVERTAKE_SUCCESS,				//!< The overtake was successful, the car is in front of the safety car.
	eSTATE_OVERTAKE_FAILED					//!< The car got lost and stop. Manual reposition is needed.
} eSTATE_OVERTAKE;

//**********************************************************************************************************************
//!	Contains all of the control parameters.
//**********************************************************************************************************************
typedef struct
{
	cPD_CONTROLLER_PARAMS lapParade;				//!< Warm up lap, follow the safety car. Chance to overtake.
	cPD_CONTROLLER_PARAMS overtaking;				//!< Overtake the safety car.
	cPD_CONTROLLER_PARAMS lap1[SRUN_LAP_SEGMENTS];	//!< First lap, safest run.
	cPD_CONTROLLER_PARAMS lap2[SRUN_LAP_SEGMENTS];  //!< Second lap, moderate run.
	cPD_CONTROLLER_PARAMS lap3[SRUN_LAP_SEGMENTS];  //!< Third lap, fastest run.
} cSRUN_PD_CONTROL_PARAM_LIST;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

//! @brief
void TaskInit_SpeedRun (void);


//! @brief
void Task_SpeedRun (void* p);

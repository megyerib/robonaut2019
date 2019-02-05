////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_speedRun_StateMachines.h
//!  \brief     This is a submodule that hold the state machines and the controller of the speed run app.
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------
// Defines -------------------------------------------------------------------------------------------------------------

//! Determines how many segments a lap has.
#define	SRUN_LAP_SEGMENTS				16

#define SRUN_FOLLOW_DISTANCE			40			//!< cm TODO determine

#define SRUN_START_GATE_DISTANCE		15			//!< cm TODO measure

#define SRUN_OVERTAKE_SEGMENT			8
#define SRUN_OVERTAKE_TURN_TIME 		400 		//!< 400 * task period (5ms) = 2s
#define SRUN_OVERTAKE_PASS_TIME			2000		//!< 2000* 5ms = 10s
#define SRUN_OVERTAKE_FIND_TIME			800			//!< 800 * 5ms = 4s
#define SRUN_OVERTAKE_SPEED_SLOW		15			//!< %
#define SRUN_OVERTAKE_SPEED_FAST		35			//!< %
#define SRUN_OVERTAKE_SERVO_ANGLE		20*PI/180	//!< rad

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
	cPD_CNTRL_PARAMS lapParade;				//!< Warm up lap, follow the safety car. Chance to overtake.
	cPD_CNTRL_PARAMS overtaking;				//!< Overtake the safety car.
	cPD_CNTRL_PARAMS lap1[SRUN_LAP_SEGMENTS];	//!< First lap, safest run.
	cPD_CNTRL_PARAMS lap2[SRUN_LAP_SEGMENTS];  //!< Second lap, moderate run.
	cPD_CNTRL_PARAMS lap3[SRUN_LAP_SEGMENTS];  //!< Third lap, fastest run.
} cSRUN_PD_CONTROL_PARAM_LIST;

typedef enum
{
	eSEG_STRAIGHT = 0,
	eSEG_CORNER,
	eSEG_SPEED_UP,
	eSEG_SLOW_DOWN,
	eSEG_SLOW_OR_SPEED,
	eSEG_LOST_TRACK
} eSEGMENT_TYPE;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//! Initializes all of the state machines.
//!
//! @return -
//**********************************************************************************************************************
void sRunInitStateMachines (void);

//**********************************************************************************************************************
//! State machine that manages the lap and holds the driving strategies.
//!
//! @return -
//**********************************************************************************************************************
void sRunMainStateMachine (void);

//**********************************************************************************************************************
//! This state machine is responsible for how the car act at the specific segments of the lap.
//!
//! @return -
//**********************************************************************************************************************
bool sRunDriveStateMachine (void);

//**********************************************************************************************************************
//! State machine for the overtaking maneuver.
//!
//! TODO parameters
//!
//! @return -
//**********************************************************************************************************************
void sRunOvertakeStateMachine (void);

//**********************************************************************************************************************
//! TODO
//!
//! @return -
//**********************************************************************************************************************
void sRunParadeLapAlgorithm	(void);

//**********************************************************************************************************************
//!	Line follow PD controller.
//!
//! @return -
//**********************************************************************************************************************
void sRunCntrLineFollow (void);

void SRunSM_Set_TryToOvertake (const bool value);
bool SRunSM_Get_TryToOvertake (void);

void SRunSM_Set_smMainStateS (const eSTATE_MAIN value);
bool SRunSM_Get_smMainState (void);

void SRunSM_Set_actLapSegment (const uint8_t value);
uint8_t SRun_Get_ActLapSegment (void);

void SRunSM_Set_actualParams (const cPD_CNTRL_PARAMS value);
cPD_CNTRL_PARAMS SRunSM_Get_actualParams (void);

void SRunSM_Set_paramList (const cSRUN_PD_CONTROL_PARAM_LIST value);
cSRUN_PD_CONTROL_PARAM_LIST SRunSM_Get_paramListSRun (void);


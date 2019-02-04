////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze_StateMachine.h
//!  \brief    
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_maze.h"
#include "app_common.h"
#include "navigation.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//! States of the main state machine of the maze algorithm.
//**********************************************************************************************************************
typedef enum
{
	eSTATE_MAIN_READY       = 0,		//!< Start position and car waits for a trigger.
	eSTATE_MAIN_DISCOVER,				//!< The car is driving through the maze, finding crossings and segments.
	eSTATE_MAIN_INCLINATION,			//!< The car has discovered the maze, it has to leave it now.
	eSTATE_MAIN_OUT						//!< The car is out of the maze
} eSTATE_MAIN;

//**********************************************************************************************************************
//! List that contain all of the control parameters.
//**********************************************************************************************************************
typedef struct
{
	cPD_CONTROLLER_PARAMS discover;		//!< Contains the control parameters of the Discover state.
	cPD_CONTROLLER_PARAMS inclination;	//!< Contains the control parameters of the Inclination state.
} cMAZE_PD_CONTROL_PARAM_LIST;

//**********************************************************************************************************************
//!
//**********************************************************************************************************************
typedef struct
{
	uint32_t left;
	uint32_t middle;
	uint32_t right;
} cSEGMENT_CROSSINGS;

//**********************************************************************************************************************
//!
//**********************************************************************************************************************
typedef struct
{
	cNAVI_STATE start;
	cNAVI_STATE end;
	uint32_t positive[3];		// 0 = L, 1 = M, 2 = R
	uint32_t negative[3];		// 0 = L, 1 = M, 2 = R
} cSEGMENT;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//! This function holds the main state machine that can navigate through the labyrinth.
//!
//! The main state machine has 4 states: Ready, Discover, Inclination, Out. The car starts in the Ready state and waits
//! for a trigger (remote, start gate). On trigger the car gets into the Discover state. In this state the car runs
//! through the labyrinth looking for the segments and the inclination point. When this point and all of the segments
//! are found and discovered, then the car gets into the Inclination state. Now the car plans a route to leave the
//! labyrinth and changes lane to the speed run route, behind the safety car. This is the Out state.
//!
//! @return -
//**********************************************************************************************************************
void MazeMainStateMachine (void);

void MazeStateMachinesInit (void);

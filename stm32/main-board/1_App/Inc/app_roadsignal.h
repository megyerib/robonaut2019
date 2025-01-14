////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze_roadsignal.h
//!  \brief     
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	NoCrossing = 0,
	CrossingRtoA,   // Becsatlakozás jobb kereszteződésbe: DoubleFar, DoubleNearLeft,  SingleRight
	CrossingLtoA,   // Becsatlakozás bal kereszteződésbe : DoubleFar, DoubleNearRight, SingleLeft
	CrossingAtoLB,  // Bal kereszteződés előre           : Single,    DoubleNearRight
	CrossingAtoRB,  // Jobb kereszteződés előre          : Single,    DoubleNearLeft
	CrossingBtoA_R, // Jobb kereszteződés vissza         : DoubleFar, DoubleNearRight,  SingleRight
	CrossingBtoA_L, // Bal kereszteződés vissza          : DoubleFar, DoubleNearLeft,   SingleLeft
	ExitForwardRight,
	ExitForwardLeft,
	ExitBackwardRight,
	ExitBackwardLeft
}
CROSSING_TYPE;

/*typedef enum
{
	NoSignal,
	Accelerate,
	Brake
}
RACE_ROAD_SIGNAL;*/

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void TaskInit_roadSignal(void);
void Task_roadSignal(void* p);
CROSSING_TYPE getCrossingType();
//RACE_ROAD_SIGNAL getRaceRoadSignal();

float getPrevLine();
float getLeftLine();
float getRightLine();
float get3Lines();

// END -----------------------------------------------------------------------------------------------------------------

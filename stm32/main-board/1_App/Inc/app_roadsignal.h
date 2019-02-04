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
	NoCrossing,
	CrossingRtoA,   // Becsatlakozás jobb keresztezõdésbe: DoubleFar, DoubleNearLeft,  SingleRight
	CrossingLtoA,   // Becsatlakozás bal keresztezõdésbe : DoubleFar, DoubleNearRight, SingleLeft
	CrossingAtoLB,  // Bal keresztezõdés elõre           : Single,    DoubleNearRight
	CrossingAtoRB,  // Jobb keresztezõdés elõre          : Single,    DoubleNearLeft
	CrossingBtoA_R, // Jobb keresztezõdés vissza         : DoubleFar, DoubleNearRight,  SingleRight
	CrossingBtoA_L, // Bal keresztezõdés vissza          : DoubleFar, DoubleNearLeft,   SingleLeft
	ExitForwardRight,
	ExitForwardLeft,
	ExitBackwardRight,
	ExitBackwardLeft
}
CROSSING_TYPE;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void TaskInit_roadSignal(void);
void Task_roadSignal(void* p);
CROSSING_TYPE getCrossingType();

float getPrevLine();
float getLeftLine();
float getRightLine();

// END -----------------------------------------------------------------------------------------------------------------

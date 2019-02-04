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
	CrossingRtoA,   // Becsatlakoz�s jobb keresztez�d�sbe: DoubleFar, DoubleNearLeft,  SingleRight
	CrossingLtoA,   // Becsatlakoz�s bal keresztez�d�sbe : DoubleFar, DoubleNearRight, SingleLeft
	CrossingAtoLB,  // Bal keresztez�d�s el�re           : Single,    DoubleNearRight
	CrossingAtoRB,  // Jobb keresztez�d�s el�re          : Single,    DoubleNearLeft
	CrossingBtoA_R, // Jobb keresztez�d�s vissza         : DoubleFar, DoubleNearRight,  SingleRight
	CrossingBtoA_L, // Bal keresztez�d�s vissza          : DoubleFar, DoubleNearLeft,   SingleLeft
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

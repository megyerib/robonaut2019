////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_race_roadsignal.h
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
	InitSlow,
	InitFast,
	Slow,
	Fast
}
RACE_RS;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

RACE_RS getRaceRs();
void setRaceRs(RACE_RS state);

// END -----------------------------------------------------------------------------------------------------------------

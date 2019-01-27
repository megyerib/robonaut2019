////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze.c
//!  \brief
//!  \details
//!
//! 	Gráf pontjai:
//! 		N:    N-edik keresztezõdés, elõre
//! 		N+16: N-edik keresztezõdés, hátra
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "navigation.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define JUNCTION_NUM_MAX  (16u)
#define JUNCTION_DIR_MASK (0x10u)
#define EDGE_NUM_MAX      (128u)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	right,
	left,
	front,
	rear
}
DIR;

typedef enum
{
	forward,
	back
}
ORI;

typedef struct
{
	uint8_t straightExists;
	uint8_t backExists;
	uint8_t curveExists;
	uint8_t straightVisited;
	uint8_t backVisited;
	uint8_t curveVisited;
	DIR     direction;
	cNAVI_STATE position;
}
JUNCTION;

typedef struct
{
	int junction    : 5;
	DIR direction   : 2;
	ORI orientation : 1;
}
EXIT;

typedef struct
{
	uint8_t start;
	uint8_t end;
	EXIT    start_type;
	EXIT    end_type;
}
EDGE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static JUNCTION junctions[JUNCTION_NUM_MAX];

static int junction_num = 0;

static int junctionFound = 0;
static int exitFound     = 0;

static EDGE edges[EDGE_NUM_MAX];

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int isMazeComplete();
static void checkRoadSignal();
static void lineController();
static void exitTrack();
static void handleJunction();

// Global function definitions -----------------------------------------------------------------------------------------

void maze()
{
	// TODO Reset position

	// TODO set junction position
	// junctions[0].position =

	junctions[0].straightExists  = 1;
	junctions[0].backExists      = 0;
	junctions[0].curveExists     = 0;
	junctions[0].straightVisited = 0;
	junctions[0].backVisited     = 0;
	junctions[0].curveVisited    = 0;

	junction_num++;

	while (1)
	{
		if (isMazeComplete())
		{
			// TODO lehajtás
		}
		else
		{
			lineController();

			checkRoadSignal();

			if (junctionFound)
			{
				handleJunction();
				junctionFound = 0;
			}

			if (exitFound)
			{
				exitTrack();
				exitFound = 0;
			}
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static int isMazeComplete()
{
	int junctionsUncplt = 0;

	for (int i = 0; i < junction_num; i++)
	{
		junctionsUncplt += junctions[i].backExists     ^ junctions[i].backVisited;
		junctionsUncplt += junctions[i].straightExists ^ junctions[i].straightVisited;
		junctionsUncplt += junctions[i].curveExists    ^ junctions[i].curveVisited;
	}

	if (junctionsUncplt == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static void checkRoadSignal()
{
	// TODO
	// Set junctionFound, exitFound
}

static void lineController()
{
	// TODO
}

static void exitTrack()
{

}

static void handleJunction()
{

}

// END -----------------------------------------------------------------------------------------------------------------

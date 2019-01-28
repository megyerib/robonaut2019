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
//!     Ha hátra megyünk, a bal-jobb irányt akkor is az elõre álló autóhoz képest vesszük.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "navigation.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define DONTCARE          (0)
#define JUNCTION_NUM_MAX  (16u)
#define JUNCTION_DIR_MASK (0x10u)
#define EDGE_NUM_MAX      (128u)
#define PATH_MAX_LEN      (20u)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	carDirForward,
	carDirBackward,

	carDirNum
}
CAR_DIR;

typedef enum
{
	exitFront,
	exitRear,
	exitSide,

	exitTypeNum
}
EXIT_TYPE;

typedef enum
{
	dirRight,
	dirLeft,

	dirNum
}
DIR;

typedef struct
{
	uint8_t     exitVisited[exitTypeNum];
	DIR         direction;
	cNAVI_STATE position;
}
JUNCTION;

typedef enum
{
	discoveryMode, // Megyünk a világba' egy ismeretlen úton
	travelMode     // El akarunk jutni egy ismert helyre egy ismert úton
}
MAZE_MODE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static JUNCTION junctions[JUNCTION_NUM_MAX];

static int junction_num = 0;

static int junctionFound = 0;
static int exitFound     = 0;

static int currentVertex = 0;

static CAR_DIR carDirection = carDirForward;
static MAZE_MODE mode = discoveryMode;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int isMazeComplete();
static void checkRoadSignal();
static void followLine();
static void exitTrack();
static void handleJunction();
static int isJunctionVisited();
static void registerPath();
static int isPathVisited();
static void addJunction();
static void chooseExit(EXIT_TYPE exitType, DIR exitDir);
static void switchLine(DIR d);

// Global function definitions -----------------------------------------------------------------------------------------

void maze()
{
	while (1)
	{
		if (isMazeComplete())
		{
			exitTrack();
		}
		else
		{
			followLine();

			checkRoadSignal();

			if (junctionFound)
			{
				handleJunction();
				junctionFound = 0;
			}

			if (exitFound)
			{

				exitFound = 0;
			}
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static int isMazeComplete()
{
	int exitsUncplt = 0;

	// Ha még nincs keresztezõdés (frissen rajtoltunk)
	if (junction_num == 0)
	{
		return 0;
	}

	for (int i = 0; i < junction_num; i++)
	{
		exitsUncplt += exitTypeNum;

		for (int j = 0; j < exitTypeNum; j++)
		{
			exitsUncplt -= junctions[i].exitVisited[j];
		}
	}

	if (exitsUncplt == 0)
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

static void followLine()
{
	// TODO
}

static void exitTrack()
{

}

static void handleJunction()
{
	int recentlyVisited = junction_num;

	// Még nem voltunk itt
	if (!isJunctionVisited())
	{
		addJunction();
	}

	// Út regisztrálása,
	// Ha még nem voltunk itt és már nem a rajt után vagyunk
	if (!isPathVisited() && recentlyVisited > 0)
	{
		registerPath();
	}

	if (mode == discoveryMode)
	{
		// TODO útvonaltervezés

		mode = travelMode;
	}

	// TODO kijárat kiválasztása a következõ szakasz alapján
}

static int isJunctionVisited()
{
	// TODO
	return 0;
}

static void registerPath()
{
	// Ha rajt után vagyunk, nem veszünk fel élt
	// TODO


}

static int isPathVisited()
{
	// TODO
	return 0;
}

// > ------> >
// < <------ <
static void addForwardEdges()
{

}

// > ------> >
// > <------ >
// < <------ <
// < ------> <
/*static void addEdgesWithReverse()
{

}*/

static void addJunction()
{
	// TODO
}

// TODO biztos meg lehet oldani ügyesebben is
// Ha meghívódik a függvény, a keresztezõdés közepe a két szenzor közt van.
//
// ----------------------
// -------------
//             ^
//   A keresztezõdés közepe
//
static void chooseExit(EXIT_TYPE exitType, DIR exitDir)
{
	// Elõre állunk
	if ((currentVertex & JUNCTION_DIR_MASK) == 0)
	{
		switch (exitType)
		{
			case exitFront:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirLeft);
				}
				else
				{
					switchLine(dirRight);
				}

				carDirection = carDirForward;

				break;
			}
			case exitRear:
			{
				carDirection = carDirBackward;

				break;
			}
			case exitSide:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirRight);
				}
				else
				{
					switchLine(dirLeft);
				}

				carDirection = carDirForward;

				break;
			}
			default:
			{
				break;
			}
		}
	}
	// Hátra állunk

	/*                   |
	 *                 Hátra
	 *                   |
	 *                   ^
	 *                  /|\
	 *          Balra  / | \  Jobbra
	 *  (Jobb kanyar) /  |  \ (Bal kanyar)
	 *
	 *                 Elõre
	 */

	else
	{
		switch (exitType)
		{
			case exitFront:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirRight);
				}
				else
				{
					switchLine(dirLeft);
				}

				carDirection = carDirBackward;

				break;
			}
			case exitRear:
			{
				carDirection = carDirForward;

				break;
			}
			case exitSide:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirLeft);
				}
				else
				{
					switchLine(dirRight);
				}

				carDirection = carDirBackward;

				break;
			}
			default:
			{
				break;
			}
		}
	}
}

static void switchLine(DIR d)
{
	// TODO
	// TODO arra is figyelni kell, ha éppen nem érzékeli mindkét vonalat a szenzor
}

// END -----------------------------------------------------------------------------------------------------------------

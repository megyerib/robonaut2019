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

#define TRACE(x)

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

typedef struct
{
	uint8_t startJunction;
	EXIT_TYPE startExit;
	uint8_t endJunction;
	EXIT_TYPE endExit;
}
EDGE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static JUNCTION junctions[JUNCTION_NUM_MAX];

static int junction_num = 0;

static int junctionDetected = 0; // Most
static int exitDetectedFw   = 0; // Elõre; Az aktuális szakaszon
static int exitDetectedBw   = 0; // Visszafele; Az aktuális szakaszon
static int exitFound        = 0; // Úgy általában
static int exitEdgeNum;

static int currentVertex = 0;

static CAR_DIR carDirection = carDirForward;

EXIT_TYPE pathExit; // Az a keresztezõdés-kijárat (típus), amelyiken az út végén elindulunk.

EDGE edges[EDGE_NUM_MAX];
int edge_num = 0;

EDGE currentEdge;

int path[PATH_MAX_LEN];
int path_len = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int isMazeComplete();
static void checkRoadSignal();
static void followLine();
static void calcExitPath();
static void handleJunction();
static int isJunctionVisited();
static void registerPath();
static int isPathVisited();
static void addJunction();
static void chooseExit(EXIT_TYPE exitType, DIR exitDir);
static void switchLine(DIR d);
static void calcPath(int junction, EXIT_TYPE exit);
static void registerExit();

// Global function definitions -----------------------------------------------------------------------------------------

void maze()
{
	while (1)
	{
		if (isMazeComplete())
		{
			calcExitPath();
			// TODO
			break;
		}
		else
		{
			followLine();
			checkRoadSignal();

			if (junctionDetected)
			{
				handleJunction();
				junctionDetected = 0;
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
	// Set junctionDetected, exitFound
}

static void followLine()
{
	// TODO
}

static void calcExitPath()
{
	calcPath(edges[exitEdgeNum].startJunction, edges[exitEdgeNum].startExit);
}

// This is where the fun begins
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

		if (!exitFound && (exitDetectedFw || exitDetectedBw))
		{
			registerExit();
		}
	}

	if (path_len == 0) // Nem terveztünk további útvonalat
	{
		// TODO útvonaltervezés

		if (path_len == 0) // Ha ebben a keresztezõdésben van a kijárat, kimenyünk.
		{
			chooseExit(pathExit, 0 /* TODO */); // Kiválasztjuk, hogy melyik irányba megyünk
			// TODO: currentVertex-bõl meg tudjuk mondani az irányt
		}
		else // Megyünk a következõ célpont felé
		{
			// TODO következõ kijárat kiválasztása az útvonal alapján

			// TODO útvonal frissítése (aktuális szakasz törlése)
		}
	}
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
	// Elõre mutató él felvétele

	edges[edge_num] = currentEdge;
	edge_num++;

	// Fordított él felvétele:
	// - A végpontok fel vannak cserélve és a másik irányba néznek.
	// - A kijáratok a végpontokhoz vannak kötve, tehát helyileg felcserélõdnek.

	edges[edge_num].startJunction = edges[edge_num-1].endJunction   ^ JUNCTION_DIR_MASK;
	edges[edge_num].endJunction   = edges[edge_num-1].startJunction ^ JUNCTION_DIR_MASK;

	edges[edge_num].startExit     = edges[edge_num-1].endExit;
	edges[edge_num].endExit       = edges[edge_num-1].startExit;

	edge_num++;
}

// > ------> >
// > <------ >
// < <------ <
// < ------> <
/*static void addEdgesWithReverse()
{
	// Ugyanaz, mint az elõzõ, csak a két élnek felvesszük egy-egy
	// olyan párját is, aminél a végpontok fel vannak cserélve.
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

static void calcPath(int junction, EXIT_TYPE exit)
{
	// TODO

	pathExit = exit;
}

static void registerExit()
{
	// TODO

	if (exitDetectedBw)
	{
		// TODO irány megfordítása
	}

	exitFound = 1;
}

// END -----------------------------------------------------------------------------------------------------------------

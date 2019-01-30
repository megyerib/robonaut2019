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
#include "math.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define DONTCARE          (0)
#define JUNCTION_NUM_MAX  (16u)
#define JUNCTION_DIR_MASK (0x10u)
#define EDGE_NUM_MAX      (128u)
#define PATH_MAX_LEN      (20u)
#define JUNCTION_DIST_THR (0.15f)
#define JUNCTION_NOT_DISCOVERED (-1)

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
	uint8_t   startJunction;
	EXIT_TYPE startExit;
	uint8_t   endJunction;
	EXIT_TYPE endExit;
	float     length;
}
EDGE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static JUNCTION junctions[JUNCTION_NUM_MAX];

static int junction_num = 0;

static int junctionDetected = 0; // Most
static int exitDetectedFw   = 0; // Elõre; Az aktuális szakaszon
static int exitDetectedBw   = 0; // Visszafele; Az aktuális szakaszon
static int exitFound        = 0; // Úgy általában

static EDGE exitEdge;

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
static int whichJunctionIsThis(cNAVI_STATE curNs);
static void registerPath();
static int isPathVisited();
static void addJunction(cNAVI_STATE pos, DIR d);
static void chooseExit(EXIT_TYPE exitType, DIR exitDir);
static void switchLine(DIR d);
static void calcPath(int junction, EXIT_TYPE exit);
static void registerExit();
static void addForwardEdges();

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
			exitsUncplt -= junctions[i].exitVisited[j];
	}

	return (exitsUncplt == 0) ? 1 : 0;
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
	calcPath(exitEdge.startJunction, exitEdge.startExit);
}

// This is where the fun begins
static void handleJunction()
{
	int recentlyVisited = junction_num;
	cNAVI_STATE curNs; // TODO
	DIR jDir = 0; // TODO keresztezõdés irányának meghatározása

	// Még nem voltunk itt
	if (whichJunctionIsThis(curNs) == JUNCTION_NOT_DISCOVERED)
	{
		addJunction(curNs, jDir);
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

		if (path_len == 0) // Ha ebben a keresztezõdésben van a kijelölt kanyarkijárat, kimegyünk.
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

// Returns -1 if none of the discovered ones
static int whichJunctionIsThis(cNAVI_STATE curNs)
{
	float dE;
	float dN;

	for (int i = 0; i < junction_num; i++)
	{
		dE = curNs.p.e - junctions[i].position.p.e;
		dN = curNs.p.n - junctions[i].position.p.n;

		float dist = sqrtf(dE * dE + dN * dN);

		if (dist <= JUNCTION_DIST_THR)
			return i; // WARNING it returns with the first match
	}

	return JUNCTION_NOT_DISCOVERED;
}

static void registerPath()
{
	addForwardEdges();
}

static int isPathVisited(EDGE e)
{
	int startVisited = junctions[e.startJunction].exitVisited[e.startExit];
	int endVisited   = junctions[e.endJunction  ].exitVisited[e.endExit  ];

	if ((startVisited ^ endVisited) != 0)
	{
		// Az elõzõ út egyik végét már felfedeztük, a másikat még nem (nem kéne bekövetkeznie)
		// ERROR
	}

	return (startVisited & endVisited);
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

static void addJunction(cNAVI_STATE pos, DIR d)
{
	junctions[junction_num].position = pos;

	for (int i = 0; i < exitTypeNum; i++)
		junctions[junction_num].exitVisited[i] = 0;

	junctions[junction_num].direction = d;

	junction_num++;
}

// Ha meghívódik a függvény, a keresztezõdés közepe a két szenzor közt van.
//
// ----------------------
// -------------
//             ^
//   A keresztezõdés közepe
//
static void chooseExit(EXIT_TYPE exitType, DIR exitDir)
{
	static const DIR exits[2][dirNum][exitTypeNum] =
	{ { {               // |Ori|Crvdir | Exit  |
			            // +---+-------+-------+
		dirLeft,        // | ^ | right | front |
		DONTCARE,       // | ^ | right | rear  |
		dirRight        // | ^ | right | side  |
		},{             // |   |       |       |
		dirRight,       // | ^ | left  | front |
		DONTCARE,       // | ^ | left  | rear  |
		dirLeft         // | ^ | left  | side  |
		} },{ {         // |   |       |       |
		dirRight,       // | v | right | front |
		DONTCARE,       // | v | right | rear  |
		dirLeft         // | v | right | side  |
		},{             // |   |       |       |
		dirLeft,        // | v | left  | front |
		DONTCARE,       // | v | left  | rear  |
		dirRight        // | v | left  | side  |
	} } };              // +---+-------+-------+

	static const CAR_DIR car_dir[2][dirNum][exitTypeNum] =
	{ { {               // |Ori|Crvdir | Exit  |
					    // +---+-------+-------+
		carDirForward,  // | ^ | right | front |
		carDirBackward, // | ^ | right | rear  |
		carDirForward   // | ^ | right | side  |
		},{             // |   |       |       |
		carDirForward,  // | ^ | left  | front |
		carDirBackward, // | ^ | left  | rear  |
		carDirForward   // | ^ | left  | side  |
		} },{ {         // |   |       |       |
		carDirBackward, // | v | right | front |
		carDirForward,  // | v | right | rear  |
		carDirBackward  // | v | right | side  |
		},{             // |   |       |       |
		carDirBackward, // | v | left  | front |
		carDirForward,  // | v | left  | rear  |
		carDirBackward  // | v | left  | side  |
	} } };              // +---+-------+-------+

	int orientation = ((currentVertex & JUNCTION_DIR_MASK) == 0);
	switchLine(exits[orientation][exitDir][exitType]);
	carDirection = car_dir[orientation][exitDir][exitType];
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

// A következõ keresztezõdésbe éréskor hívódik meg.
static void registerExit()
{
	if (exitDetectedFw)
	{
		exitEdge = currentEdge;
	}
	else // Irány megfordítása
	{
		exitEdge.startJunction = currentEdge.endJunction   & JUNCTION_DIR_MASK;
		exitEdge.endJunction   = currentEdge.startJunction & JUNCTION_DIR_MASK;

		exitEdge.startExit = currentEdge.endExit;
		exitEdge.endExit   = currentEdge.startExit;
	}

	exitFound = 1;
}

// END -----------------------------------------------------------------------------------------------------------------

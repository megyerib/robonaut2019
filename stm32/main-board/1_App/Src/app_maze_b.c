////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze.c
//!  \brief
//!  \details
//!
//! 	Gr�f pontjai:
//! 		N:    N-edik keresztez�d�s, el�re
//! 		N+16: N-edik keresztez�d�s, h�tra
//!
//!     Ha h�tra megy�nk, a bal-jobb ir�nyt akkor is az el�re �ll� aut�hoz k�pest vessz�k.
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
static int exitDetectedFw   = 0; // El�re; Az aktu�lis szakaszon
static int exitDetectedBw   = 0; // Visszafele; Az aktu�lis szakaszon
static int exitFound        = 0; // �gy �ltal�ban

static EDGE exitEdge;

static int currentVertex = 0;

static CAR_DIR carDirection = carDirForward;

EXIT_TYPE pathExit; // Az a keresztez�d�s-kij�rat (t�pus), amelyiken az �t v�g�n elindulunk.

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

	// Ha m�g nincs keresztez�d�s (frissen rajtoltunk)
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
	DIR jDir = 0; // TODO keresztez�d�s ir�ny�nak meghat�roz�sa

	// M�g nem voltunk itt
	if (whichJunctionIsThis(curNs) == JUNCTION_NOT_DISCOVERED)
	{
		addJunction(curNs, jDir);
	}

	// �t regisztr�l�sa,
	// Ha m�g nem voltunk itt �s m�r nem a rajt ut�n vagyunk
	if (!isPathVisited() && recentlyVisited > 0)
	{
		registerPath();

		if (!exitFound && (exitDetectedFw || exitDetectedBw))
		{
			registerExit();
		}
	}

	if (path_len == 0) // Nem tervezt�nk tov�bbi �tvonalat
	{
		// TODO �tvonaltervez�s

		if (path_len == 0) // Ha ebben a keresztez�d�sben van a kijel�lt kanyarkij�rat, kimegy�nk.
		{
			chooseExit(pathExit, 0 /* TODO */); // Kiv�lasztjuk, hogy melyik ir�nyba megy�nk
			// TODO: currentVertex-b�l meg tudjuk mondani az ir�nyt
		}
		else // Megy�nk a k�vetkez� c�lpont fel�
		{
			// TODO k�vetkez� kij�rat kiv�laszt�sa az �tvonal alapj�n

			// TODO �tvonal friss�t�se (aktu�lis szakasz t�rl�se)
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
		// Az el�z� �t egyik v�g�t m�r felfedezt�k, a m�sikat m�g nem (nem k�ne bek�vetkeznie)
		// ERROR
	}

	return (startVisited & endVisited);
}

// > ------> >
// < <------ <
static void addForwardEdges()
{
	// El�re mutat� �l felv�tele

	edges[edge_num] = currentEdge;
	edge_num++;

	// Ford�tott �l felv�tele:
	// - A v�gpontok fel vannak cser�lve �s a m�sik ir�nyba n�znek.
	// - A kij�ratok a v�gpontokhoz vannak k�tve, teh�t helyileg felcser�l�dnek.

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
	// Ugyanaz, mint az el�z�, csak a k�t �lnek felvessz�k egy-egy
	// olyan p�rj�t is, amin�l a v�gpontok fel vannak cser�lve.
}*/

static void addJunction(cNAVI_STATE pos, DIR d)
{
	junctions[junction_num].position = pos;

	for (int i = 0; i < exitTypeNum; i++)
		junctions[junction_num].exitVisited[i] = 0;

	junctions[junction_num].direction = d;

	junction_num++;
}

// Ha megh�v�dik a f�ggv�ny, a keresztez�d�s k�zepe a k�t szenzor k�zt van.
//
// ----------------------
// -------------
//             ^
//   A keresztez�d�s k�zepe
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
	// TODO arra is figyelni kell, ha �ppen nem �rz�keli mindk�t vonalat a szenzor
}

static void calcPath(int junction, EXIT_TYPE exit)
{
	// TODO

	pathExit = exit;
}

// A k�vetkez� keresztez�d�sbe �r�skor h�v�dik meg.
static void registerExit()
{
	if (exitDetectedFw)
	{
		exitEdge = currentEdge;
	}
	else // Ir�ny megford�t�sa
	{
		exitEdge.startJunction = currentEdge.endJunction   & JUNCTION_DIR_MASK;
		exitEdge.endJunction   = currentEdge.startJunction & JUNCTION_DIR_MASK;

		exitEdge.startExit = currentEdge.endExit;
		exitEdge.endExit   = currentEdge.startExit;
	}

	exitFound = 1;
}

// END -----------------------------------------------------------------------------------------------------------------

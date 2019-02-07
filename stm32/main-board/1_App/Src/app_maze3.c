////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze3.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include "start.h"
#include "dijkstra.h"
#include "navigation.h"
#include "app_roadsignal.h"
#include "random.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define JUNCTION_MAX_NUM (16u)
#define VERTEX_ORI_MASK (0x10);
#define NO_MATCHING_JUNCTION (-1);

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	start = 0,

	first_followLine,
	first_crossingFound,

	discovery_followLine,
	discovery_crossingFound,
	discovery_exitFound,

	travel_planRoute,
	//travel_choose,
	travel_followLine,
	travel_crossingFound,
	// Exit can't be newly found on a discovered path

	exit_planRoute,
	//exit_choose,
	exit_followLine,
	exit_crossingFound,

	error_random,

	finish
}
MAZE_STATE;

typedef enum
{
	dirLeft = 0,
	dirRight
}
DIR;

typedef enum
{
	oriForward = 0,
	oriBackward
}
ORI;

typedef enum
{
	exitSource,
	exitLeft,
	exitRight,

	exitNum
}
EXIT;

typedef struct
{
	cNAVI_STATE nav;
	DIR type;
	int visited[exitNum];
}
JUNCTION;

typedef struct
{
	int startVertex; // Orientáció számít
	int startExit;
	int endVertex; // Orientáció számít
	int EndExit;
	int length;
}
GRAPH_EDGE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static MAZE_STATE mazeState = start;
static JUNCTION junctions[JUNCTION_MAX_NUM];
static int junctionNum = 0;

// Queues
extern QueueHandle_t qNaviN_f;
extern QueueHandle_t qNaviE_f;
extern QueueHandle_t qNaviPSI_f;

static int curJunction;
static int curJunctionOri;

static GRAPH_EDGE curEdge;
static GRAPH_EDGE edges[32];

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void lineFollow();

static int newCrossing(cNAVI_STATE nav, ORI orientation);

static int isCrossing(CROSSING_TYPE c);
static int isCrossingLeft(CROSSING_TYPE c);
static int isCrossingForward(CROSSING_TYPE c);
static int isExit(CROSSING_TYPE c);

static cNAVI_STATE getPosition();
static int isJunctionDiscovered(int jnum);
static EXIT getEntryType(CROSSING_TYPE ctype);
static int whichJunctionIsThis(cNAVI_STATE pos);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Maze3 (void)
{

}

float Task_Maze3 ()
{
	float lineToFollow = 0; // TODO ezzel fogunk visszatérni
	int skip = 0;
	int crossing = getCrossingType();

	while (skip == 0)
	{
		switch (mazeState)
		{
		case start:
		{
			if (startGetState() != s0)
			{
				skip = 1;
			}
			else
			{
				mazeState = first_followLine;
			}

			break;
		}
		case first_followLine:
		{
			lineFollow();

			if (isCrossing(crossing))
			{
				mazeState = first_crossingFound;
			}
			else
			{
				skip = 1;
			}

			break;
		}
		case first_crossingFound: // new crossing; every exit unvisited
		{
			cNAVI_STATE curPos = getPosition();

			curJunctionOri = isCrossingForward(crossing) ? oriForward : oriBackward;
			curJunction    = newCrossing(curPos, curJunctionOri);

			JUNCTION* j = &junctions[curJunction];

			j->visited[exitSource]    = 0;
			j->visited[exitLeft]  = 0;
			j->visited[exitRight] = 0;

			j->type = isCrossingLeft(crossing) ? dirLeft : dirRight;

			mazeState = travel_planRoute; // TODO Biztos?

			break;
		}
		case discovery_followLine:
		{
			lineFollow();

			if (isCrossing(crossing))
			{
				mazeState = discovery_crossingFound;
			}
			else if (isExit(crossing))
			{
				mazeState = discovery_exitFound;
			}
			else
			{
				skip = 1;
			}

			break;
		}
		case discovery_crossingFound: // TODO
		{
			// Melyik keresztezõdésben vagyunk?
			cNAVI_STATE curPos = getPosition();
			curJunction = whichJunctionIsThis(curPos);


			// Felvesszük a keresztezõdést
			curJunctionOri = isCrossingForward(crossing) ? oriForward : oriBackward;

			JUNCTION* j = &junctions[curJunction];

			j->visited[exitSource]    = 0;
			j->visited[exitLeft]  = 0;
			j->visited[exitRight] = 0;

			j->type = isCrossingLeft(crossing) ? dirLeft : dirRight;

			// Kiegészítjük az élt, amirõl elindultunk
			EXIT entryType = getEntryType(crossing);

			// TODO ha már minden fel van fedezve, elrontottunk valamit, ezért átváltunk random módba

			break;
		}
		case discovery_exitFound: // TODO
		{
			// Felvesszük a kijáratot (mivel discovery, ezért nem találhattuk meg)

			break;
		}
		case travel_planRoute: // TODO
		{
			int curVertex = curJunction;

			// Keresztezõdésbõl hívjuk meg mindig ezt az állapotot
			if (curJunctionOri == oriForward)
			{
				if (junctions[curJunction].visited[exitLeft] == 0)
				{
					curEdge.startVertex = curVertex;
					curEdge.startExit   = exitLeft;

					lineToFollow = getLeftLine();
					mazeState = discovery_followLine;
					skip = 1;
				}
				else if (junctions[curJunction].visited[exitRight] == 0)
				{
					curEdge.startVertex = curVertex;
					curEdge.startExit   = exitRight;

					lineToFollow = getRightLine();
					skip = 1;
					mazeState = discovery_followLine;
				}
				else
				{
					// TODO Dijkstra
				}
			}
			else // Orientation: backward
			{
				if (junctions[curJunction].visited[exitSource] == 0)
				{
					curVertex |= VERTEX_ORI_MASK; // Fordított edge

					curEdge.startVertex = curVertex;
					curEdge.startExit   = exitSource;

					lineToFollow = getLeftLine();
					mazeState = discovery_followLine;
					skip = 1;
				}
				else
				{
					// TODO Dijkstra
				}
			}

			break;
		}
		case travel_followLine:
		{
			lineFollow();

			if (isCrossing(crossing))
			{
				mazeState = travel_crossingFound;
			}
			else
			{
				skip = 1;
			}

			break;
		}
		case travel_crossingFound: // TODO
		{
			// Továbblépünk a következõ keresztezõdésre

			break;
		}
		case exit_planRoute: // TODO
		{
			break;
		}
		case exit_followLine:
		{
			lineFollow();

			if (isCrossing(crossing))
			{
				mazeState = exit_crossingFound;
			}

			break;
		}
		case exit_crossingFound: // TODO
		{
			break;
		}
		case finish: // TODO
		{
			break;
		}
		case error_random:
		{
			if (crossing == CrossingAtoLB || crossing == CrossingAtoRB)
			{
				ANGVELd angvel = inertGetAngVel();

				if (random((float)angvel.omega_z))
				{
					lineToFollow = getLeftLine();
				}
				else
				{
					lineToFollow = getRightLine();
				}
			}
			else
			{
				lineToFollow = getPrevLine();
			}

			skip = 1;
			break;
		}
		}
	}

	return lineToFollow;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void lineFollow()
{

}

static int isCrossing(CROSSING_TYPE c)
{
	return (
		c == CrossingRtoA   ||
		c == CrossingLtoA   ||
		c == CrossingAtoLB  ||
		c == CrossingAtoRB  ||
		c == CrossingBtoA_R ||
		c == CrossingBtoA_L
	);
}

static int isCrossingLeft(CROSSING_TYPE c)
{
	return (
		c == CrossingLtoA   ||
		c == CrossingAtoLB  ||
		c == CrossingBtoA_L
	);
}

static int isCrossingForward(CROSSING_TYPE c)
{
	return (
		c == CrossingAtoLB  ||
		c == CrossingAtoRB
	);
}

static int isExit(CROSSING_TYPE c)
{
	return (
		c == ExitForwardRight  ||
		c == ExitForwardLeft   ||
		c == ExitBackwardRight ||
		c == ExitBackwardLeft
	);
}

// Global:
// junctions[]
// junctionNum
static int newCrossing(cNAVI_STATE nav, ORI orientation)
{
	JUNCTION* j = &junctions[junctionNum];
	int ret = junctionNum;

	if (orientation == oriBackward)
	{
		if (nav.psi < 180.0)
		{
			nav.psi += 180.0;
		}
		else
		{
			nav.psi -= 180.0;
		}
	}

	j->nav = nav;

	j->visited[exitSource]    = 0;
	j->visited[exitLeft]  = 0;
	j->visited[exitRight] = 0;

	junctionNum++;

	return ret;
}

static cNAVI_STATE getPosition()
{
	cNAVI_STATE ret;

	xQueuePeek(qNaviN_f,   &ret.p.n, 0);
	xQueuePeek(qNaviE_f,   &ret.p.n, 0);
	xQueuePeek(qNaviPSI_f, &ret.psi, 0);

	return ret;
}

static int isJunctionDiscovered(int jnum)
{
	JUNCTION* j = &junctions[jnum];

	return (j->visited[0] && j->visited[1] && j->visited[2]);
}

static EXIT getEntryType(CROSSING_TYPE ctype)
{
	if (ctype == CrossingAtoLB || ctype == CrossingAtoLB)
	{
		return exitSource;
	}
	else if (ctype == CrossingBtoA_L || ctype == CrossingRtoA)
	{
		return exitRight;
	}
	else if (ctype == CrossingBtoA_R || ctype == CrossingLtoA)
	{
		return exitLeft;
	}

	return 0;
}

static int whichJunctionIsThis(cNAVI_STATE pos)
{
	// TODO
	return NO_MATCHING_JUNCTION;
}

// END -----------------------------------------------------------------------------------------------------------------

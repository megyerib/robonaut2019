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
#define VERTEX_ORI_MASK (0x00000010); // 16
#define VERTEX_ORI_MASK_INVERSE (0xFFFFFFEF);
#define NO_MATCHING_JUNCTION (-1)

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
	exit_waitForExit,

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

/*typedef struct
{
	int startV; // Orientáció számít
	int startExit;
	int endV; // Orientáció számít
	int endExit;
	int cost;
}
MAZE_EDGE;*/

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
static int curVertex; // <- Junction, Ori

static int validVertexList[32];

static EDGE curEdge;

static EDGE edges[32];
static int edgeNum = 0;

static int exitRecentlyFound = 0;
static int exitVertex;
static EXIT exitJunctionExit;
static ORI exitOri;

// Tervezett út
static int plannedPath[JUNCTION_MAX_NUM * 2];
static int pathStartIndex;
static int pathEndIndex;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int newCrossing(cNAVI_STATE nav, ORI orientation);

static int isCrossing(CROSSING_TYPE c);
static int isCrossingLeft(CROSSING_TYPE c);
static int isCrossingForward(CROSSING_TYPE c);
static int isExit(CROSSING_TYPE c);

static cNAVI_STATE getPosition();
static int isJunctionDiscovered(int jnum);
static EXIT getEntryType(CROSSING_TYPE ctype);
static int whichJunctionIsThis(cNAVI_STATE pos);
static void addEdgeAndInverse(EDGE e);
static int vertexToJunctionNum(int vnum);

static int calcVertexNum(int junction, ORI orientation);
static void getTargetVerticeList(int* tlist);

static void planRouteToNearest(int startV);
static void planRoute(int startV, int endV);

// Global function definitions -----------------------------------------------------------------------------------------

// TODO pozíció korrigálása a keresztezõdésekben
// TODO keresztezõdés, vertex, orientáció aktualizálása

float MazeStm()
{
	float lineToFollow = getPrevLine();
	int skip = 0;
	int crossing;

	while (skip == 0)
	{
		crossing = getCrossingType();

		switch (mazeState)
		{
		case start:
		{
			// Várunk, amíg a start kapu el nem enged.

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
			// Követjük a vonalat, amíg az elsõ keresztezõdéshez nem érünk.

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
		case first_crossingFound:
		{
			// Felvesszük az elsõ keresztezõdést: Minden kijárat felderítetlen
			// Korrigáljuk a pozíciót egész 90 fokra. TODO
			// Továbbugrunk travel_planRoute-ba, ami az aktuális keresztezõdésbõl kitalálja, hogy hol vagyunk.

			cNAVI_STATE curPos = getPosition();

			curJunctionOri = isCrossingForward(crossing) ? oriForward : oriBackward;
			curJunction    = newCrossing(curPos, curJunctionOri);
			curVertex      = calcVertexNum(curJunction, curJunctionOri);

			JUNCTION* j = &junctions[curJunction];

			j->type = isCrossingLeft(crossing) ? dirLeft : dirRight;

			mazeState = travel_planRoute;

			break;
		}
		case discovery_followLine:
		{
			// Követjük az ismeretlen vonalat.
			// Ha keresztezõdést látunk, lekezeljük.
			// Ha kijáratot látunk, rögzítjük (discovery_exitFound), majd ha keresztezõdésbe értünk, elmentjük.

			if (isCrossing(crossing))
			{
				mazeState = discovery_crossingFound;
			}
			else if (isExit(crossing))
			{
				// TODO kijárat típusának mentése
				mazeState = discovery_exitFound;
			}
			else
			{
				skip = 1;
			}

			break;
		}
		case discovery_crossingFound:
		{
			// Keresztezõdésbe értünk.
			// Megnézzük, hogy voltunk-e már itt.
			// Ha igen, korrigáljuk a teljes pozíciót. TODO
			// Ha nem, felvesszük és csak a szöget korrigáljuk 90 fokra. TODO
			// A végén útvonalat tervezünk.
			// Ha láttunk kijáratot, elmentjük. TODO

			JUNCTION* j;
			cNAVI_STATE curPos = getPosition();

			// Melyik keresztezõdésben vagyunk?
			curJunctionOri = isCrossingForward(crossing) ? oriForward : oriBackward;
			curJunction = whichJunctionIsThis(curPos);

			if (curJunction == NO_MATCHING_JUNCTION)
			{
				// Felvesszük a keresztezõdést
				curJunction = newCrossing(curPos, curJunctionOri);

				j = &junctions[curJunction];

				j->type = isCrossingLeft(crossing) ? dirLeft : dirRight;
			}
			else
			{
				j = &junctions[curJunction];
				// TODO ha már minden fel van fedezve, elrontottunk valamit, ezért átváltunk random módba
			}

			// Kiegészítjük az élt, amirõl elindultunk
			curEdge.endExit = getEntryType(crossing);

			if (curJunctionOri == oriForward)
			{
				curEdge.endV = curJunction;
			}
			else
			{
				curEdge.endV = curJunction | VERTEX_ORI_MASK;
			}

			// Felfedeztük az él két végét -> visited = 1
			junctions[vertexToJunctionNum(curEdge.startV)].visited[curEdge.startExit] = 1;
			junctions[vertexToJunctionNum(curEdge.endV)].visited[curEdge.endExit] = 1;

			// Felvesszük a két új élt
			addEdgeAndInverse(curEdge);

			// TODO cost

			// Kijárat?
			if (exitRecentlyFound)
			{
				if (exitOri == oriForward)
				{
					exitVertex = edgeNum - 2; // Az elõzõleg látott él
					exitJunctionExit = edges[edgeNum - 2].startExit;
				}
				else
				{
					exitVertex = edgeNum - 1; // Az elõzõleg látott él inverze
					exitJunctionExit = edges[edgeNum - 1].startExit;
				}

				exitRecentlyFound = 0;
			}

			// Megyünk tovább
			mazeState = travel_planRoute;

			break;
		}
		case discovery_exitFound:
		{
			// Jelezzük a kijáratot (mivel discovery, ezért nem találhattuk meg)
			// Ha keresztezõdésbe érünk, felvesszük a kijáratot
			// Ha végeztünk a pályával, exit TODO

			exitRecentlyFound = 1;

			if (crossing == ExitForwardLeft || crossing == ExitForwardRight)
			{
				exitOri = oriForward;
			}
			else
			{
				exitOri = oriBackward;
			}

			mazeState = discovery_followLine;

			break;
		}
		case travel_planRoute:
		{
			// Ha a keresztezõdésünkben van felfedezetlen kijárat, arra megyünk
			// Ha nincs, keresünk magunknak (Dijkstra) TODO

			int curVertex = curJunction;

			// Keresztezõdésbõl hívjuk meg mindig ezt az állapotot
			if (curJunctionOri == oriForward)
			{
				// TODO inkább menjünk random, mi van, ha mindig balra próbál elindulni?
				if (junctions[curJunction].visited[exitLeft] == 0)
				{
					curEdge.startV = curVertex;
					curEdge.startExit   = exitLeft;

					lineToFollow = getLeftLine();
					mazeState = discovery_followLine;
					skip = 1;
				}
				else if (junctions[curJunction].visited[exitRight] == 0)
				{
					curEdge.startV = curVertex;
					curEdge.startExit   = exitRight;

					lineToFollow = getRightLine();
					skip = 1;
					mazeState = discovery_followLine;
				}
				else
				{
					planRouteToNearest(curVertex);
					// TODO Dijkstra után elindulunk
				}
			}
			else // Orientation: backward
			{
				if (junctions[curJunction].visited[exitSource] == 0)
				{
					curVertex |= VERTEX_ORI_MASK; // Fordított edge

					curEdge.startV = curVertex;
					curEdge.startExit   = exitSource;

					lineToFollow = getLeftLine();
					mazeState = discovery_followLine;
					skip = 1;
				}
				else
				{
					planRouteToNearest(curVertex);
					// TODO Dijkstra után elindulunk
				}
			}

			break;
		}
		case travel_followLine:
		{
			// Követjük a vonalat, ha keresztezõdésbe érünk, kiértékeljük.
			// Kijáratot nem vehetünk észre, mert azt egy discovery szakaszban már észrevettük.

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
		case travel_crossingFound:
		{
			// Megnézzük, hogy az út mely részén tartunk
			// Megyünk tovább, ha van még az útból
			// Ha nincs, travelPlan, ami kiválasztja a megfelelõ kijáratot

			// Végeztünk?
			if (pathStartIndex == pathEndIndex)
			{
				mazeState = travel_planRoute; // Kiválasztjuk a kimenõ irányt
			}
			else
			{
				pathStartIndex++;

				EXIT exitType = plannedPath[pathStartIndex];

				// Kijárat kiválasztása
				if (curJunctionOri == oriForward)
				{
					if (exitType == exitRight)
					{
						lineToFollow = getRightLine();
						mazeState = discovery_followLine;
						skip = 1;
					}
					else // exitType == exitLeft
					{
						lineToFollow = getLeftLine();
						mazeState = discovery_followLine;
						skip = 1;
					}
				}
				else // curJunctionOri == oriBackward
				{
					//lineToFollow = getPrevLine(); // Már kiválasztottuk
					mazeState = discovery_followLine;
					skip = 1;
				}
			}

			break;
		}
		case exit_planRoute: // TODO
		{
			// TODO curVertex

			planRoute(curVertex, exitVertex);

			// TODO Dijkstra után elindulunk

			break;
		}
		case exit_followLine:
		{
			// Vonalat követünk, ha keresztezõdést látunk, lekezeljük.

			if (isCrossing(crossing))
			{
				mazeState = exit_crossingFound;
			}
			else
			{
				skip = 1;
			}

			break;
		}
		case exit_crossingFound:
		{
			// Ha nem végeztünk, haladunk tovább az úton TODO
			// Ha végeztünk, kiválasztjuk a megfelelõ kijáratot és arra megyünk tovább. TODO

			// Végeztünk?
			if (pathStartIndex == pathEndIndex)
			{
				// TODO Kiválasztjuk a kimenõ irányt
				mazeState = exit_waitForExit;
			}
			else
			{
				pathStartIndex++;

				EXIT exitType = plannedPath[pathStartIndex];

				// Kijárat kiválasztása
				if (curJunctionOri == oriForward)
				{
					if (exitType == exitRight)
					{
						lineToFollow = getRightLine();
						mazeState = discovery_followLine;
						skip = 1;
					}
					else // exitType == exitLeft
					{
						lineToFollow = getLeftLine();
						mazeState = discovery_followLine;
						skip = 1;
					}
				}
				else // curJunctionOri == oriBackward
				{
					//lineToFollow = getPrevLine(); // Már kiválasztottuk
					mazeState = discovery_followLine;
					skip = 1;
				}
			}

			break;
		}
		case exit_waitForExit:
		{
			// Megyünk, amíg meg nem látjuk a lehajtót
			// Ha meglátjuk, kilépünk az állapotgépbõl

			if (isExit(crossing))
			{
				mazeState = finish;
			}
			else
			{
				skip = 1;
			}

			break;
		}
		case finish:
		{
			// TODO jelezzük valahogy a kilépést

			skip = 1;

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

static void addEdgeAndInverse(EDGE e)
{
	edges[edgeNum] = e;
	edgeNum++;

	// Invert edge
	edges[edgeNum].startV = e.endV   ^ VERTEX_ORI_MASK;
	edges[edgeNum].endV   = e.startV ^ VERTEX_ORI_MASK;
	edges[edgeNum].startExit   = e.endExit;
	edges[edgeNum].endExit     = e.startExit;
	edgeNum++;
}

static int vertexToJunctionNum(int vnum)
{
	return vnum & VERTEX_ORI_MASK_INVERSE;
}

static void getTargetVerticeList(int* tlist)
{
	for (int i = 0; i < junctionNum; i++)
	{
		if (junctions[i].visited[exitRight] || junctions[i].visited[exitLeft])
		{
			tlist[i] = 1;
		}
		if (junctions[i].visited[exitSource])
		{
			tlist[JUNCTION_MAX_NUM + i] = 1;
		}
	}
}

static int calcVertexNum(int junction, ORI orientation)
{
	if (orientation == oriBackward)
	{
		junction &= VERTEX_ORI_MASK;
	}

	return junction;
}

static void planRouteToNearest(int startV)
{
	getTargetVerticeList(validVertexList);

	dijkstraPathToNearestValid(edges, validVertexList, edgeNum, startV, plannedPath, &pathEndIndex);
	pathEndIndex--;
	pathStartIndex = 0;
}

static void planRoute(int startV, int endV)
{
	dijkstra(edges, edgeNum, startV, endV, plannedPath, &pathEndIndex);
	pathEndIndex--;
	pathStartIndex = 0;
}

// END -----------------------------------------------------------------------------------------------------------------

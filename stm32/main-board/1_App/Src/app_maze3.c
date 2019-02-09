////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze3.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <math.h>
#include "app_common.h"
#include "start.h"
#include "dijkstra.h"
#include "navigation.h"
#include "app_roadsignal.h"
#include "random.h"
#include "speed.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define JUNCTION_MAX_NUM (16u)
#define VERTEX_ORI_MASK (0x10) // 16
#define VERTEX_ORI_MASK_INVERSE (~VERTEX_ORI_MASK)
#define NO_MATCHING_JUNCTION (-1)
#define DISTANCE_TH  (1.5f)
#define ANGLE_TH     (PI/2)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	start = 0,

	first_followLine,
	first_crossingFound,

	discovery_followLine,
	discovery_crossingFound,
	//discovery_exitFound,

	travel_planRoute,
	//travel_choose,
	travel_followLine,
	travel_crossingFound,
	// Exit can't be newly found on a discovered path

	exit_planRoute,
	//exit_choose,
	exit_followLine,
	exit_crossingFound,
	exit_onExitSection,

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

static float startDistance;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int newCrossing(cNAVI_STATE nav, ORI orientation);

static int isCrossing(CROSSING_TYPE c);
static int isCrossingLeft(CROSSING_TYPE c);
static int isCrossingForward(CROSSING_TYPE c);
static int isExit(CROSSING_TYPE c);

static cNAVI_STATE getPosition();
static int isJunctionDiscovered(int jnum);
static EXIT getEntryType(CROSSING_TYPE ctype);
static int whichJunctionIsThis(cNAVI_STATE pos, ORI orientation);
static void addEdgeAndInverse(EDGE e);
static int vertexToJunctionNum(int vnum);

static int calcVertexNum(int junction, ORI orientation);
static void getTargetVerticeList(int* tlist);

static void planRouteToNearest(int startV);
static void planRoute(int startV, int endV);
static float normAngleRad(float angle);
static void resetPosition(cNAVI_STATE nav, ORI orientation);
static int isMazeFinished();

static void startDistanceMeasuring();
static int getDistanceDm();
static float roundRightAngle(float angle);

// Global function definitions -----------------------------------------------------------------------------------------

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
			// Korrigáljuk a pozíciót egész 90 fokra.
			// Továbbugrunk travel_planRoute-ba, ami az aktuális keresztezõdésbõl kitalálja, hogy hol vagyunk.

			cNAVI_STATE curPos = getPosition();
			curPos.psi = roundRightAngle(curPos.psi);
			naviResetNaviState(curPos);

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
				// Jelezzük a kijáratot (mivel discovery, ezért nem találhattuk meg)
				// Ha keresztezõdésbe érünk, felvesszük a kijáratot

				exitRecentlyFound = 1;

				if (crossing == ExitForwardLeft || crossing == ExitForwardRight)
				{
					exitOri = oriForward;
				}
				else
				{
					exitOri = oriBackward;
				}
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
			// Ha igen, korrigáljuk a teljes pozíciót.
			// Ha nem, felvesszük és csak a szöget korrigáljuk 90 fokra.
			// A végén útvonalat tervezünk.
			// Ha láttunk kijáratot, elmentjük.

			JUNCTION* j;

			cNAVI_STATE curPos = getPosition();

			// Melyik keresztezõdésben vagyunk?
			curJunctionOri = isCrossingForward(crossing) ? oriForward : oriBackward;
			curJunction    = whichJunctionIsThis(curPos, curJunctionOri);

			if (curJunction == NO_MATCHING_JUNCTION)
			{
				// Felvesszük a keresztezõdést
				curPos.psi = roundRightAngle(curPos.psi);
				naviResetNaviState(curPos);

				curJunction = newCrossing(curPos, curJunctionOri);
				curVertex   = calcVertexNum(curJunction, curJunctionOri);

				j = &junctions[curJunction];

				j->type = isCrossingLeft(crossing) ? dirLeft : dirRight;
			}
			else
			{
				j = &junctions[curJunction];
				resetPosition(j->nav, curJunctionOri);
				curVertex   = calcVertexNum(curJunction, curJunctionOri);

				if (isJunctionDiscovered(curJunction))
				{
					// Ha már minden fel van fedezve, elrontottunk valamit,
					// ezért átváltunk random módba
					mazeState = error_random;
					skip = 1;
				}
			}

			// Kiegészítjük az élt, amirõl elindultunk
			curEdge.endExit = getEntryType(crossing);
			curEdge.endV    = curVertex;
			curEdge.cost    = getDistanceDm();
			addEdgeAndInverse(curEdge);

			// Felfedeztük az él két végét -> visited = 1
			junctions[vertexToJunctionNum(curEdge.startV)].visited[curEdge.startExit] = 1;
			junctions[vertexToJunctionNum(curEdge.endV)].visited[curEdge.endExit]     = 1;

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
		case travel_planRoute:
		{
			// Ha a keresztezõdésünkben van felfedezetlen kijárat, arra megyünk
			// Ha nincs, keresünk magunknak (Dijkstra)

			// Keresztezõdésbõl hívjuk meg mindig ezt az állapotot
			if (curJunctionOri == oriForward)
			{
				// Inkább menjünk random, ne mindig egy irányba próbáljunk elindulni
				EXIT firstExit  = random((float)inertGetAngVel().omega_z) ? exitRight : exitLeft;
				EXIT secondExit = (firstExit == exitRight) ? exitLeft : exitRight;

				if (junctions[curJunction].visited[firstExit] == 0)
				{
					curEdge.startV    = curVertex;
					curEdge.startExit = firstExit;

					lineToFollow = (firstExit == exitRight) ? getRightLine() : getLeftLine();
				}
				else if (junctions[curJunction].visited[secondExit] == 0)
				{
					curEdge.startV    = curVertex;
					curEdge.startExit = secondExit;

					lineToFollow = (secondExit == exitRight) ? getRightLine() : getLeftLine();
				}
				else
				{
					planRouteToNearest(curVertex);

					lineToFollow = (edges[plannedPath[0]].startExit == exitRight) ? getRightLine() : getLeftLine();
				}
			}
			else // Orientation: backward
			{
				if (junctions[curJunction].visited[exitSource] == 0)
				{
					curEdge.startV    = curVertex;
					curEdge.startExit = exitSource;
				}
				else
				{
					planRouteToNearest(curVertex);
				}

				// Követjük tovább az egyetlen vonalat
			}

			startDistanceMeasuring();
			mazeState = discovery_followLine;
			skip = 1;

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

			// Pozíció beállítása
			curVertex      = edges[pathEndIndex].endV;
			curJunctionOri = (curVertex & VERTEX_ORI_MASK) ? oriForward : oriBackward;
			curJunction    = (curVertex & VERTEX_ORI_MASK_INVERSE);

			resetPosition(junctions[curJunction].nav, curJunctionOri);

			// Végeztünk az úttal?
			if (pathStartIndex == pathEndIndex)
			{
				mazeState = isMazeFinished() ? exit_planRoute : travel_planRoute;
			}
			else
			{
				pathStartIndex++;

				EXIT exitType = edges[plannedPath[pathStartIndex]].startV;

				// Kijárat kiválasztása
				lineToFollow = (exitType == exitRight) ? getRightLine() : getLeftLine();

				startDistanceMeasuring();
				mazeState = discovery_followLine;
				skip      = 1;
			}

			break;
		}
		case exit_planRoute:
		{
			planRoute(curVertex, exitVertex);

			if (edges[plannedPath[0]].startV == exitRight)
			{
				lineToFollow = getRightLine();
			}
			else
			{
				lineToFollow = getLeftLine();
			}

			mazeState = exit_followLine;
			skip = 1;

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
			// Ha nem végeztünk, haladunk tovább az úton
			// Ha végeztünk, kiválasztjuk a megfelelõ kijáratot és arra megyünk tovább.
			// Már nem frissítünk pozíciót

			// Végeztünk?
			if (pathStartIndex == pathEndIndex)
			{
				if (exitJunctionExit == exitLeft)
				{
					lineToFollow = getLeftLine();
				}
				else
				{
					lineToFollow = getRightLine();
				}

				mazeState = exit_onExitSection;
				skip = 1;
			}
			else
			{
				pathStartIndex++;

				EXIT exitType = edges[plannedPath[pathStartIndex]].startExit;

				// Kijárat kiválasztása
				if (exitType == exitRight)
				{
					lineToFollow = getRightLine();

				}
				else
				{
					lineToFollow = getLeftLine();
				}

				mazeState = exit_followLine;
				skip = 1;
			}

			break;
		}
		case exit_onExitSection:
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
			// Különben küövetjük az elõzõ vonalat

			skip = 1;
			break;
		}
		}
	}

	return lineToFollow;
}

float MazeStmRandom()
{
	float lineToFollow;
	float crossing = getCrossingType();

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
	junctionNum++;

	if (orientation == oriBackward)
	{
		nav.psi = normAngleRad(nav.psi + 180.0);
	}

	j->nav = nav;

	j->visited[exitSource] = 0;
	j->visited[exitLeft]   = 0;
	j->visited[exitRight]  = 0;

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

static int whichJunctionIsThis(cNAVI_STATE pos, ORI orientation)
{
	float distance;
	float dN;
	float dE;
	float dpsi;

	if (orientation == oriBackward)
	{
		pos.psi = normAngleRad(pos.psi + PI);
	}

	for (int i = 0; i < junctionNum; i++)
	{
		dN = junctions[i].nav.p.n - pos.p.n;
		dE = junctions[i].nav.p.e - pos.p.e;
		dpsi = fabs(junctions[i].nav.psi - pos.psi);
		distance = sqrtf(dN*dN + dE*dE);

		if (distance < DISTANCE_TH && dpsi < ANGLE_TH)
		{
			return i;
		}
	}

	return NO_MATCHING_JUNCTION;
}

static void addEdgeAndInverse(EDGE e)
{
	edges[edgeNum] = e;
	edgeNum++;

	// Invert edge
	edges[edgeNum].startV    = e.endV   ^ VERTEX_ORI_MASK;
	edges[edgeNum].endV      = e.startV ^ VERTEX_ORI_MASK;
	edges[edgeNum].startExit = e.endExit;
	edges[edgeNum].endExit   = e.startExit;
	edges[edgeNum].cost      = e.cost;
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
			tlist[VERTEX_ORI_MASK + i] = 1;
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

static float normAngleRad(float angle)
{
	if (angle > 0)
	{
		while (angle > PI+0.0001) // Float arithmetic
		{
			angle -= PI;
		}
	}
	else
	{
		while (angle < -PI-0.0001) // Float arithmetic
		{
			angle += PI;
		}
	}

	return angle;
}

static void resetPosition(cNAVI_STATE nav, ORI orientation)
{
	if (orientation == oriBackward)
	{
		nav.psi = normAngleRad(nav.psi + 180.0f);
	}

	naviResetNaviState(nav);
}

static int isMazeFinished()
{
	int ret = 1;

	for (int i = 0; i < junctionNum; i++)
	{
		ret &= junctions[i].visited[exitLeft];
		ret &= junctions[i].visited[exitRight];
		ret &= junctions[i].visited[exitSource];
	}

	return ret;
}

static void startDistanceMeasuring()
{
	startDistance = speedGetDistance();
}

static int getDistanceDm()
{
	return (int)((speedGetDistance() - startDistance)/10);
}

static float roundRightAngle(float angle)
{
	angle = normAngleRad(angle);
	angle /= PI/2;
	angle = round(angle);
	angle *= PI/2;

	return angle;
}

// END -----------------------------------------------------------------------------------------------------------------

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
	int startV; // Orient�ci� sz�m�t
	int startExit;
	int endV; // Orient�ci� sz�m�t
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

// Tervezett �t
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
			// V�runk, am�g a start kapu el nem enged.

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
			// K�vetj�k a vonalat, am�g az els� keresztez�d�shez nem �r�nk.

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
			// Felvessz�k az els� keresztez�d�st: Minden kij�rat felder�tetlen
			// Korrig�ljuk a poz�ci�t eg�sz 90 fokra.
			// Tov�bbugrunk travel_planRoute-ba, ami az aktu�lis keresztez�d�sb�l kital�lja, hogy hol vagyunk.

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
			// K�vetj�k az ismeretlen vonalat.
			// Ha keresztez�d�st l�tunk, lekezelj�k.
			// Ha kij�ratot l�tunk, r�gz�tj�k (discovery_exitFound), majd ha keresztez�d�sbe �rt�nk, elmentj�k.

			if (isCrossing(crossing))
			{
				mazeState = discovery_crossingFound;
			}
			else if (isExit(crossing))
			{
				// Jelezz�k a kij�ratot (mivel discovery, ez�rt nem tal�lhattuk meg)
				// Ha keresztez�d�sbe �r�nk, felvessz�k a kij�ratot

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
			// Keresztez�d�sbe �rt�nk.
			// Megn�zz�k, hogy voltunk-e m�r itt.
			// Ha igen, korrig�ljuk a teljes poz�ci�t.
			// Ha nem, felvessz�k �s csak a sz�get korrig�ljuk 90 fokra.
			// A v�g�n �tvonalat tervez�nk.
			// Ha l�ttunk kij�ratot, elmentj�k.

			JUNCTION* j;

			cNAVI_STATE curPos = getPosition();

			// Melyik keresztez�d�sben vagyunk?
			curJunctionOri = isCrossingForward(crossing) ? oriForward : oriBackward;
			curJunction    = whichJunctionIsThis(curPos, curJunctionOri);

			if (curJunction == NO_MATCHING_JUNCTION)
			{
				// Felvessz�k a keresztez�d�st
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
					// Ha m�r minden fel van fedezve, elrontottunk valamit,
					// ez�rt �tv�ltunk random m�dba
					mazeState = error_random;
					skip = 1;
				}
			}

			// Kieg�sz�tj�k az �lt, amir�l elindultunk
			curEdge.endExit = getEntryType(crossing);
			curEdge.endV    = curVertex;
			curEdge.cost    = getDistanceDm();
			addEdgeAndInverse(curEdge);

			// Felfedezt�k az �l k�t v�g�t -> visited = 1
			junctions[vertexToJunctionNum(curEdge.startV)].visited[curEdge.startExit] = 1;
			junctions[vertexToJunctionNum(curEdge.endV)].visited[curEdge.endExit]     = 1;

			// Kij�rat?
			if (exitRecentlyFound)
			{
				if (exitOri == oriForward)
				{
					exitVertex = edgeNum - 2; // Az el�z�leg l�tott �l
					exitJunctionExit = edges[edgeNum - 2].startExit;
				}
				else
				{
					exitVertex = edgeNum - 1; // Az el�z�leg l�tott �l inverze
					exitJunctionExit = edges[edgeNum - 1].startExit;
				}

				exitRecentlyFound = 0;
			}

			// Megy�nk tov�bb
			mazeState = travel_planRoute;

			break;
		}
		case travel_planRoute:
		{
			// Ha a keresztez�d�s�nkben van felfedezetlen kij�rat, arra megy�nk
			// Ha nincs, keres�nk magunknak (Dijkstra)

			// Keresztez�d�sb�l h�vjuk meg mindig ezt az �llapotot
			if (curJunctionOri == oriForward)
			{
				// Ink�bb menj�nk random, ne mindig egy ir�nyba pr�b�ljunk elindulni
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

				// K�vetj�k tov�bb az egyetlen vonalat
			}

			startDistanceMeasuring();
			mazeState = discovery_followLine;
			skip = 1;

			break;
		}
		case travel_followLine:
		{
			// K�vetj�k a vonalat, ha keresztez�d�sbe �r�nk, ki�rt�kelj�k.
			// Kij�ratot nem vehet�nk �szre, mert azt egy discovery szakaszban m�r �szrevett�k.

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
			// Megn�zz�k, hogy az �t mely r�sz�n tartunk
			// Megy�nk tov�bb, ha van m�g az �tb�l
			// Ha nincs, travelPlan, ami kiv�lasztja a megfelel� kij�ratot

			// Poz�ci� be�ll�t�sa
			curVertex      = edges[pathEndIndex].endV;
			curJunctionOri = (curVertex & VERTEX_ORI_MASK) ? oriForward : oriBackward;
			curJunction    = (curVertex & VERTEX_ORI_MASK_INVERSE);

			resetPosition(junctions[curJunction].nav, curJunctionOri);

			// V�gezt�nk az �ttal?
			if (pathStartIndex == pathEndIndex)
			{
				mazeState = isMazeFinished() ? exit_planRoute : travel_planRoute;
			}
			else
			{
				pathStartIndex++;

				EXIT exitType = edges[plannedPath[pathStartIndex]].startV;

				// Kij�rat kiv�laszt�sa
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
			// Vonalat k�vet�nk, ha keresztez�d�st l�tunk, lekezelj�k.

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
			// Ha nem v�gezt�nk, haladunk tov�bb az �ton
			// Ha v�gezt�nk, kiv�lasztjuk a megfelel� kij�ratot �s arra megy�nk tov�bb.
			// M�r nem friss�t�nk poz�ci�t

			// V�gezt�nk?
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

				// Kij�rat kiv�laszt�sa
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
			// Megy�nk, am�g meg nem l�tjuk a lehajt�t
			// Ha megl�tjuk, kil�p�nk az �llapotg�pb�l

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
			// TODO jelezz�k valahogy a kil�p�st

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
			// K�l�nben k��vetj�k az el�z� vonalat

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

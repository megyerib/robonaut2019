////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze_StateMachine.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_maze_StateMachine.h"
#include "app_maze.h"
#include "app_roadsignal.h"
#include "line.h"
#include "bsp_servo.h"
#include "app_common.h"
#include "speed.h"
#include <string.h>

// Defines -------------------------------------------------------------------------------------------------------------

#define MAZE_MAP_MAX_SEGEMENTS	(20u)	//!< The map can contain this much segments.
#define MAZE_FINDABLE_SEGEMNST	(12u)	//!< The labyrinth has this much segments that are need to be discovered.
#define MAZE_N_ACCURACY			(10u)	//!< Two N values are the same if they differ this much.

#define MAZE_INCLIN_TURN_DIST	(0.35f)
#define MAZE_INCLIN_LEAVE_DIST	(0.35f)
#define MAZE_INCLIN_TURN_ANGLE	(-30.0f*PI/180.0f)

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Flag that indicates if the maze task is finished.
bool mazeFinished;

eSTATE_MAIN smMainState;	//! This variable indicates the actual state of the main state machine of the maze algorithm.

cSEGMENT map[MAZE_MAP_MAX_SEGEMENTS];	//! The graph map of the labyrinth.
bool segments[MAZE_FINDABLE_SEGEMNST];	//! A list of the discoverable segments. A bit is set when the segment was found and the car has driven it through.

//! The number of the segment where the exit point is to be found.
//uint32_t inclinSegment;
//uint8_t inclinSegmentOrient;	// 0 = negative, 1 = positive
//bool inclinDirection;			// left = false, rigth = true
//uint32_t inclinTime;
//bool inclinStarted;
//bool inclinTimeUp;
//uint32_t inclinStopTime = 400;
//
uint32_t actualSegment;
uint32_t nextNewSegmentIndex;
uint32_t alreadyFoundSegment;
//bool turnOffLineFollow;


cPD_CNTRL_PARAMS mazeActualParams;		//! The controller parameters of the actual state in which the car is currently.
cMAZE_PD_CONTROL_PARAM_LIST paramList;	//! List of the controller parameters for all of the main states.

static cNAVI_STATE naviStateCrossing;
static cNAVI_STATE naviStateCar;

static uint8_t neighbourMatrixA [MAZE_MAP_MAX_SEGEMENTS][MAZE_MAP_MAX_SEGEMENTS];
static uint8_t neighbourMatrixB [MAZE_MAP_MAX_SEGEMENTS][MAZE_MAP_MAX_SEGEMENTS];

static uint8_t exitRoute[20];

extern QueueHandle_t qNaviN_f;
extern QueueHandle_t qNaviE_f;
extern QueueHandle_t qNaviPSI_f;

float mazeActLine;
float mazePrevLine;
float mazeServoAngle;

static eSTATE_INCLIN inclinState;
static float inclinDistStart;
static float inclinDistEnd;


// Local (static) function prototypes ----------------------------------------------------------------------------------

static bool mazeCrossingAlreadyFound 	(const cNAVI_STATE crossingNaviState);
static void mazeUpdateMap 				(const CROSSING_TYPE crossingType);
static void mazeCheckDiscoveredSegments (void);
static void mazeMergeSegments 			(void);
static void mazeStateMachineDiscovery   (void);
static bool mazeAllSegmentsDiscovered	(void);
static void mazePlanExitRoute			(void);

// Global function definitions -----------------------------------------------------------------------------------------

void MazeStateMachinesInit (void)
{
	uint8_t i;

	mazeFinished = true;
	smMainState  = eSTATE_MAIN_INCLINATION;

	for (i = 0; i < MAZE_MAP_MAX_SEGEMENTS; i++)
	{
		map[i].end.p.n = 0;
		map[i].end.p.e = 0;
		map[i].end.psi = 0;
		map[i].start.p.n = 0;
		map[i].start.p.e = 0;
		map[i].start.psi = 0;
		map[i].negative[0] = 0;
		map[i].negative[1] = 0;
		map[i].negative[2] = 0;
		map[i].positive[0] = 0;
		map[i].positive[1] = 0;
		map[i].positive[2] = 0;
	}

	// Reset trackers.
	memset(segments, 0, MAZE_FINDABLE_SEGEMNST);

	//inclinSegment = 0;
	//actualSegment = 0;
	//nextNewSegmentIndex = 1;
	//alreadyFoundSegment = 0;

	mazeActLine = 0.0f;
	mazePrevLine = 0.0f;

	(void)exitRoute;		// TODO
	(void)naviStateCar;		// TODO

	inclinState = eSTATE_INCLIN_START;
}

//! Function: MazeMainStateMachine
float MazeMainStateMachine (void)
{
	float lineToFollow;
	float crossing;

	switch (smMainState)
	{
		case eSTATE_MAIN_READY:
		{
			// Standing in the start position and radio trigger.
			mazeActualParams.Speed = 0;

			// TODO on race use this! : if (startGetState() == s0)
			if (true)
			{
				// Trigger received -> DISCOVER state.
				smMainState = eSTATE_MAIN_DISCOVER;
				actualSegment = 1;
				nextNewSegmentIndex = 2;

				mazePlanExitRoute();
			}
			break;
		}
		case eSTATE_MAIN_DISCOVER:
		{
			crossing = getCrossingType();

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

			break;
		}
		case eSTATE_MAIN_INCLINATION:
		{
			// Load in control parameters.
			mazeActualParams.Kp = paramList.inclination.Kp;
			mazeActualParams.Kd = paramList.inclination.Kd;
			mazeActualParams.Speed = paramList.inclination.Speed;

			mazeStateMachineInclination();

			break;
		}
		case eSTATE_MAIN_OUT:
		{
			// Maze task is finished.
			mazeFinished = true;

			break;
		}
		default:
		{
			// NOP
			break;
		}
	}
}

void mazeStateMachineInclination (void)
{
	float lenght;

	// Actual distance from start.
	inclinDistEnd = speedGetDistance();

	// Calculate distance.
	lenght = inclinDistEnd - inclinDistStart;

	// In the right place. Inclination to the right.
	switch (inclinState)
	{
		case eSTATE_INCLIN_START:
		{
			// Distance measurement.
			inclinDistStart = speedGetDistance();

			// Turn off the line follow controller.
			appMazeLineFollow_OFF();

			// Change state.
			inclinState = eSTATE_INCLIN_STEER_OUT;
			break;
		}
		case eSTATE_INCLIN_STEER_OUT:
		{
			// Turn the wheel to the right.
			mazeServoAngle = MAZE_INCLIN_TURN_ANGLE;

			// After a given distance, change state.
			if (lenght > MAZE_INCLIN_TURN_DIST)
			{
				// Reset distance measurement.
				inclinDistStart = speedGetDistance();

				inclinState = eSTATE_INCLIN_LEAVE_LINE;
			}

			break;
		}
		case eSTATE_INCLIN_LEAVE_LINE:
		{
			// Steer back to straight.
			mazeServoAngle = SERVO_MIDDLE_RAD;

			// After a given distance catch the line.
			if (lenght > MAZE_INCLIN_LEAVE_DIST)
			{
				inclinState = eSTATE_INCLIN_CATCH_LINE;
			}

			break;
		}
		case eSTATE_INCLIN_CATCH_LINE:
		{
			if (lineGetRawFrontFloat().cnt != 0)
			{
				// Turn back the line follow controller.
				appMazeLineFollow_ON();

				mazeFinished = true;
			}
			break;
		}
		default:
		{
			break;
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void mazeStateMachineDiscovery (void)
{
	CROSSING_TYPE currCross;

	//  Ask the line sensor for the actual road sign.
	currCross = getCrossingType();

	// Select the appropriate reaction.
	if (currCross != NoCrossing)
	{
		// Get navigation status.
		xQueuePeek(qNaviN_f,   &naviStateCrossing.p.n, 0);
		xQueuePeek(qNaviE_f,   &naviStateCrossing.p.e, 0);
		xQueuePeek(qNaviPSI_f, &naviStateCrossing.psi, 0);

		// Check if the crossing was already found.
		if (mazeCrossingAlreadyFound(naviStateCrossing) == false)
		{
			// 2 new segments found.
			mazeUpdateMap(currCross);

			// Follow one direction.
			if (map[actualSegment].positive[0] != 0)
			{
				// Turn left.
				mazeActLine = getLeftLine();
				actualSegment = map[actualSegment].positive[0];
			}
			else if (map[actualSegment].positive[1] != 0)
			{
				// Turn middle.
				mazeActLine = getPrevLine();
				actualSegment = map[actualSegment].positive[1];
			}
			else if (map[actualSegment].positive[2] != 0)
			{
				// Turn right.
				mazeActLine = getRightLine();
				actualSegment = map[actualSegment].positive[2];
			}
		}
		else
		{
			// Turn to an undiscovered direction.
			if (map[actualSegment].positive[0] != alreadyFoundSegment && map[actualSegment].positive[0] != 0)
			{
				// Turn left.
				mazeActLine = getLeftLine();
				actualSegment = map[actualSegment].positive[0];
			}
			else if (map[actualSegment].positive[1] != alreadyFoundSegment && map[actualSegment].positive[1] != 0)
			{
				// Turn middle.
				mazeActLine = getPrevLine();
				actualSegment = map[actualSegment].positive[1];
			}
			else if (map[actualSegment].positive[2] != alreadyFoundSegment && map[actualSegment].positive[2] != 0)
			{
				// Turn right.
				mazeActLine = getRightLine();
				actualSegment = map[actualSegment].positive[2];
			}
			else
			{
				// Turn random.
				uint8_t random = (uint32_t)(naviStateCrossing.psi * 10000) % 10;

				if (random >= 5)
				{
					mazeActLine = getPrevLine();
				}
				else
				{
					if (map[actualSegment].positive[0] != 0)
					{
						mazeActLine = getLeftLine();
					}
					else if (map[actualSegment].positive[2] != 0)
					{
						mazeActLine = getRightLine();
					}
				}
			}
		}

		// Merge segments if their navi state is the same at end or start position.
		mazeMergeSegments();

		// Mark segment discovered.
		mazeCheckDiscoveredSegments();

		// Check if the labyrinth is fully discovered.
		if (mazeAllSegmentsDiscovered() == true)
		{
			smMainState = eSTATE_MAIN_INCLINATION;
		}
	}
	else if (/*currCross ==*/false )
	{
		/*inclinSegment = actualSegment;

		inclinSegmentOrient = 1;*/
	}
	else if (/*currCross == ExitBackward*/ false)
	{
		/*inclinSegment = actualSegment;

		inclinSegmentOrient = 0;*/
	}
	else
	{
		// Follow the main line.
		mazeActLine = getPrevLine();
	}

}

static bool mazeCrossingAlreadyFound (const cNAVI_STATE crossingNaviState)
{
	bool newFound = false;
	uint8_t i;
	uint8_t nearestSegment;
	float distN = map[0].start.p.n;

	for (i = 0; i < MAZE_MAP_MAX_SEGEMENTS; i++)
	{
		if (map[i].start.p.n <= distN)
		{
			distN = map[i].start.p.n;
			nearestSegment = i;
		}
	}

	if (distN < MAZE_N_ACCURACY)
	{
		newFound = true;
		alreadyFoundSegment = nearestSegment;
	}

	return newFound;
}

static void mazeUpdateMap (const CROSSING_TYPE crossingType)
{
	// Save navigation status.
	map[actualSegment].end = naviStateCrossing;

	// Type of the crossing.
	//				  CL
	// 				 /
	//  		____/
	//	A _____________ B
	//			____
	//			    \									.
	//				 \									.
	//				  CR
	switch (crossingType)
	{
		case CrossingAtoLB:
		{
			// From A to B and CL direction.

			// Directions in the crossing from A.
			map[actualSegment].positive[0] = nextNewSegmentIndex + 1;
			map[actualSegment].positive[1] = nextNewSegmentIndex;
			map[actualSegment].positive[2] = 0;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex + 1] = 1;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex] = 1;

			// New segment B.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = 0;
			map[nextNewSegmentIndex].negative[1] = actualSegment;
			map[nextNewSegmentIndex].negative[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			nextNewSegmentIndex++;

			// New segment CL.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = 0;
			map[nextNewSegmentIndex].negative[1] = actualSegment;
			map[nextNewSegmentIndex].negative[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			nextNewSegmentIndex++;
			break;
		}
		case CrossingAtoRB:
		{
			// From A to B and CR direction.

			// Directions in the crossing from A.
			map[actualSegment].positive[0] = 0;
			map[actualSegment].positive[1] = nextNewSegmentIndex;
			map[actualSegment].positive[2] = nextNewSegmentIndex + 1;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex + 1] = 1;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex] = 1;

			// New segment B
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = 0;
			map[nextNewSegmentIndex].negative[1] = actualSegment;
			map[nextNewSegmentIndex].negative[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			nextNewSegmentIndex++;

			// New segment CR
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = 0;
			map[nextNewSegmentIndex].negative[1] = actualSegment;
			map[nextNewSegmentIndex].negative[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			nextNewSegmentIndex++;
			break;
		}
		case CrossingBtoA_L: // TODO
		{
			// From B to A and CL direction.

			// Directions in the crossing from B.
			map[actualSegment].positive[0] = 0;
			map[actualSegment].positive[1] = nextNewSegmentIndex + 1;
			map[actualSegment].positive[2] = 0;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex+1] = 1;

			// New segment CL.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive[0] = 0;
			map[nextNewSegmentIndex].positive[1] = nextNewSegmentIndex + 1;
			map[nextNewSegmentIndex].positive[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex + 1] = 1;
			nextNewSegmentIndex++;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].negative[1] = actualSegment;
			map[nextNewSegmentIndex].negative[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex - 1] = 1;
			nextNewSegmentIndex++;
			break;
		}
		case CrossingBtoA_R:
		{
			// From B to A and CR direction.

			// Directions in the crossing from B.
			map[actualSegment].positive[0] = 0;
			map[actualSegment].positive[1] = nextNewSegmentIndex;
			map[actualSegment].positive[2] = 0;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex] = 1;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = 0;
			map[nextNewSegmentIndex].negative[1] = actualSegment;
			map[nextNewSegmentIndex].negative[2] = nextNewSegmentIndex + 1;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex + 1] = 1;
			nextNewSegmentIndex++;

			// New segment CR.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive[0] = 0;
			map[nextNewSegmentIndex].positive[1] = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].positive[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex - 1] = 1;
			nextNewSegmentIndex++;
			break;
		}
		case CrossingLtoA:
		{
			// From CL to A.

			// Direction from CL in the crossing.
			map[actualSegment].positive[0] = 0;
			map[actualSegment].positive[1] = nextNewSegmentIndex;
			map[actualSegment].positive[2] = 0;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex] = 1;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = actualSegment;
			map[nextNewSegmentIndex].negative[1] = nextNewSegmentIndex + 1;
			map[nextNewSegmentIndex].negative[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex + 1] = 1;
			nextNewSegmentIndex++;

			// New segment B.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive[0] = 0;
			map[nextNewSegmentIndex].positive[1] = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].positive[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex - 1] = 1;
			nextNewSegmentIndex++;
			break;
		}
		case CrossingRtoA:
		{
			// From CR to A.

			// Direction from CR in the crossing.
			map[actualSegment].positive[0] = 0;
			map[actualSegment].positive[1] = nextNewSegmentIndex;
			map[actualSegment].positive[2] = 0;
			neighbourMatrixA[actualSegment][nextNewSegmentIndex] = 1;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative[0] = 0;
			map[nextNewSegmentIndex].negative[1] = nextNewSegmentIndex + 1;
			map[nextNewSegmentIndex].negative[2] = actualSegment;
			neighbourMatrixB[nextNewSegmentIndex][actualSegment] = 1;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex + 1] = 1;
			nextNewSegmentIndex++;

			// New segment B.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive[0] = 0;
			map[nextNewSegmentIndex].positive[1] = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].positive[2] = 0;
			neighbourMatrixB[nextNewSegmentIndex][nextNewSegmentIndex - 1] = 1;
			nextNewSegmentIndex++;
			break;
		}
		default:
		{
			// NOP
			break;
		}
	}
}

static void mazeCheckDiscoveredSegments (void)
{
	uint8_t i;

	for (i = 0; i < MAZE_MAP_MAX_SEGEMENTS; i++)
	{
		if (map[i].start.p.n != 0 && map[i].start.p.e != 0 && map[i].start.psi != 0
		   && map[i].end.p.n != 0 && map[i].end.p.e != 0   && map[i].end.psi != 0)
		{
			segments[i-1] = 1;
		}
	}
}

static void mazeMergeSegments (void)
{
	uint8_t i;

	if (   map[actualSegment].positive[0] == map[alreadyFoundSegment].positive[0]
		&& map[actualSegment].positive[1] == map[alreadyFoundSegment].positive[1]
		&& map[actualSegment].positive[2] == map[alreadyFoundSegment].positive[2]
		)
	{
		// The turning directions in the same orientation are the same
		if (map[actualSegment].negative[0] == 0 && map[actualSegment].negative[1] == 0 && map[actualSegment].negative[2]  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[actualSegment].negative[0] = map[alreadyFoundSegment].negative[0];
			map[actualSegment].negative[1] = map[alreadyFoundSegment].negative[1];
			map[actualSegment].negative[2] = map[alreadyFoundSegment].negative[2];
		}
		else if (map[alreadyFoundSegment].negative[0] == 0 && map[alreadyFoundSegment].negative[1] == 0 && map[alreadyFoundSegment].negative[2]  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[alreadyFoundSegment].negative[0] = map[actualSegment].negative[0];
			map[alreadyFoundSegment].negative[1] = map[actualSegment].negative[1];
			map[alreadyFoundSegment].negative[2] = map[actualSegment].negative[2];
		}
	}
	else if (   map[actualSegment].positive[0] == map[alreadyFoundSegment].negative[0]
			 && map[actualSegment].positive[1] == map[alreadyFoundSegment].negative[1]
			 && map[actualSegment].positive[2] == map[alreadyFoundSegment].negative[2]
			)
	{
		//
		if (map[actualSegment].negative[0] == 0 && map[actualSegment].negative[1] == 0 && map[actualSegment].negative[2]  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[actualSegment].negative[0] = map[alreadyFoundSegment].positive[0];
			map[actualSegment].negative[1] = map[alreadyFoundSegment].positive[1];
			map[actualSegment].negative[2] = map[alreadyFoundSegment].positive[2];

		}
		else if (map[alreadyFoundSegment].negative[0] == 0 && map[alreadyFoundSegment].negative[1] == 0 && map[alreadyFoundSegment].negative[2]  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[alreadyFoundSegment].positive[0] = map[actualSegment].negative[0];
			map[alreadyFoundSegment].positive[1] = map[actualSegment].negative[1];
			map[alreadyFoundSegment].positive[2] = map[actualSegment].negative[2];

		}
	}
	else if (   map[actualSegment].negative[0] == map[alreadyFoundSegment].negative[0]
			 && map[actualSegment].negative[1] == map[alreadyFoundSegment].negative[1]
			 && map[actualSegment].negative[2] == map[alreadyFoundSegment].negative[2]
			)
	{
		// The turning directions in the same orientation are the same

		if (map[actualSegment].negative[0] == 0 && map[actualSegment].negative[1] == 0 && map[actualSegment].negative[2]  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[actualSegment].positive[0] = map[alreadyFoundSegment].positive[0];
			map[actualSegment].positive[1] = map[alreadyFoundSegment].positive[1];
			map[actualSegment].positive[2] = map[alreadyFoundSegment].positive[2];

		}
		else if (map[alreadyFoundSegment].negative[0] == 0 && map[alreadyFoundSegment].negative[1] == 0 && map[alreadyFoundSegment].negative[2]  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[alreadyFoundSegment].positive[0] = map[actualSegment].positive[0];
			map[alreadyFoundSegment].positive[1] = map[actualSegment].positive[1];
			map[alreadyFoundSegment].positive[2] = map[actualSegment].positive[2];
		}
	}

	// Change the redundant segments to the corrected.
	for (i = 0; i < MAZE_MAP_MAX_SEGEMENTS; i++)
	{
		if (actualSegment < alreadyFoundSegment)
		{
			if(map[i].negative[0] == alreadyFoundSegment)  map[i].negative[0] = actualSegment;
			if(map[i].negative[1] == alreadyFoundSegment)  map[i].negative[1] = actualSegment;
			if(map[i].negative[2] == alreadyFoundSegment)  map[i].negative[2] = actualSegment;
			if(map[i].positive[0] == alreadyFoundSegment)  map[i].positive[0] = actualSegment;
			if(map[i].positive[1] == alreadyFoundSegment)  map[i].positive[1] = actualSegment;
			if(map[i].positive[2] == alreadyFoundSegment)  map[i].positive[2] = actualSegment;
		}
		else
		{
			if(map[i].negative[0] == actualSegment)  map[i].negative[0] = alreadyFoundSegment;
			if(map[i].negative[1] == actualSegment)  map[i].negative[1] = alreadyFoundSegment;
			if(map[i].negative[2] == actualSegment)  map[i].negative[2] = alreadyFoundSegment;
			if(map[i].positive[0] == actualSegment)  map[i].positive[0] = alreadyFoundSegment;
			if(map[i].positive[1] == actualSegment)  map[i].positive[1] = alreadyFoundSegment;
			if(map[i].positive[2] == actualSegment)  map[i].positive[2] = alreadyFoundSegment;
		}
	}
}

static bool mazeAllSegmentsDiscovered (void)
{
	uint8_t i;
	bool temp = true;

	for ( i = 0;  i < MAZE_FINDABLE_SEGEMNST; i++)
	{
		temp &= segments[i];
	}

	return temp;
}

static void mazePlanExitRoute (void)
{
/*	uint8_t i;
	uint8_t j;
	uint8_t k;
	uint8_t l;
	uint8_t m;
	uint8_t n;
	uint8_t o;
	uint8_t p;
	uint8_t q;
	uint8_t r;
	uint8_t t;
	uint8_t v;
	bool pathFound = false;


	actualSegment = 1;
	//inclinSegment = 7;

	neighbourMatrixA[1][3] = 1;
	neighbourMatrixA[2][3] = 1;
	neighbourMatrixA[3][4] = 1;
	neighbourMatrixA[3][5] = 1;
	neighbourMatrixA[4][7] = 1;
	neighbourMatrixA[5][7] = 1;
	neighbourMatrixA[7][1] = 1;
	neighbourMatrixA[7][2] = 1;


	for (i = 1; i < MAZE_MAP_MAX_SEGEMENTS+1; ++i)
	{
		if (neighbourMatrixA[actualSegment][i] != 0 && pathFound == false)
		{
			if (i == inclinSegment)
			{
				exitRoute[0] = i;
				pathFound = true;
			}
			else
			{
				for (j = 1; j < MAZE_MAP_MAX_SEGEMENTS+1; ++j)
				{
					if (neighbourMatrixA[i][j] != 0 && pathFound == false)
					{
						if (j == inclinSegment)
						{
							exitRoute[0] = i; exitRoute[1] = j;
							pathFound = true;
						}
						else
						{
							for (k = 1; k < MAZE_MAP_MAX_SEGEMENTS+1; ++k)
							{
								if (neighbourMatrixA[j][k] != 0 && pathFound == false)
								{
									if (k == inclinSegment)
									{
										exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k;
										pathFound = true;
									}
									else
									{
										for (l = 1; l < MAZE_MAP_MAX_SEGEMENTS+1; ++l)
										{
											if (neighbourMatrixA[k][l] != 0 && pathFound == false)
											{
												if (l == inclinSegment)
												{
													exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
													pathFound = true;
												}
												else
												{
													for (m = 1; m < MAZE_MAP_MAX_SEGEMENTS+1; ++m)
													{
														if (neighbourMatrixA[l][m] != 0 && pathFound == false)
														{
															if (m == inclinSegment)
															{
																exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
																exitRoute[4] = m;
																pathFound = true;
															}
															else
															{
																for (n = 1; n < MAZE_MAP_MAX_SEGEMENTS+1; ++n)
																{
																	if (neighbourMatrixA[m][n] != 0 && pathFound == false)
																	{
																		if (n == inclinSegment)
																		{
																			exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
																			exitRoute[4] = m; exitRoute[5] = n;
																			pathFound = true;
																		}
																		else
																		{
																			for (o = 1; o < MAZE_MAP_MAX_SEGEMENTS+1; ++o)
																			{
																				if (neighbourMatrixA[n][o] != 0 && pathFound == false)
																				{
																					if (o == inclinSegment)
																					{
																						exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
																						exitRoute[4] = m; exitRoute[5] = n; exitRoute[6] = o;
																						pathFound = true;
																					}
																					else
																					{
																						for (p = 1; p < MAZE_MAP_MAX_SEGEMENTS+1; ++p)
																						{
																							if (neighbourMatrixA[o][p] != 0 && pathFound == false)
																							{
																								if (p == inclinSegment)
																								{
																									exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
																									exitRoute[4] = m; exitRoute[5] = n; exitRoute[6] = o; exitRoute[7] = p;
																									pathFound = true;
																								}
																								else
																								{
																									for (q = 1; q < MAZE_MAP_MAX_SEGEMENTS+1; ++q)
																									{
																										if (neighbourMatrixA[p][q] != 0 && pathFound == false)
																										{
																											if (q == inclinSegment)
																											{
																												exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
																												exitRoute[4] = m; exitRoute[5] = n; exitRoute[6] = o; exitRoute[7] = p;
																												exitRoute[8] = q;
																												pathFound = true;
																											}
																											else
																											{
																												for (r = 1; r < MAZE_MAP_MAX_SEGEMENTS+1; ++r)
																												{
																													if (neighbourMatrixA[q][r] != 0 && pathFound == false)
																													{
																														if (r == inclinSegment)
																														{
																															exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
																															exitRoute[4] = m; exitRoute[5] = n; exitRoute[6] = o; exitRoute[7] = p;
																															exitRoute[8] = q; exitRoute[9];
																															pathFound = true;
																														}
																														else
																														{
																															for (r = 1; r < MAZE_MAP_MAX_SEGEMENTS+1; ++r)
																															{
																																if (neighbourMatrixA[q][r] != 0 && pathFound == false)
																																{
																																	if (r == inclinSegment)
																																	{
																																		exitRoute[0] = i; exitRoute[1] = j; exitRoute[2] = k; exitRoute[3] = l;
																																		exitRoute[4] = m; exitRoute[5] = n; exitRoute[6] = o; exitRoute[7] = p;
																																		exitRoute[8] = q; exitRoute[9]; exitRoute[10];
																																		pathFound = true;
																																	}
																																	else
																																	{
																																		// NO PATH
																																	}
																																}
																															}
																														}
																													}
																												}
																											}
																										}
																									}
																								}
																							}
																						}
																					}
																				}
																			}
																		}
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}


	uint8_t plan[12];
	uint8_t wantedSeg;
	uint8_t preExitSeg[3];
	uint8_t secondSeg[3];
	uint8_t thirdSeg[3];
	uint8_t i;
	uint8_t j;
	uint8_t k;
	uint8_t l;
	uint8_t m;
	uint8_t n;
	uint8_t o;
	uint8_t p;

	wantedSeg = inclinSegment;

	secondSeg[0] = map[actualSegment].positive[0];
	secondSeg[1] = map[actualSegment].positive[1];
	secondSeg[2] = map[actualSegment].positive[2];

	plan[0] = actualSegment;

	if (inclinSegmentOrient == true)
	{
		preExitSeg[0] = map[inclinSegment].positive[0];
		preExitSeg[1] = map[inclinSegment].positive[1];
		preExitSeg[2] = map[inclinSegment].positive[2];
	}
	else
	{
		preExitSeg[0] = map[inclinSegment].negative[0];
		preExitSeg[1] = map[inclinSegment].negative[1];
		preExitSeg[2] = map[inclinSegment].negative[2];
	}


	if (secondSeg[0] != 0)
	{
		plan[1] = secondSeg[0];

		thirdSeg[0] = map[secondSeg].positive[0];
		thirdSeg[1] = map[secondSeg].positive[1];
		thirdSeg[2] = map[secondSeg].positive[2];

		if (thirdSeg[0] != 0)
		{

		}
	}


	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j) {
			for (k = 0; k < 3; ++k) {
				for (l = 0; l < 3; ++l) {
					for (m = 0; m < 3; ++m) {
						for (n = 0; n < 3; ++n) {
							for (o = 0; o < 3; ++o) {
								for (p = 0; p < 3; ++p) {
									plan[0] = actualSegment;
									plan[1] = map[i].positive[j];
									plan[1] = map[j].positive[j];
									plan[1] = map[k].positive[j];
									plan[1] = map[l].positive[j];
									plan[1] = map[i].positive[j];
									plan[1] = map[i].positive[j];
									plan[1] = map[i].positive[j];
								}
							}
						}
					}
				}
			}
		}
	}*/
}


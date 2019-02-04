////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze_StateMachine.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_maze_StateMachine.h"
#include "app_roadsignal.h"
#include "line.h"
#include "bsp_servo.h"
#include <string.h>

// Defines -------------------------------------------------------------------------------------------------------------

#define MAZE_MAP_MAX_SEGEMENTS	(20u)	//!< The map can contain this much segments.
#define MAZE_FINDABLE_SEGEMNST	(12u)	//!< The labyrinth has this much segments that are need to be discovered.
#define MAZE_N_ACCURACY			(10u)	//!< Two N values are the same if they differ this much.

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Flag that indicates if the maze task is finished.
bool mazeFinished;

//! This variable indicates the actual state of the main state machine of the maze algorithm.
eSTATE_MAIN smMainState;
//! The graph map of the labyrinth.
cSEGMENT map[MAZE_MAP_MAX_SEGEMENTS];
//! A list of the discoverable segments. A bit is set when the segment was found and the car has driven it through.
bool segments[MAZE_FINDABLE_SEGEMNST];
//! The number of the segment where the exit point is to be found.
uint32_t inclinSegment;
uint8_t inclinSegmentOrient;	// 0 = negative, 1 = positive
bool inclinDirection;			// left = false, rigth = true

uint32_t actualSegment;
uint32_t nextNewSegmentIndex;
uint32_t alreadyFoundSegment;
bool turnOffLineFollow;

//! The controller parameters of the actual state in which the car is currently.
cPD_CONTROLLER_PARAMS actualParams;
//! List of the controller parameters for all of the main states.
cMAZE_PD_CONTROL_PARAM_LIST paramList;

static cNAVI_STATE naviStateCrossing;
static cNAVI_STATE naviStateCar;

static uint8_t neighbourMatrixA [MAZE_MAP_MAX_SEGEMENTS][MAZE_MAP_MAX_SEGEMENTS];
static uint8_t neighbourMatrixB [MAZE_MAP_MAX_SEGEMENTS][MAZE_MAP_MAX_SEGEMENTS];

static uint8_t exitRoute[20];

extern QueueHandle_t qNaviN_f;
extern QueueHandle_t qNaviE_f;
extern QueueHandle_t qNaviPSI_f;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void mazeStateMachineDiscovery   (void);
static void mazeStateMachineInclination (void);

static bool mazeCrossingAlreadyFound 	(const cNAVI_STATE crossingNaviState);
static void mazeUpdateMap 				(const RoadSignal crossingType);
static void mazeCheckDiscoveredSegments (void);
static void mazeMergeSegments 			(void);

static bool mazeAllSegmentsDiscovered	(void);
static void mazePlanExitRoute			(void);
static void mazeFollowRoute 			(void);

// Global function definitions -----------------------------------------------------------------------------------------

void MazeStateMachinesInit (void)
{
	uint8_t i;

	mazeFinished = false;
	smMainState  = eSTATE_MAIN_READY;

	for (i = 0; i < MAZE_MAP_MAX_SEGEMENTS; i++)
	{
		map[i].end.p.n = 0;
		map[i].end.p.e = 0;
		map[i].end.psi = 0;
		map[i].start.p.n = 0;
		map[i].start.p.e = 0;
		map[i].start.psi = 0;
		map[i].negative[0]   = 0;
		map[i].negative[1] = 0;
		map[i].negative[2]  = 0;
		map[i].positive[0]   = 0;
		map[i].positive[1] = 0;
		map[i].positive[2]  = 0;
	}

	// Reset trackers.
	memset(segments, 0, MAZE_FINDABLE_SEGEMNST);

	inclinSegment = 0;
	actualSegment = 0;
	nextNewSegmentIndex = 1;
	alreadyFoundSegment = 0;

	turnOffLineFollow = false;
}

//! Function: MazeMainStateMachine
void MazeMainStateMachine (void)
{
	switch (smMainState)
	{
		case eSTATE_MAIN_READY:
		{
			// Standing in the start position and radio trigger.
			actualParams.Speed = 0;

			// TODO on race use this! : if (startGetState() == s0)
			if (true)
			{
				// Trigger received -> DISCOVER state.
				smMainState = eSTATE_MAIN_DISCOVER;
				actualSegment = 1;
				nextNewSegmentIndex = 2;
			}
			break;
		}
		case eSTATE_MAIN_DISCOVER:
		{
			// Load in the control parameters.
			actualParams.Kp = paramList.discover.Kp;
			actualParams.Kd = paramList.discover.Kd;
			actualParams.Speed = paramList.discover.Speed;

			// Map making, navigation, path tracking.
			//TODO implement
			if (mazeFinished != true)
			{
				mazeStateMachineDiscovery();
			}
			else
			{
				// All of the segments are discovered and reached -> INCLINATION state.
				smMainState = eSTATE_MAIN_INCLINATION;
			}
			break;
		}
		case eSTATE_MAIN_INCLINATION:
		{
			// Load in control parameters.
			actualParams.Kp = paramList.inclination.Kp;
			actualParams.Kd = paramList.inclination.Kd;
			actualParams.Speed = paramList.inclination.Speed;

			mazeStateMachineInclination();

			break;
		}
		case eSTATE_MAIN_OUT:
		{
			vTaskDelay(2000);

			// Stop/Park behind the safety-car.
			actualParams.Speed = 0;

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

// Local (static) function definitions ---------------------------------------------------------------------------------

static void mazeStateMachineDiscovery (void)
{
	CROSSING_TYPE currCross;

	//  Ask the line sensor for the actual road sign.
	currCross = getCrossingType();

	// Select the appropriate reaction.
	if (/* cross roads */ true)	// TODO
	{
		// Get navigation status.
		xQueuePeek(qNaviN_f,   &naviStateCar.p.n, 0);
		xQueuePeek(qNaviE_f,   &naviStateCar.p.e, 0);
		xQueuePeek(qNaviPSI_f, &naviStateCar.psi, 0);

		// Check if the crossing was already found.
		if (mazeCrossingAlreadyFound(naviStateCrossing) == false)
		{
			// 2 new segments found.
			mazeUpdateMap(currRoad);

			// Follow one direction.
			if (map[actualSegment].positive[0] != 0)
			{
				// Turn left. TODO


				actualSegment = map[actualSegment].positive[0];
			}
			else if (map[actualSegment].positive[1] != 0)
			{
				// Turn middle. TODO

				actualSegment = map[actualSegment].positive[1];
			}
			else if (map[actualSegment].positive[2] != 0)
			{
				// Turn right. TODO

				actualSegment = map[actualSegment].positive[2];
			}
		}
		else
		{
			// Turn to an undiscovered direction.
			if (map[actualSegment].positive[0] != alreadyFoundSegment && map[actualSegment].positive[0] != 0)
			{
				// Turn left. TODO

				actualSegment = map[actualSegment].positive[0];
			}
			else if (map[actualSegment].positive[1] != alreadyFoundSegment && map[actualSegment].positive[1] != 0)
			{
				// Turn middle. TODO

				actualSegment = map[actualSegment].positive[1];
			}
			else if (map[actualSegment].positive[2] != alreadyFoundSegment && map[actualSegment].positive[2] != 0)
			{
				// Turn right. TODO

				actualSegment = map[actualSegment].positive[2];
			}
			else
			{
				// Turn random. TODO
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
	else if (/* exit */ false)
	{
		inclinSegment = actualSegment;

		if (/* inclin direction ok == */ false)
		{
			inclinSegmentOrient = 1;
		}
		else
		{
			inclinSegmentOrient = 0;
		}

		// inclinDirection =  get
	}
	else
	{
		// Follow the main line.
	}

}

static void mazeStateMachineInclination (void)
{
	CROSSING_TYPE crossing;

	crossing = getCrossingType();

	//____________________________________________STEP 1________________________________________________
	if (actualSegment != inclinSegment)	// TODO check direction
	{
		// Plan a path to the exit
		mazePlanExitRoute();

		// Drive to the exit segment
		mazeFollowRoute();
	}
	else
	{
		//____________________________________________STEP 2________________________________________________
		// At the exit find the markings and slow down.
		if (crossing == CrossingAtoLB)		// TODO
		{
			// Steer in the direction if the markings until the car leaves the lines (45deg).
			if (inclinDirection == false)
			{
				servoSetAngle(PI/180*150);
			}
			else
			{
				servoSetAngle(PI/180*60);
			}

			// Wait to leave the line.
			if (/*no line detected*/ false)
			{
				turnOffLineFollow = false;
			}
			else
			{
				turnOffLineFollow = true;
			}

			// Check the distance sensor for collision and go until the new line is found. If collision warning,
			// then stop.

			// New lines found -> OUT state.
			if (/* vonal */ false)
			{
				smMainState = eSTATE_MAIN_OUT;
			}
		}
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

static void mazeUpdateMap (const RoadSignal crossingType)
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
		case 0:	// TODO
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
		case 1: // TODO
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
		case 2: // TODO
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
		case 3: // TODO
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
		case 4: // TODO
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
		case 5:	//TODO
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
	/uint8_t plan[12];
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
	}
}

static void mazeFollowRoute (void)
{

}

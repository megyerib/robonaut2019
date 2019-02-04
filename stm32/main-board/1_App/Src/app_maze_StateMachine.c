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
uint8_t inclinSegmentStart;	// 0 = negative, 1 = positive
bool inclinDirection;		// left = false, rigth = true

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
		map[i].negative.left   = 0;
		map[i].negative.middle = 0;
		map[i].negative.right  = 0;
		map[i].positive.left   = 0;
		map[i].positive.middle = 0;
		map[i].positive.right  = 0;
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
	RoadSignal currRoad;

	//  Ask the line sensor for the actual road sign.
	currRoad = lineGetRoadSignal();

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
			if (map[actualSegment].positive.left != 0)
			{
				// Turn left. TODO


				actualSegment = map[actualSegment].positive.left;
			}
			else if (map[actualSegment].positive.middle != 0)
			{
				// Turn middle. TODO

				actualSegment = map[actualSegment].positive.middle;
			}
			else if (map[actualSegment].positive.right != 0)
			{
				// Turn right. TODO

				actualSegment = map[actualSegment].positive.right;
			}
		}
		else
		{
			// Turn to an undiscovered direction.
			if (map[actualSegment].positive.left != alreadyFoundSegment && map[actualSegment].positive.left != 0)
			{
				// Turn left. TODO

				actualSegment = map[actualSegment].positive.left;
			}
			else if (map[actualSegment].positive.middle != alreadyFoundSegment && map[actualSegment].positive.middle != 0)
			{
				// Turn middle. TODO

				actualSegment = map[actualSegment].positive.middle;
			}
			else if (map[actualSegment].positive.right != alreadyFoundSegment && map[actualSegment].positive.right != 0)
			{
				// Turn right. TODO

				actualSegment = map[actualSegment].positive.right;
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
			inclinSegmentStart = 1;
		}
		else
		{
			inclinSegmentStart = 0;
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
			map[actualSegment].positive.left = nextNewSegmentIndex + 1;
			map[actualSegment].positive.middle = nextNewSegmentIndex;
			map[actualSegment].positive.right = 0;

			// New segment B.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = 0;
			map[nextNewSegmentIndex].negative.middle = actualSegment;
			map[nextNewSegmentIndex].negative.right  = 0;
			nextNewSegmentIndex++;

			// New segment CL.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = 0;
			map[nextNewSegmentIndex].negative.middle = actualSegment;
			map[nextNewSegmentIndex].negative.right  = 0;
			nextNewSegmentIndex++;

			break;
		}
		case 1: // TODO
		{
			// From A to B and CR direction.

			// Directions in the crossing from A.
			map[actualSegment].positive.left = 0;
			map[actualSegment].positive.middle = nextNewSegmentIndex;
			map[actualSegment].positive.right = nextNewSegmentIndex + 1;

			// New segment B
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = 0;
			map[nextNewSegmentIndex].negative.middle = actualSegment;
			map[nextNewSegmentIndex].negative.right  = 0;
			nextNewSegmentIndex++;

			// New segment CR
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = 0;
			map[nextNewSegmentIndex].negative.middle = actualSegment;
			map[nextNewSegmentIndex].negative.right  = 0;
			nextNewSegmentIndex++;
			break;
		}
		case 2: // TODO
		{
			// From B to A and CL direction.

			// Directions in the crossing from B.
			map[actualSegment].positive.left = 0;
			map[actualSegment].positive.middle = nextNewSegmentIndex + 1;
			map[actualSegment].positive.right = 0;

			// New segment CL.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive.left 	 = 0;
			map[nextNewSegmentIndex].positive.middle = nextNewSegmentIndex + 1;
			map[nextNewSegmentIndex].positive.right  = 0;
			nextNewSegmentIndex++;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].negative.middle = actualSegment;
			map[nextNewSegmentIndex].negative.right  = 0;
			nextNewSegmentIndex++;
			break;
		}
		case 3: // TODO
		{
			// From B to A and CR direction.

			// Directions in the crossing from B.
			map[actualSegment].positive.left = 0;
			map[actualSegment].positive.middle = nextNewSegmentIndex;
			map[actualSegment].positive.right = 0;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = 0;
			map[nextNewSegmentIndex].negative.middle = actualSegment;
			map[nextNewSegmentIndex].negative.right  = nextNewSegmentIndex + 1;
			nextNewSegmentIndex++;

			// New segment CR.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive.left 	 = 0;
			map[nextNewSegmentIndex].positive.middle = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].positive.right  = 0;
			nextNewSegmentIndex++;
			break;
		}
		case 4: // TODO
		{
			// From CL to A.

			// Direction from CL in the crossing.
			map[actualSegment].positive.left = 0;
			map[actualSegment].positive.middle = nextNewSegmentIndex;
			map[actualSegment].positive.right = 0;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = actualSegment;
			map[nextNewSegmentIndex].negative.middle = nextNewSegmentIndex + 1;
			map[nextNewSegmentIndex].negative.right  = 0;
			nextNewSegmentIndex++;

			// New segment B.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive.left 	 = 0;
			map[nextNewSegmentIndex].positive.middle = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].positive.right  = 0;
			nextNewSegmentIndex++;
			break;
		}
		case 5:	//TODO
		{
			// From CR to A.

			// Direction from CR in the crossing.
			map[actualSegment].positive.left = 0;
			map[actualSegment].positive.middle = nextNewSegmentIndex;
			map[actualSegment].positive.right = 0;

			// New segment A.
			map[nextNewSegmentIndex].start = naviStateCrossing;
			map[nextNewSegmentIndex].negative.left 	 = 0;
			map[nextNewSegmentIndex].negative.middle = nextNewSegmentIndex + 1;
			map[nextNewSegmentIndex].negative.right  = actualSegment;
			nextNewSegmentIndex++;

			// New segment B.
			map[nextNewSegmentIndex].end = naviStateCrossing;
			map[nextNewSegmentIndex].positive.left 	 = 0;
			map[nextNewSegmentIndex].positive.middle = nextNewSegmentIndex - 1;
			map[nextNewSegmentIndex].positive.right  = 0;
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

	if (   map[actualSegment].positive.left   == map[alreadyFoundSegment].positive.left
		&& map[actualSegment].positive.middle == map[alreadyFoundSegment].positive.middle
		&& map[actualSegment].positive.right  == map[alreadyFoundSegment].positive.right
		)
	{
		// The turning directions in the same orientation are the same
		if (map[actualSegment].negative.left == 0 && map[actualSegment].negative.middle == 0 && map[actualSegment].negative.right  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[actualSegment].negative.left   = map[alreadyFoundSegment].negative.left;
			map[actualSegment].negative.middle = map[alreadyFoundSegment].negative.middle;
			map[actualSegment].negative.right  = map[alreadyFoundSegment].negative.right;
		}
		else if (map[alreadyFoundSegment].negative.left == 0 && map[alreadyFoundSegment].negative.middle == 0 && map[alreadyFoundSegment].negative.right  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[alreadyFoundSegment].negative.left   = map[actualSegment].negative.left;
			map[alreadyFoundSegment].negative.middle = map[actualSegment].negative.middle;
			map[alreadyFoundSegment].negative.right  = map[actualSegment].negative.right;
		}
	}
	else if (   map[actualSegment].positive.left   == map[alreadyFoundSegment].negative.left
			 && map[actualSegment].positive.middle == map[alreadyFoundSegment].negative.middle
			 && map[actualSegment].positive.right  == map[alreadyFoundSegment].negative.right
			)
	{
		//
		if (map[actualSegment].negative.left == 0 && map[actualSegment].negative.middle == 0 && map[actualSegment].negative.right  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[actualSegment].negative.left   = map[alreadyFoundSegment].positive.left;
			map[actualSegment].negative.middle = map[alreadyFoundSegment].positive.middle;
			map[actualSegment].negative.right  = map[alreadyFoundSegment].positive.right;

		}
		else if (map[alreadyFoundSegment].negative.left == 0 && map[alreadyFoundSegment].negative.middle == 0 && map[alreadyFoundSegment].negative.right  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[alreadyFoundSegment].positive.left   = map[actualSegment].negative.left;
			map[alreadyFoundSegment].positive.middle = map[actualSegment].negative.middle;
			map[alreadyFoundSegment].positive.right  = map[actualSegment].negative.right;

		}
	}
	else if (   map[actualSegment].negative.left   == map[alreadyFoundSegment].negative.left
			 && map[actualSegment].negative.middle == map[alreadyFoundSegment].negative.middle
			 && map[actualSegment].negative.right  == map[alreadyFoundSegment].negative.right
			)
	{
		// The turning directions in the same orientation are the same

		if (map[actualSegment].negative.left == 0 && map[actualSegment].negative.middle == 0 && map[actualSegment].negative.right  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[actualSegment].positive.left   = map[alreadyFoundSegment].positive.left;
			map[actualSegment].positive.middle = map[alreadyFoundSegment].positive.middle;
			map[actualSegment].positive.right  = map[alreadyFoundSegment].positive.right;

		}
		else if (map[alreadyFoundSegment].negative.left == 0 && map[alreadyFoundSegment].negative.middle == 0 && map[alreadyFoundSegment].negative.right  == 0)
		{
			// The actual segment turning directions are incomplete.
			map[alreadyFoundSegment].positive.left   = map[actualSegment].positive.left;
			map[alreadyFoundSegment].positive.middle = map[actualSegment].positive.middle;
			map[alreadyFoundSegment].positive.right  = map[actualSegment].positive.right;
		}
	}

	// Change the redundant segments to the corrected.
	for (i = 0; i < MAZE_MAP_MAX_SEGEMENTS; i++)
	{
		if (actualSegment < alreadyFoundSegment)
		{
			if(map[i].negative.left   == alreadyFoundSegment)  map[i].negative.left   = actualSegment;
			if(map[i].negative.middle == alreadyFoundSegment)  map[i].negative.middle = actualSegment;
			if(map[i].negative.right  == alreadyFoundSegment)  map[i].negative.right  = actualSegment;
			if(map[i].positive.left   == alreadyFoundSegment)  map[i].positive.left   = actualSegment;
			if(map[i].positive.middle == alreadyFoundSegment)  map[i].positive.middle = actualSegment;
			if(map[i].positive.right  == alreadyFoundSegment)  map[i].positive.right  = actualSegment;
		}
		else
		{
			if(map[i].negative.left   == actualSegment)  map[i].negative.left   = alreadyFoundSegment;
			if(map[i].negative.middle == actualSegment)  map[i].negative.middle = alreadyFoundSegment;
			if(map[i].negative.right  == actualSegment)  map[i].negative.right  = alreadyFoundSegment;
			if(map[i].positive.left   == actualSegment)  map[i].positive.left   = alreadyFoundSegment;
			if(map[i].positive.middle == actualSegment)  map[i].positive.middle = alreadyFoundSegment;
			if(map[i].positive.right  == actualSegment)  map[i].positive.right  = alreadyFoundSegment;
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
	uint8_t i;
	for (i = 0; i < MAZE_MAP_MAX_SEGEMENTS; i++)
	{

	}
}

static void mazeFollowRoute (void)
{

}

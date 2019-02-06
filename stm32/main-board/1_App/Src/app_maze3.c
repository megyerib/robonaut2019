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
#include "app_roadsignal.h"
#include "navigation.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define JUNCTION_MAX_NUM (16u)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	start = 0,

	first_follow,
	first_crossing,

	discovery_follow,
	discovery_crossing,
	discovery_exit,

	travel_plan,
	travel_choose,
	travel_follow,
	travel_crossing,

	exit_plan,
	exit_choose,
	exit_follow,
	exit_crossing,

	finish
}
MAZE_STATE;

typedef enum
{
	left = 0,
	right
}
DIR;

typedef enum
{
	source,
	leftTurn,
	rightTurn,

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

// Local (static) & extern variables -----------------------------------------------------------------------------------

static MAZE_STATE mazeState = start;
static JUNCTION junctions[JUNCTION_MAX_NUM];
static int junctionNum = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void lineFollow();
static int isCrossing(CROSSING_TYPE c);
static int isExit(CROSSING_TYPE c);
static int newCrossing(cNAVI_STATE nav);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Maze3 (void)
{

}

void Task_Maze3 (void* p)
{
	int skip;
	int crossing;

	while (1)
	{
		skip = 0;
		crossing = getCrossingType();

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
					mazeState = first_follow;
				}

				break;
			}
			case first_follow:
			{
				lineFollow();

				if (isCrossing(crossing))
				{
					mazeState = first_crossing;
				}
				else
				{
					skip = 1;
				}

				break;
			}
			case first_crossing: // TODO
			{
				break;
			}
			case discovery_follow:
			{
				lineFollow();

				if (isCrossing(crossing))
				{
					mazeState = discovery_crossing;
				}
				else if (isExit(crossing))
				{
					mazeState = discovery_exit;
				}
				else
				{
					skip = 1;
				}

				break;
			}
			case discovery_crossing: // TODO
			{
				break;
			}
			case discovery_exit: // TODO
			{
				break;
			}
			case travel_plan: // TODO
			{
				break;
			}
			case travel_choose: // TODO
			{
				break;
			}
			case travel_follow:
			{
				lineFollow();

				if (isCrossing(crossing))
				{
					mazeState = travel_crossing;
				}
				else
				{
					skip = 1;
				}

				break;
			}
			case travel_crossing: // TODO
			{
				break;
			}
			case exit_plan: // TODO
			{
				break;
			}
			case exit_choose: // TODO
			{
				break;
			}
			case exit_follow:
			{
				lineFollow();

				if (isCrossing(crossing))
				{
					mazeState = exit_crossing;
				}

				break;
			}
			case exit_crossing: // TODO
			{
				break;
			}
			case finish: // TODO
			{
				break;
			}
			}
		}

		vTaskDelay(5);
	}
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

static int isExit(CROSSING_TYPE c)
{
	return (
		c == ExitForwardRight  ||
		c == ExitForwardLeft   ||
		c == ExitBackwardRight ||
		c == ExitBackwardLeft
	);
}

static int newCrossing(cNAVI_STATE nav)
{
	JUNCTION* j = &junctions[junctionNum];
	int ret = junctionNum;

	j->nav = nav;

	j->visited[source]    = 0;
	j->visited[leftTurn]  = 0;
	j->visited[rightTurn] = 0;

	junctionNum++;

	return ret;
}

// END -----------------------------------------------------------------------------------------------------------------

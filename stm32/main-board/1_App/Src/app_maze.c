////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze.c
//!  \brief		Contains the logic, how to discover and get out of the labyrinth.
//!  \details	See in maze.h.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_maze.h"
#include "app_common.h"
#include "navigation.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	eSTATE_MAIN_READY       = 0,	//! Start position and car waits for a trigger.
	eSTATE_MAIN_DISCOVER,			//! The car is driving through the maze, finding crossings and segments.
	eSTATE_MAIN_INCLINATION,		//! The car has discovered the maze, it has to leave it now.
	eSTATE_MAIN_OUT					//! The car is out of the maze
} eSTATE_MAIN;

typedef struct
{
	cNAVI_STATE start;
	cNAVI_STATE end;
	struct cSEGMENT* left;
	struct cSEGMENT* midle;
	struct cSEGMENT* right;
} cSEGMENT_INFO;

typedef struct
{
	cSEGMENT_INFO alfa;
	cSEGMENT_INFO beta;
	bool dir;
} cSEGMENT;

// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Flag that indicates if the maze task is finished.
static bool mazeFinished;

//! This variable indicates the actual state of the main state machine of the maze algorithm.
static eSTATE_MAIN smMainState;

//! The graph map of the labyrinth.
static cSEGMENT map[20];


// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Maze (void)
{
	mazeFinished = false;

	smMainState = eSTATE_MAIN_READY;

	xTaskCreate(Task_Maze,
				"TASK_MAZE",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_MAZE_PRIO,
				NULL);
}

void Task_Maze (void* p)
{
	(void)p;

	while (1)
	{
		//__________________________________________________STATE MACHINCE______________________________________________

		// Run the state machine until the job is done.
		if (mazeFinished == false)
		{
			switch (smMainState)
			{
				case eSTATE_MAIN_READY:
				{
					// Standing in the start position and radio trigger.

					// Trigger received -> DISCOVER state.
					smMainState = eSTATE_MAIN_DISCOVER;
					break;
				}
				case eSTATE_MAIN_DISCOVER:
				{
					// Map making, navigation, path tracking.

					// All of the segments are discovered and reached -> INCLINATION state.
					smMainState = eSTATE_MAIN_INCLINATION;
					break;
				}
				case eSTATE_MAIN_INCLINATION:
				{
					//____________________________________________STEP 1________________________________________________
					// Plan a path to the exit

					// Drive to the exit

					//____________________________________________STEP 2________________________________________________
					// At the exit find the markings and slow down.

					// Steer in the direction if the markings until the car leaves the lines (45deg).

					// Check the distance sensor for collision and go until the new line is found. If collision warning,
					// then stop.

					// New lines found -> OUT state.
					smMainState = eSTATE_MAIN_OUT;
					break;
				}
				case eSTATE_MAIN_OUT:
				{
					// Stop/Park behind the safety-car.

					// Maze task is finished.
					mazeFinished = true;
					break;
				}
				default:
				{
					// ERROR: Not valid state. Stop!

					break;
				}
			}
		}

		//_____________________________________________RESET SPEED RUN BUTTON___________________________________________

		// Check the reset (skip maze run) signal. TODO
		//if ( event bit set )
		//{
		//	// Reset signal received. Skip the maze and signal to the speed run state machine.
		//	mazeFinished = true;
		//}

		vTaskDelay(TASK_DELAY_5_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------
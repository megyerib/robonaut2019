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
#include "trace.h"

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
	cPD_CONTROLLER_PARAMS discover;
	cPD_CONTROLLER_PARAMS inclination;
} cMAZE_PD_CONTROL_PARAM_LIST;

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

//TODO comment
static bool segements[12];

//TODO comment
static uint32_t inclinSegment;

// TODO comment
static cPD_CONTROLLER_PARAMS actualParams;
static cMAZE_PD_CONTROL_PARAM_LIST paramList;

// TODO comment
static cTRACE_RX_DATA rxData;
static bool 	recMainSMReset;
static uint32_t recMainSMResetTo;
static uint32_t recGetState;
static uint32_t recSetState;
static float	recSetKp;
static float	recSetKd;
static uint32_t recSetSpeed;

// TODO comment
static uint32_t txMainSM;
static float 	txGetKp;
static float 	txGetKd;
static float 	txGetSpeed;
static uint32_t txSegments;
static uint32_t txActState;
static float    txActKp;
static float    txActKd;
static uint32_t txActSpeed;
static uint32_t txInclinSegment;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void 	ProcessReceivedCommand (void);
static void 	TraceMazeInformations  (void);
static uint32_t MazeSegmentsConverter  (void);

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
		//___________________________________________RECEIVE/PROCESS PARAMETERS_________________________________________
		ProcessReceivedCommand();

		//__________________________________________________STATE MACHINE_______________________________________________

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

		//_______________________________________RESET BUTTON / CHANGE TO SPEED RUN_____________________________________

		// Check the reset (skip maze run) signal. TODO
		//if ( event bit set )
		//{
		//	// Reset signal received. Skip the maze and signal to the speed run state machine.
		//	mazeFinished = true;
		//}

		//_____________________________________________________TRACE____________________________________________________
		TraceMazeInformations();

		vTaskDelay(TASK_DELAY_5_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void ProcessReceivedCommand (void)
{
	// Get the received data.
	rxData = traceGetRxData();

	// Separate the parameters.
	recMainSMReset 	 = rxData.MazeMainSMReset;
	recMainSMResetTo = rxData.MazeMainSMResetTo;
	recGetState 	 = rxData.MazeGetState;
	recSetState 	 = rxData.MazeSetState;
	recSetKp 		 = rxData.MazeSetKp;
	recSetKd 		 = rxData.MazeSetKd;
	recSetSpeed  	 = rxData.MazeSetSpeed;

	// Reset the main state machine and reset flag. _______
	if (recMainSMReset == true)
	{
		smMainState = recMainSMResetTo;
		recMainSMReset = false;
	}

	// Return the requested parameters. ___________________
	if (recGetState == eSTATE_MAIN_DISCOVER)
	{
		txGetKp    = paramList.discover.Kp;
		txGetKd    = paramList.discover.Kd;
		txGetSpeed = paramList.discover.Speed;
	}
	else if (recGetState == eSTATE_MAIN_INCLINATION)
	{
		txGetKp    = paramList.inclination.Kp;
		txGetKd    = paramList.inclination.Kd;
		txGetSpeed = paramList.inclination.Speed;
	}
	else
	{
		txGetKp    = 0.0f;
		txGetKd    = 0.0f;
		txGetSpeed = 0;
	}

	// Set the received parameters. ______________________
	if (recSetState == eSTATE_MAIN_DISCOVER)
	{
		paramList.discover.Kp    = recSetKp;
		paramList.discover.Kd    = recSetKd;
		paramList.discover.Speed = recSetSpeed;
	}
	else if (recGetState == eSTATE_MAIN_INCLINATION)
	{
		paramList.inclination.Kp    = recSetKp;
		paramList.inclination.Kd    = recSetKd;
		paramList.inclination.Speed = recSetSpeed;
	}
	else
	{
		//NOP
	}
}

static void TraceMazeInformations  (void)
{
	txMainSM        = smMainState;
	txSegments      = MazeSegmentsConverter();
	txActState      = smMainState;
	txActKp         = actualParams.Kp;
	txActKd         = actualParams.Kd;
	txActSpeed      = actualParams.Speed;
	txInclinSegment = inclinSegment;

	traceBluetooth(BT_LOG_MAZE_MAIN_SM,        &txMainSM);
	traceBluetooth(BT_LOG_MAZE_GET_KP,         &txGetKp);
	traceBluetooth(BT_LOG_MAZE_GET_KD,         &txGetKd);
	traceBluetooth(BT_LOG_MAZE_GET_SPEED,      &txGetSpeed);
	traceBluetooth(BT_LOG_MAZE_SEGMENTS, 	   &txSegments);
	traceBluetooth(BT_LOG_MAZE_ACT_STATE, 	   &txActState);
	traceBluetooth(BT_LOG_MAZE_ACT_KP, 		   &txActKp);
	traceBluetooth(BT_LOG_MAZE_ACT_KD, 		   &txActKd);
	traceBluetooth(BT_LOG_MAZE_ACT_SPEED, 	   &txActSpeed);
	traceBluetooth(BT_LOG_MAZE_INCLIN_SEGMENT, &txInclinSegment);
}

static uint32_t MazeSegmentsConverter  (void)
{
	uint32_t retVal = 0;

	if (segements[0] == true)	retVal += 2048;
	if (segements[1] == true)	retVal += 1024;
	if (segements[2] == true)	retVal += 512;
	if (segements[3] == true)	retVal += 256;
	if (segements[4] == true)	retVal += 128;
	if (segements[5] == true)	retVal += 64;
	if (segements[6] == true)	retVal += 32;
	if (segements[7] == true)	retVal += 16;
	if (segements[8] == true)	retVal += 8;
	if (segements[9] == true)	retVal += 4;
	if (segements[10] == true)	retVal += 2;
	if (segements[11] == true)	retVal += 1;

	return retVal;
}

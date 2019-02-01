////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze.c
//!  \brief		Contains the logic, how to discover and get out of the labyrinth.
//!  \details	See in maze.h.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "event_groups.h"
#include "app_maze.h"
#include "app_common.h"
#include "navigation.h"
#include "trace.h"
#include "remote.h"
#include "main.h"
#include "motor.h"
#include "line.h"
#include "bsp_servo.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//! States of the main state machine of the maze algorithm.
//**********************************************************************************************************************
typedef enum
{
	eSTATE_MAIN_READY       = 0,		//!< Start position and car waits for a trigger.
	eSTATE_MAIN_DISCOVER,				//!< The car is driving through the maze, finding crossings and segments.
	eSTATE_MAIN_INCLINATION,			//!< The car has discovered the maze, it has to leave it now.
	eSTATE_MAIN_OUT						//!< The car is out of the maze
} eSTATE_MAIN;

//**********************************************************************************************************************
//! List that contain all of the control parameters.
//**********************************************************************************************************************
typedef struct
{
	cPD_CONTROLLER_PARAMS discover;		//!< Contains the control parameters of the Discover state.
	cPD_CONTROLLER_PARAMS inclination;	//!< Contains the control parameters of the Inclination state.
} cMAZE_PD_CONTROL_PARAM_LIST;

//**********************************************************************************************************************
//!
//**********************************************************************************************************************
typedef struct
{
	cNAVI_STATE start;
	cNAVI_STATE end;
	struct cSEGMENT* left;
	struct cSEGMENT* midle;
	struct cSEGMENT* right;
} cSEGMENT_INFO;

//**********************************************************************************************************************
//!
//**********************************************************************************************************************
typedef struct
{
	cSEGMENT_INFO alfa;
	cSEGMENT_INFO beta;
	bool dir;
} cSEGMENT;

// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Event flag (first bit) that indicates if we have left the maze.
EventGroupHandle_t event_MazeOut;
//! Flag that indicates if the maze task is finished.
static bool mazeFinished;

//! This variable indicates the actual state of the main state machine of the maze algorithm.
static eSTATE_MAIN smMainState;
//! The graph map of the labyrinth.
static cSEGMENT map[20];
//! A list of the discoverable segments. A bit is set when the segment was found and the car has driven it through.
static bool segements[12];
//! The number of the segment where the exit point is to be found.
static uint32_t inclinSegment;

//! The controller parameters of the actual state in which the car is currently.
static cPD_CONTROLLER_PARAMS actualParams;
//! List of the controller parameters for all of the main states.
static cMAZE_PD_CONTROL_PARAM_LIST paramList;

//! Structure that contain the received serial data.
static cTRACE_RX_DATA rxData;
//! Flag that indicates if the car must be stopped.
static bool	recStopCar;
//! Flag that indicates if the main state machine must be reset.
static bool recMainSMReset;
//! Holds the state into which the main state machine must be reset.
static uint32_t recMainSMResetTo;
//! Request for the control parameters of a valid state.
static uint32_t recGetState;
//! A state that's parameters has to be changed.
static uint32_t recSetState;
//! New Kp parameter for the selected state.
static float recSetKp;
//! New Kd parameter for the selected state.
static float recSetKd;
//! New Speed parameter for the selected state.
static uint32_t recSetSpeed;

//! Holds the actual state of the main state machine.
static uint32_t txMainSM;
//! Holds the Kp parameter of the requested state.
static float txGetKp;
//! Holds the Kd parameter of the requested state.
static float txGetKd;
//! Holds the Speed parameter of the requested state.
static uint32_t	txGetSpeed;
//! Contains the information about the segments if they are discovered or not.
static uint32_t txSegments;
//! Holds the actual state of the car.
static uint32_t txActState;
//! Holds the Kp parameter of the actual state.
static float txActKp;
//! Holds the Kd parameter of the actual state.
static float txActKd;
//! Holds the Speed parameter of the actual state.
static uint32_t txActSpeed;
//! Hold the number of the segment on which the exit point is present.
static uint32_t txInclinSegment;

static float txSteerWheelAngle;
static float txServoAngle;
static uint8_t txLineNumber;
static float txLineMainLinePos;
static float txLineSecLinePos;

//! Position of the main line in the previous task period.
static float line_prevPos;
//! Position of the main line in the current task period.
static float line_pos;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void 	MazeMainStateMachine   (void);
static void 	MazeProcessRecCommands (void);
static void 	MazeCheckRemote		   (void);
static void 	MazeTraceInformations  (void);
static uint32_t MazeSegmentsConverter  (void);
static void		MazeCntrLineFollow	   (void);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Maze (void)
{
	event_MazeOut = xEventGroupCreate();
	xEventGroupClearBits(event_MazeOut, 1);

	mazeFinished = false;

	// Start in the Ready state.
	smMainState = eSTATE_MAIN_READY;

	// Reset trackers.
	memset(segements, 0, 12);
	inclinSegment = 0;

	// Initial parameters of the Discover state.
	paramList.discover.Kp	 = 1;
	paramList.discover.Kd	 = 1;
	paramList.discover.Speed = 0;

	// Initial parameters of the Discover state.
	paramList.inclination.Kp	 = 1;
	paramList.inclination.Kd	 = 1;
	paramList.inclination.Speed  = 0;

	// Initial parameters for the line follower controller.
	line_pos 	 = 0;
	line_prevPos = 0;

	// Task can be created now.
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
		// Process the received parameters.
		MazeProcessRecCommands();

		// Check for the remote controller signal.
		MazeCheckRemote();

		// Run the state machine until the job is done or stop signal received.
		if (mazeFinished == false && recStopCar == false)
		{
			MazeMainStateMachine();
		}
		else if (recStopCar == true)
		{
			// Stop signal is received, stop the car.
			actualParams.Speed = 0;
			motorSetDutyCycle(0);
		}

		// Indicate if the maze is finished and signal the app_speedRun to start. If hard reset is present, then skip
		// the maze and start the speed run.
		if (mazeFinished == true || xEventGroupGetBits(event_MazeOut) > 0)
		{
			// Reset signal received. Skip the maze and signal to the speed run state machine.
			mazeFinished = true;
			xEventGroupSetBits(event_MazeOut, 0);

			smMainState = eSTATE_MAIN_OUT;
		}

		// Detect line and control the servo and the speed of the car.
		MazeCntrLineFollow();

		// TODO Check for frontal collision.

		// Trace out the necessary infos.
		MazeTraceInformations();

		vTaskDelay(TASK_DELAY_5_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//**********************************************************************************************************************
//! This function holds the main state machine that can navigate through the labyrinth.
//!
//! The main state machine has 4 states: Ready, Discover, Inclination, Out. The car starts in the Ready state and waits
//! for a trigger (remote, start gate). On trigger the car gets into the Discover state. In this state the car runs
//! through the labyrinth looking for the segments and the inclination point. When this point and all of the segments
//! are found and discovered, then the car gets into the Inclination state. Now the car plans a route to leave the
//! labyrinth and changes lane to the speed run route, behind the safety car. This is the Out state.
//!
//! @return -
//**********************************************************************************************************************
static void MazeMainStateMachine (void)
{
	switch (smMainState)
	{
		case eSTATE_MAIN_READY:
		{
			// Standing in the start position and radio trigger.
			actualParams.Speed = 0;

			//TODO Wait for the start radio signal.
			if (true)
			{
				// Trigger received -> DISCOVER state.
				smMainState = eSTATE_MAIN_DISCOVER;
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

			// All of the segments are discovered and reached -> INCLINATION state.
			//smMainState = eSTATE_MAIN_INCLINATION;
			break;
		}
		case eSTATE_MAIN_INCLINATION:
		{
			// Load in control parameters.
			actualParams.Kp = paramList.inclination.Kp;
			actualParams.Kd = paramList.inclination.Kd;
			actualParams.Speed = paramList.inclination.Speed;

			//____________________________________________STEP 1________________________________________________
			// Plan a path to the exit

			// Drive to the exit

			//____________________________________________STEP 2________________________________________________
			// At the exit find the markings and slow down.

			// Steer in the direction if the markings until the car leaves the lines (45deg).

			// Check the distance sensor for collision and go until the new line is found. If collision warning,
			// then stop.

			// New lines found -> OUT state.
			//smMainState = eSTATE_MAIN_OUT;
			break;
		}
		case eSTATE_MAIN_OUT:
		{
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

//**********************************************************************************************************************
//! TODO
//!
//! @return -
//**********************************************************************************************************************
static void	MazeCntrLineFollow (void)
{
	float line_diff;
	float servo_angle;
	float P_modifier;
	float D_modifier;

	// Detect line.
	line_prevPos = line_pos;
	line_pos = lineGetSingle() / 1000; // m -> mm
	line_diff = line_pos - line_prevPos;

	// Control the servo.
	P_modifier = line_pos  * actualParams.Kp;
	D_modifier = line_diff * actualParams.Kd;
	servo_angle = -0.75f * (P_modifier + D_modifier);

	// Actuate.
	motorSetDutyCycle(actualParams.Speed);
	servoSetAngle(servo_angle);

	// Trace
	txServoAngle = servo_angle;
	txLineMainLinePos = line_pos;
}

//**********************************************************************************************************************
//!	This function translates the commands of the CDT application and answers them.
//!
//! This function updates the received data structure. According to the command it can reset the main state machine,
//! provide specific parameters to be returned and it can also modify existing control parameters.
//!
//! @return -
//**********************************************************************************************************************
static void MazeProcessRecCommands (void)
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

//**********************************************************************************************************************
//!	This function checks if the remote signal is present.
//!
//! The function turns the green led on (LD2) if the remote signal is present and resets the #recStopCar flag so the
//! car can run.
//!
//! @return -
//**********************************************************************************************************************
static void MazeCheckRemote	(void)
{
	if (remoteGetState())
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		recStopCar |= false;
	}
	else
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		recStopCar |= true;
	}
}

//**********************************************************************************************************************
//! This function collects the information about the maze app and send out them.
//!
//!
//!
//! @return -
//**********************************************************************************************************************
static void MazeTraceInformations  (void)
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

	txServoAngle = servoGetAngle();
	txLineNumber = lineGetRawFront().cnt;

	if (lineGetRoadSignal() != Nothing)
	{
		txLineSecLinePos = lineGetRawFront().lines[2];
	}

	traceBluetooth(BT_LOG_STEER_WHEEL_ANGLE, &txSteerWheelAngle);
	traceBluetooth(BT_LOG_SERVO_ANGLE, &txServoAngle);
	traceBluetooth(BT_LOG_LINE_LINE_NBR, &txLineNumber);
	traceBluetooth(BT_LOG_LINE_MAIN_LINE_POS, &txLineMainLinePos);
	traceBluetooth(BT_LOG_LINE_SEC_LINE_POS, &txLineSecLinePos);
}

//**********************************************************************************************************************
//!	This function convert a bool array to a integer.
//!
//! The bit will represent a value of 2 power and summed together.
//!
//! @return AN integer value that holds the informations of a 12 bit flag array.
//**********************************************************************************************************************
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

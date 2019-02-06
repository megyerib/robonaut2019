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
#include "app_maze_StateMachine.h"
#include "app_roadsignal.h"
#include "app_common.h"
#include "navigation.h"
#include "trace.h"
#include "remote.h"
#include "main.h"
#include "motor.h"
#include "line.h"
#include "bsp_servo.h"
#include "speed.h"
#include "app_controllers.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define MAZE_SPEED_TI		(50.0f)
#define MAZE_SPEED_KC		(100.0f)

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Event flag (first bit) that indicates if we have left the maze.
EventGroupHandle_t event_MazeOut;

extern bool mazeFinished;
extern eSTATE_MAIN smMainState;
extern cSEGMENT map[20];
extern bool segments[12];
extern uint32_t inclinSegment;
extern cPD_CNTRL_PARAMS mazeActualParams;
extern cMAZE_PD_CONTROL_PARAM_LIST paramList;

 bool turnOffLineFollow;

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

extern float mazeActLine;
extern float mazePrevLine;
extern float mazeServoAngle;
static float mazeActSpeed;
static float mazePrevSpeed;
static float mazeFk;
static uint32_t mazeActSpeedDuty;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void 	MazeProcessRecCommands (void);
static void 	MazeCheckRemote		   (void);
static void 	MazeTraceInformations  (void);
static uint32_t MazeSegmentsConverter  (void);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Maze (void)
{
	event_MazeOut = xEventGroupCreate();
	xEventGroupClearBits(event_MazeOut, 1);

	MazeStateMachinesInit();

	// Initial parameters of the Discover state.
	paramList.discover.Kp	 = 0.025;
	paramList.discover.Kd	 = 3.68;
	paramList.discover.Speed = 19;

	// Initial parameters of the Discover state.
	paramList.inclination.Kp	 = 0.025;
	paramList.inclination.Kd	 = 4;
	paramList.inclination.Speed  = 15;

	// Initial parameters for the line follower controller.
	mazeActLine = 0;
	mazePrevLine = 0;

	smMainState = eSTATE_MAIN_READY;

	// Task can be created now.
	xTaskCreate(Task_Maze,
				"TASK_MAZE",
				DEFAULT_STACK_SIZE+200,
				NULL,
				TASK_MAZE_PRIO,
				NULL);
}

void Task_Maze (void* p)
{
	(void)p;
	//float r_speed = 2;
	mazeFinished = true;

	while (1)
	{
		// Process the received parameters.
		MazeProcessRecCommands();

		if (mazeFinished == false)
		{
			// Check for the remote controller signal.
			MazeCheckRemote();

			// Save previous sensor data.
			mazePrevLine = mazeActLine;
			mazePrevSpeed = mazeActSpeed;

			// Get actual sensor data.
			mazeActSpeed = speedGet();
			mazeActLine = lineGetSingle() * 1000;

			// Run the state machine until the job is done or stop signal received.
			if (mazeFinished == false && recStopCar == true)
			{
				MazeMainStateMachine();
			}

			// Controllers.
			if (mazeFinished == false)
			{
				// Control the servo.
				mazeServoAngle = cntrlLineFollow(mazeActLine, mazePrevLine, 0, mazeActualParams.Kp, mazeActualParams.Kd);

				// Control the speed.
				mazeActSpeedDuty = cntrSpeed(mazeActualParams.Speed, mazePrevSpeed, mazeActSpeed, MAZE_SPEED_TI, &mazeFk, MAZE_SPEED_KC);
			}

			// Stop if the car has to stop (remote signal is not present).
			if (recStopCar == true)
			{
				// Stop signal is received, stop the car.
				mazeActSpeedDuty = 0;
				motorSetDutyCycle(0);	// Just to be serious.
			}
		}

		// Indicate if the maze is finished and signal the app_speedRun to start. If hard reset is present, then skip
		// the maze and start the speed run.
		if (mazeFinished == true || xEventGroupGetBits(event_MazeOut) > 0)
		{
			// Reset signal received. Skip the maze and signal to the speed run state machine.
			mazeFinished = true;
			xEventGroupSetBits(event_MazeOut, 1);

			smMainState = eSTATE_MAIN_OUT;
		}

		// TODO Check for frontal collision.

		// Actuate.
		motorSetDutyCycle(mazeActSpeedDuty);
		servoSetAngle(mazeServoAngle);

		// Trace out the necessary infos.
		MazeTraceInformations();

		vTaskDelay(TASK_DELAY_5_MS);	// TASK_DELAY_5_MS TODO
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

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

	// Car stop signal
	recStopCar = rxData.StopCar;

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
		recStopCar &= false;
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
	txActKp         = mazeActualParams.Kp;
	txActKd         = mazeActualParams.Kd;
	txActSpeed      = mazeActualParams.Speed;
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
		txLineSecLinePos = lineGetRawFront().lines[0];
	}

	// TODO debug
	if (txServoAngle < PI/2)
	{
		txSteerWheelAngle =  PI/2.0f - (PI/2.0f - txServoAngle) / 2.0f;
	}
	else
	{
		txSteerWheelAngle = PI/2.0f + (txServoAngle - PI/2.0f) / 2.0f;
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

	if (segments[0] == true)	retVal += 2048;
	if (segments[1] == true)	retVal += 1024;
	if (segments[2] == true)	retVal += 512;
	if (segments[3] == true)	retVal += 256;
	if (segments[4] == true)	retVal += 128;
	if (segments[5] == true)	retVal += 64;
	if (segments[6] == true)	retVal += 32;
	if (segments[7] == true)	retVal += 16;
	if (segments[8] == true)	retVal += 8;
	if (segments[9] == true)	retVal += 4;
	if (segments[10] == true)	retVal += 2;
	if (segments[11] == true)	retVal += 1;

	return retVal;
}

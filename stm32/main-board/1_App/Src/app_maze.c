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

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Event flag (first bit) that indicates if we have left the maze.
EventGroupHandle_t event_MazeOut;

extern bool mazeFinished;
extern eSTATE_MAIN smMainState;
extern cSEGMENT map[20];
extern bool segments[12];
extern uint32_t inclinSegment;
extern cPD_CONTROLLER_PARAMS actualParams;
extern cMAZE_PD_CONTROL_PARAM_LIST paramList;

extern bool turnOffLineFollow;

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
extern float mazeLinePosPrev;
//! Position of the main line in the current task period.
extern float mazeLinePos;

static float speed_current;
static float speed_prev;
//static uint32_t D_prev;
//static uint32_t D_curr;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void 	MazeProcessRecCommands (void);
static void 	MazeCheckRemote		   (void);
static void 	MazeTraceInformations  (void);
static uint32_t MazeSegmentsConverter  (void);
static void		MazeCntrLineFollow	   (void);
static void		MazeCntrSpeed 		   (float r_speed);

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
	mazeLinePos = 0;
	mazeLinePosPrev = 0;

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

	while (1)
	{
		// Process the received parameters.
		MazeProcessRecCommands();

		// Check for the remote controller signal.
		MazeCheckRemote();

		// Get actual data.
		mazeLinePosPrev = mazeLinePos;
		mazeLinePos = lineGetSingle();

		// Run the state machine until the job is done or stop signal received.
		if (mazeFinished == false && recStopCar == false)
		{
			//MazeCntrSpeed (r_speed);
			MazeMainStateMachine();
		}
		else if (recStopCar == true)
		{
			// Stop signal is received, stop the car.
			actualParams.Speed = 0;
			motorSetDutyCycle(0);
			mazeLinePos = getPrevLine();
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
		motorSetDutyCycle(actualParams.Speed);
		//MazeCntrSpeed(); //TODO

		// TODO Check for frontal collision.

		// Trace out the necessary infos.
		MazeTraceInformations();

		vTaskDelay(TASK_DELAY_5_MS);	// TASK_DELAY_5_MS TODO
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

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

	mazeLinePos *= 1000;
	line_diff = mazeLinePos - mazeLinePosPrev;

	// Control the servo.
	P_modifier = mazeLinePos  * actualParams.Kp;
	D_modifier = line_diff * actualParams.Kd;
	servo_angle = -0.75f * (P_modifier + D_modifier);

	// Actuate.
	if (turnOffLineFollow == false)
	{
		servoSetAngle(servo_angle);
	}

	// Trace
	txServoAngle = servo_angle;
	txLineMainLinePos = mazeLinePos;
}

static void	MazeCntrSpeed (float r_speed)
{
	float e_speed;

	float Ts = 5.0;			//sampling time in ms
	float Ti = 50.0;			//integrating time ms
	float beta = exp(-Ts/Ti);
	float fk = 0;

	float Umin = 10.0;
	float Umax = 85.0;
	float uk;
	float kc = 100.0;

	speed_prev = speed_current;				// v previous
	speed_current = speedGet();				// V actual
	e_speed = r_speed - speed_current;		// v diff = v wanted - v actual
	uk = kc * e_speed + fk;
	if(uk < Umin)
	{
		uk = Umin;
	}
	if(uk > Umax)
	{
		uk = Umax;
	}
	fk = beta*fk + ( 1 - beta*uk);

	actualParams.Speed = (uint32_t) uk;

	// Actuate.
	motorSetDutyCycle(actualParams.Speed);

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

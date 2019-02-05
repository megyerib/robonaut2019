////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_speedRun.c
//!  \brief		Contains the logic, how to drive on the speed run path.
//!  \details	See in app_speedRun.h.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "event_groups.h"
#include "app_speedRun.h"
#include "trace.h"
#include "main.h"
#include "motor.h"
#include "line.h"
#include "bsp_servo.h"
#include "bsp_sharp.h"
#include "remote.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Event flag (first bit) that indicates if we have left the maze.
extern EventGroupHandle_t event_MazeOut;
//! Flag that indicates if we are on the speed run track.
static bool	speedRunStarted;

//! This button is used in case the car can not complete the Maze. The car will start waiting behind the safety car
//! to start the speed run.
static GPIO_PinState btnHardRstSpeedRun;
//! GPIO port of the hard reset button.
static GPIO_TypeDef* btnHardRst_Port;
//! GPIO pin of the hard reset button.
static uint16_t btnHardRst_Pin;

//! This button is used in that unfortunate case, if the car get lost in the speed run and must be replaced to the line.
//! IN CASE OF: car is lost, car has crashed, bad overtaking
static GPIO_PinState btnSoftRstSpeedRun;
//! GPIO port of the soft reset button.
static GPIO_TypeDef* btnSoftRst_Port;
//! GPIO pin of the soft reset button.
static uint16_t btnSoftRst_Pin;

//! Contains the received serial data.
static cTRACE_RX_DATA rxData;
//! Flag that indicates if the car must stop.
static bool	recStopCar;
//! Flag that indicates if the overtake action is allowed.
static bool recTryOvertake;
//! Flag that indicates if the main state machine must be reset. Car starts from behind the safety car.
static bool recHardReset;
//! Flag that indicates if the actual state has to be reset or a it has to be reset to a new state.
static bool recSoftReset;
//! State into which the state machine must be reset.
static uint32_t recSoftResetTo;
//! Request for the control parameter of this state.
static uint32_t recGetState;
//! Update the control parameters of the selected state.
static uint32_t recSetState;
//! New P control parameter for the selected state.
static float recSetP;
//! New Kp control parameter for the selected state.
static float recSetKp;
//! New Kd control parameter for the selected state.
static float recSetKd;
//! New Speed control parameter for the selected state.
static uint32_t recSetSpeed;

//! Actual state of the main state machine.
static uint32_t txMainSm;
//! Actual state of the drive state machine.
static uint32_t txActState;
//! Actual P control parameter.
static float txActP;
//! Actual Kp control parameter.
static float txActKp;
//! Actual Kd control parameter.
static float txActKd;
//! Actual Speed control parameter.
static uint32_t txActSpeed;
//! Requested P control parameter.
static float txGetP;
//! Requested Kp control parameter.
static float txGetKp;
//! Requested Kd control parameter.
static float txGetKd;
//! Requested Speed control parameter.
static uint32_t	txGetSpeed;

extern bool tryToOvertake;
extern eSTATE_MAIN smMainStateSRun;
extern uint8_t actLapSegment;
extern cPD_CNTRL_PARAMS actualParamsSRun;
extern cSRUN_PD_CONTROL_PARAM_LIST paramListSRun;

extern float sRunActLine;
extern float sRunPrevLine;
extern float sRunServoAngle;

static float txSteerWheelAngle;
static float txServoAngle;
static uint8_t txLineNumber;
static float txLineMainLinePos;
static float txLineSecLinePos;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void sRunSetLap1Parameters	 (void);
static void sRunSetLap2Parameters	 (void);
static void sRunSetLap3Parameters	 (void);
static void sRunProcessRecCommands   (void);
static void sRunTraceInformations    (void);
static void sRunCollectGetParams     (void);
static void sRunUpdateParams	     (void);
static void sRunCheckButtonHardRst   (void);
static void sRunCheckButtonSoftRst   (void);
static void sRunCheckStartCondition  (void);
static void SRun_CheckRemote			 (void);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_SpeedRun (void)
{
	// Configure the GPIOs of the buttons.
	btnHardRstSpeedRun = GPIO_PIN_SET;
	//btnHardRst_Port  = GPIOx;
	//btnHardRst_Pin   = GPIO_PIN_x;

	btnSoftRstSpeedRun = GPIO_PIN_SET;
	//btnSoftRst_Port  = GPIOx;
	//btnSoftRst_Pin   = GPIO_PIN_x;

	// Reset the module.
	speedRunStarted = true;  	//TODO remove
	sRunSetLap1Parameters();
	sRunSetLap2Parameters();
	sRunSetLap3Parameters();

	// Init state machines
	sRunInitStateMachines();

	// Task can be created now.
	xTaskCreate(Task_SpeedRun,
				"TASK_SPEED_RUN",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SRUN_PRIO,
				NULL);
}

void Task_SpeedRun (void* p)
{
	(void)p;

	while (1)
	{
		// Wait until the maze task is running.
		if (speedRunStarted == false)
		{
			sRunCheckStartCondition();
		}

		// Check if remote is pulled. TODO remove before race
		SRun_CheckRemote();

		// Receive and process the data from the CDT application.
		sRunProcessRecCommands();

		// Get actual data.
		sRunPrevLine = sRunActLine;
		sRunActLine = lineGetSingle() * 1000;

		// Follow line servo angle.
		if (speedRunStarted == true)
		{
			// Detect line and control the servo and the speed of the car.
			sRunCntrLineFollow();
		}

		// Main state machine that drive though the speed run track.
		if (speedRunStarted == true  && recStopCar == false)
		{
			sRunMainStateMachine();
			// TODO drive state machine needs to follow in which segment we are.
		}
		else if (recStopCar == true)
		{
			// Stop signal is received, stop the car.
			actualParamsSRun.Speed = 0;
			motorSetDutyCycle(0);
		}

		// Check the buttons.
		sRunCheckButtonHardRst();
		sRunCheckButtonSoftRst();

		// TODO Check for frontal collision.

		// Actuate.
		motorSetDutyCycle(actualParamsSRun.Speed);
		servoSetAngle(sRunServoAngle);

		// Trace out the speed run informations.
		sRunTraceInformations();

		vTaskDelay(TASK_DELAY_5_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//**********************************************************************************************************************
//!	Provides initial values for the control parameters of the Lap1.
//!
//! @return -
//**********************************************************************************************************************
static void sRunSetLap1Parameters (void)
{
	paramListSRun.lap1[0].P = 0;	paramListSRun.lap1[0].Kp = 0.025f;	paramListSRun.lap1[0].Kd = 3.5f;	paramListSRun.lap1[0].Speed = 15;	// Straight
	paramListSRun.lap1[1].P = 0; 	paramListSRun.lap1[1].Kp = 0.025f;	paramListSRun.lap1[1].Kd = 3.5f;	paramListSRun.lap1[1].Speed = 15;	// Slow
	paramListSRun.lap1[2].P = 0; 	paramListSRun.lap1[2].Kp = 0.025f;	paramListSRun.lap1[2].Kd = 3.5f;	paramListSRun.lap1[2].Speed = 15;	// Corner
	paramListSRun.lap1[3].P = 0; 	paramListSRun.lap1[3].Kp = 0.025f;	paramListSRun.lap1[3].Kd = 3.5f;	paramListSRun.lap1[3].Speed = 15;	// Speed
	paramListSRun.lap1[4].P = 0; 	paramListSRun.lap1[4].Kp = 0.025f;	paramListSRun.lap1[4].Kd = 3.5f;	paramListSRun.lap1[4].Speed = 15;	// Straight

	paramListSRun.lap1[5].P = 0; 	paramListSRun.lap1[5].Kp = 0.025f;	paramListSRun.lap1[5].Kd = 3.5f;	paramListSRun.lap1[5].Speed = 15;	// Slow
	paramListSRun.lap1[6].P = 0; 	paramListSRun.lap1[6].Kp = 0.025f;	paramListSRun.lap1[6].Kd = 3.5f;	paramListSRun.lap1[6].Speed = 15;	// Corner
	paramListSRun.lap1[7].P = 0; 	paramListSRun.lap1[7].Kp = 0.025f;	paramListSRun.lap1[7].Kd = 3.5f;	paramListSRun.lap1[7].Speed = 15;	// Speed
	paramListSRun.lap1[8].P = 0; 	paramListSRun.lap1[8].Kp = 0.025f;	paramListSRun.lap1[8].Kd = 3.5f;	paramListSRun.lap1[8].Speed = 15;	// Straight
	paramListSRun.lap1[9].P = 0; 	paramListSRun.lap1[9].Kp = 0.025f;	paramListSRun.lap1[9].Kd = 3.5f;	paramListSRun.lap1[9].Speed = 15;	// Slow

	paramListSRun.lap1[10].P = 0;	paramListSRun.lap1[10].Kp = 0.025f;	paramListSRun.lap1[10].Kd = 3.5f; 	paramListSRun.lap1[10].Speed = 15;	// Corner
	paramListSRun.lap1[11].P = 0;	paramListSRun.lap1[11].Kp = 0.025f;	paramListSRun.lap1[11].Kd = 3.5f;	paramListSRun.lap1[11].Speed = 15;	// Speed
	paramListSRun.lap1[12].P = 0;	paramListSRun.lap1[12].Kp = 0.025f;	paramListSRun.lap1[12].Kd = 3.5f;	paramListSRun.lap1[12].Speed = 15;	// Straight
	paramListSRun.lap1[13].P = 0;	paramListSRun.lap1[13].Kp = 0.025f;	paramListSRun.lap1[13].Kd = 3.5f;	paramListSRun.lap1[13].Speed = 15;	// Slow
	paramListSRun.lap1[14].P = 0;	paramListSRun.lap1[14].Kp = 0.025f;	paramListSRun.lap1[14].Kd = 3.5f;	paramListSRun.lap1[14].Speed = 15;	// Corner
	paramListSRun.lap1[15].P = 0;	paramListSRun.lap1[15].Kp = 0.025f;	paramListSRun.lap1[15].Kd = 3.5f;	paramListSRun.lap1[15].Speed = 15;	// Speed
}

//**********************************************************************************************************************
//! Provides initial values for the control parameters of the Lap2.
//!
//! @return -
//**********************************************************************************************************************
static void sRunSetLap2Parameters (void)
{
	paramListSRun.lap2[0].P = 0;	paramListSRun.lap2[0].Kp = 0.025f;	paramListSRun.lap2[0].Kd = 3.5f;	paramListSRun.lap2[0].Speed = 15;	// Straight
	paramListSRun.lap2[1].P = 0; 	paramListSRun.lap2[1].Kp = 0.025f;	paramListSRun.lap2[1].Kd = 3.5f;	paramListSRun.lap2[1].Speed = 15;	// Slow
	paramListSRun.lap2[2].P = 0; 	paramListSRun.lap2[2].Kp = 0.025f;	paramListSRun.lap2[2].Kd = 3.5f;	paramListSRun.lap2[2].Speed = 15;	// Corner
	paramListSRun.lap2[3].P = 0; 	paramListSRun.lap2[3].Kp = 0.025f;	paramListSRun.lap2[3].Kd = 3.5f;	paramListSRun.lap2[3].Speed = 15;	// Speed
	paramListSRun.lap2[4].P = 0; 	paramListSRun.lap2[4].Kp = 0.025f;	paramListSRun.lap2[4].Kd = 3.5f;	paramListSRun.lap2[4].Speed = 15;	// Straight

	paramListSRun.lap2[5].P = 0; 	paramListSRun.lap2[5].Kp = 0.025f;	paramListSRun.lap2[5].Kd = 3.5f;	paramListSRun.lap2[5].Speed = 15;	// Slow
	paramListSRun.lap2[6].P = 0; 	paramListSRun.lap2[6].Kp = 0.025f;	paramListSRun.lap2[6].Kd = 3.5f;	paramListSRun.lap2[6].Speed = 15;	// Corner
	paramListSRun.lap2[7].P = 0; 	paramListSRun.lap2[7].Kp = 0.025f;	paramListSRun.lap2[7].Kd = 3.5f;	paramListSRun.lap2[7].Speed = 15;	// Speed
	paramListSRun.lap2[8].P = 0; 	paramListSRun.lap2[8].Kp = 0.025f;	paramListSRun.lap2[8].Kd = 3.5f;	paramListSRun.lap2[8].Speed = 15;	// Straight
	paramListSRun.lap2[9].P = 0; 	paramListSRun.lap2[9].Kp = 0.025f;	paramListSRun.lap2[9].Kd = 3.5f;	paramListSRun.lap2[9].Speed = 15;	// Slow

	paramListSRun.lap2[10].P = 0;	paramListSRun.lap2[10].Kp = 0.025f;	paramListSRun.lap2[10].Kd = 3.5f;	paramListSRun.lap2[10].Speed = 15;	// Corner
	paramListSRun.lap2[11].P = 0;	paramListSRun.lap2[11].Kp = 0.025f;	paramListSRun.lap2[11].Kd = 3.5f;	paramListSRun.lap2[11].Speed = 15;	// Speed
	paramListSRun.lap2[12].P = 0;	paramListSRun.lap2[12].Kp = 0.025f;	paramListSRun.lap2[12].Kd = 3.5f;	paramListSRun.lap2[12].Speed = 15;	// Straight
	paramListSRun.lap2[13].P = 0;	paramListSRun.lap2[13].Kp = 0.025f;	paramListSRun.lap2[13].Kd = 3.5f;	paramListSRun.lap2[13].Speed = 15;	// Slow
	paramListSRun.lap2[14].P = 0;	paramListSRun.lap2[14].Kp = 0.025f;	paramListSRun.lap2[14].Kd = 3.5f;	paramListSRun.lap2[14].Speed = 15;	// Corner
	paramListSRun.lap2[15].P = 0;	paramListSRun.lap2[15].Kp = 0.025f;	paramListSRun.lap2[15].Kd = 3.5f;	paramListSRun.lap2[15].Speed = 15;	// Speed
}

//**********************************************************************************************************************
//! Provides initial values for the control parameters of the Lap3.
//!
//! @return -
//**********************************************************************************************************************
static void sRunSetLap3Parameters (void)
{
	paramListSRun.lap3[0].P = 0;	paramListSRun.lap3[0].Kp = 0.025;	paramListSRun.lap3[0].Kd = 3.5;	paramListSRun.lap3[0].Speed = 15;
	paramListSRun.lap3[1].P = 0; 	paramListSRun.lap3[1].Kp = 0.025;	paramListSRun.lap3[1].Kd = 3.5;	paramListSRun.lap3[1].Speed = 15;
	paramListSRun.lap3[2].P = 0; 	paramListSRun.lap3[2].Kp = 0.025;	paramListSRun.lap3[2].Kd = 3.5;	paramListSRun.lap3[2].Speed = 15;
	paramListSRun.lap3[3].P = 0; 	paramListSRun.lap3[3].Kp = 0.025;	paramListSRun.lap3[3].Kd = 3.5;	paramListSRun.lap3[3].Speed = 15;
	paramListSRun.lap3[4].P = 0; 	paramListSRun.lap3[4].Kp = 0.025;	paramListSRun.lap3[4].Kd = 3.5;	paramListSRun.lap3[4].Speed = 15;

	paramListSRun.lap3[5].P = 0; 	paramListSRun.lap3[5].Kp = 0.025;	paramListSRun.lap3[5].Kd = 3.5;	paramListSRun.lap3[5].Speed = 15;
	paramListSRun.lap3[6].P = 0; 	paramListSRun.lap3[6].Kp = 0.025;	paramListSRun.lap3[6].Kd = 3.5;	paramListSRun.lap3[6].Speed = 15;
	paramListSRun.lap3[7].P = 0; 	paramListSRun.lap3[7].Kp = 0.025;	paramListSRun.lap3[7].Kd = 3.5;	paramListSRun.lap3[7].Speed = 15;
	paramListSRun.lap3[8].P = 0; 	paramListSRun.lap3[8].Kp = 0.025;	paramListSRun.lap3[8].Kd = 3.5;	paramListSRun.lap3[8].Speed = 15;
	paramListSRun.lap3[9].P = 0; 	paramListSRun.lap3[9].Kp = 0.025;	paramListSRun.lap3[9].Kd = 3.5;	paramListSRun.lap3[9].Speed = 15;

	paramListSRun.lap3[10].P = 0;	paramListSRun.lap3[10].Kp = 0.025;	paramListSRun.lap3[10].Kd = 3.5;	paramListSRun.lap3[10].Speed = 15;
	paramListSRun.lap3[11].P = 0;	paramListSRun.lap3[11].Kp = 0.025;	paramListSRun.lap3[11].Kd = 3.5;	paramListSRun.lap3[11].Speed = 15;
	paramListSRun.lap3[12].P = 0;	paramListSRun.lap3[12].Kp = 0.025;	paramListSRun.lap3[12].Kd = 3.5;	paramListSRun.lap3[12].Speed = 15;
	paramListSRun.lap3[13].P = 0;	paramListSRun.lap3[13].Kp = 0.025;	paramListSRun.lap3[13].Kd = 3.5;	paramListSRun.lap3[13].Speed = 15;
	paramListSRun.lap3[14].P = 0;	paramListSRun.lap3[14].Kp = 0.025;	paramListSRun.lap3[14].Kd = 3.5;	paramListSRun.lap3[14].Speed = 15;
	paramListSRun.lap3[15].P = 0;	paramListSRun.lap3[15].Kp = 0.025;	paramListSRun.lap3[15].Kd = 3.5;	paramListSRun.lap3[15].Speed = 15;
}

//**********************************************************************************************************************
//! This function gets and processes the serial data.
//!
//! @retval -
//**********************************************************************************************************************
static void sRunProcessRecCommands (void)
{
	rxData = traceGetRxData();

	recTryOvertake	 = rxData.SRunTryOvertake;
	recHardReset 	 = rxData.SRunHardReset;
	recSoftReset 	 = rxData.SRunSoftReset;
	recSoftResetTo   = rxData.SRunSoftResetTo;
	recGetState 	 = rxData.SRunGetState;
	recSetState 	 = rxData.SRunSetState;
	recSetP 		 = rxData.SRunSetP;
	recSetKp 		 = rxData.SRunSetKp;
	recSetKd 		 = rxData.SRunSetKd;
	recSetSpeed  	 = rxData.SRunSetSpeed;

	// Update overtake flag
	tryToOvertake = recTryOvertake;

	// Emit hard reset in sRunCheckButtonHardRst function.
	// Emit soft reset in sRunCheckButtonSoftRst function.

	// Get state parameters
	sRunCollectGetParams();

	// Set parameters
	sRunUpdateParams();
}

//**********************************************************************************************************************
//!	This function send out the informations about the Speed Run task.
//!
//! @retval -
//**********************************************************************************************************************
static void sRunTraceInformations  (void)
{
	txMainSm   = smMainStateSRun;
	txActState = actLapSegment;
	txActP	   = actualParamsSRun.P;
	txActKp	   = actualParamsSRun.Kp;
	txActKd    = actualParamsSRun.Kd;
	txActSpeed = actualParamsSRun.Speed;

	traceBluetooth(BT_LOG_SRUN_MAIN_SM, 	&txMainSm);
	traceBluetooth(BT_LOG_SRUN_ACT_STATE, 	&txActState);
	traceBluetooth(BT_LOG_SRUN_ACT_P, 		&txActP);
	traceBluetooth(BT_LOG_SRUN_ACT_KP, 		&txActKp);
	traceBluetooth(BT_LOG_SRUN_ACT_KD,		&txActKd);
	traceBluetooth(BT_LOG_SRUN_ACT_SPEED,	&txActSpeed);
	traceBluetooth(BT_LOG_SRUN_GET_P, 		&txGetP);
	traceBluetooth(BT_LOG_SRUN_GET_KP,	 	&txGetKp);
	traceBluetooth(BT_LOG_SRUN_GET_KD, 		&txGetKd);
	traceBluetooth(BT_LOG_SRUN_GET_SPEED, 	&txGetSpeed);


	txServoAngle = sRunServoAngle;
	txSteerWheelAngle = sRunServoAngle;
	txLineNumber = lineGetRawFront().cnt;

	/*if (lineGetRoadSignal() != Nothing)
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
	}*/

	traceBluetooth(BT_LOG_STEER_WHEEL_ANGLE, &txSteerWheelAngle);
	traceBluetooth(BT_LOG_SERVO_ANGLE, &txServoAngle);
	traceBluetooth(BT_LOG_LINE_LINE_NBR, &txLineNumber);
	traceBluetooth(BT_LOG_LINE_MAIN_LINE_POS, &txLineMainLinePos);
	traceBluetooth(BT_LOG_LINE_SEC_LINE_POS, &txLineSecLinePos);
}

//**********************************************************************************************************************
//! This function collects the requested P, KP, KD, Speed parameters from a specific state.
//!
//! @return -
//**********************************************************************************************************************
static void sRunCollectGetParams (void)
{
	switch (smMainStateSRun)
	{
		case eSTATE_MAIN_PARADE_LAP:
		{
			txGetP		= paramListSRun.lapParade.P;
			txGetKp		= paramListSRun.lapParade.Kp;
			txGetKd		= paramListSRun.lapParade.Kd;
			txGetSpeed	= paramListSRun.lapParade.Speed;
			break;
		}
		case eSTATE_MAIN_OVERTAKING:
		{
			txGetP		= paramListSRun.overtaking.P;
			txGetKp		= paramListSRun.overtaking.Kp;
			txGetKd		= paramListSRun.overtaking.Kd;
			txGetSpeed	= paramListSRun.overtaking.Speed;
			break;
		}
		case eSTATE_MAIN_LAP_1:
		{
			txGetP		= paramListSRun.lap1[recGetState].P;
			txGetKp		= paramListSRun.lap1[recGetState].Kp;
			txGetKd		= paramListSRun.lap1[recGetState].Kd;
			txGetSpeed	= paramListSRun.lap1[recGetState].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			txGetP		= paramListSRun.lap2[recGetState].P;
			txGetKp		= paramListSRun.lap2[recGetState].Kp;
			txGetKd		= paramListSRun.lap2[recGetState].Kd;
			txGetSpeed	= paramListSRun.lap2[recGetState].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			txGetP		= paramListSRun.lap3[recGetState].P;
			txGetKp		= paramListSRun.lap3[recGetState].Kp;
			txGetKd		= paramListSRun.lap3[recGetState].Kd;
			txGetSpeed	= paramListSRun.lap3[recGetState].Speed;
			break;
		}
		default:
		{
			break;
		}
	}
}

//**********************************************************************************************************************
//!	This function updates the control parameters of a selected state.
//!
//! @return -
//**********************************************************************************************************************
static void sRunUpdateParams (void)
{
	switch (smMainStateSRun)
	{
		case eSTATE_MAIN_PARADE_LAP:
		{
			paramListSRun.lapParade.P     = recSetP;
			paramListSRun.lapParade.Kp	  = recSetKp;
			paramListSRun.lapParade.Kd	  =	recSetKd;
			paramListSRun.lapParade.Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_OVERTAKING:
		{
			paramListSRun.overtaking.P     = recSetP;
			paramListSRun.overtaking.Kp	   = recSetKp;
			paramListSRun.overtaking.Kd	   = recSetKd;
			paramListSRun.overtaking.Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_LAP_1:
		{
			paramListSRun.lap1[recSetState].P     = recSetP;
			paramListSRun.lap1[recSetState].Kp	  = recSetKp;
			paramListSRun.lap1[recSetState].Kd	  =	recSetKd;
			paramListSRun.lap1[recSetState].Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			paramListSRun.lap2[recSetState].P     = recSetP;
			paramListSRun.lap2[recSetState].Kp	  = recSetKp;
			paramListSRun.lap2[recSetState].Kd	  =	recSetKd;
			paramListSRun.lap2[recSetState].Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			paramListSRun.lap3[recSetState].P     = recSetP;
			paramListSRun.lap3[recSetState].Kp	  = recSetKp;
			paramListSRun.lap3[recSetState].Kd	  =	recSetKd;
			paramListSRun.lap3[recSetState].Speed = recSetSpeed;
			break;
		}
		default:
		{
			break;
		}
	}
}

//**********************************************************************************************************************
//!	This function is responsible for checking the Hard reset button and signaling if it was pressed.
//!
//! @return -
//**********************************************************************************************************************
static void sRunCheckButtonHardRst (void)
{
	btnHardRstSpeedRun = HAL_GPIO_ReadPin(btnHardRst_Port, btnHardRst_Pin);
	if (/*btnHardRstSpeedRun == GPIO_PIN_RESET || TODO*/ recHardReset == true)
	{
		// Reset signal received. Skip the maze and signal to the speed run state machine.

		// Reset the state machine.
		smMainStateSRun = eSTATE_MAIN_WAIT_BEHIND;
		actLapSegment = 0;

		// Signal to the maze task that we are out of the maze.
		xEventGroupSetBits(event_MazeOut, 0);

		// Reset event is handled.
		recHardReset = false;
	}
}

//**********************************************************************************************************************
//!	This function check the soft reset button and if it was pressed, then resets the state machine.
//!
//! @return -
//**********************************************************************************************************************
static void sRunCheckButtonSoftRst (void)
{
	btnSoftRstSpeedRun = HAL_GPIO_ReadPin(btnSoftRst_Port, btnSoftRst_Pin);
	if (/*btnSoftRstSpeedRun == GPIO_PIN_RESET || TODO */ recSoftReset == true)
	{
		// Reset signal received. Skip the maze and signal to the speed run state machine.

		if (recSoftReset == true)
		{
			smMainStateSRun = recSoftResetTo;
		}
		else
		{
			// TODO think this through.
			// actLapSegment =
		}

		// Reset event is handled.
		recSoftReset = false;
	}
}

//**********************************************************************************************************************
//! This function checks if the maze task is finished and the speed run can be started.
//!
//! @return -
//**********************************************************************************************************************
static void sRunCheckStartCondition (void)
{
	EventBits_t bits = xEventGroupGetBits(event_MazeOut);

	// Wait for the event.
	if (bits > 0)
	{
		speedRunStarted = true;
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
static void SRun_CheckRemote	(void)
{
	if (remoteGetState())
	{
		// Trigger is pulled.
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		recStopCar &= false;
	}
	else
	{
		// Trigger is released.
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		recStopCar |= true;
	}
}

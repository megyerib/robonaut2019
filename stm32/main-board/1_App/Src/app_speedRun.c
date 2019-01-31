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

// Defines -------------------------------------------------------------------------------------------------------------

#define SRUN_FOLLOW_DISTANCE			40			//!< cm

#define SRUN_OVERTAKE_SEGMENT			8
#define SRUN_OVERTAKE_TURN_TIME 		400 		//!< 400 * task period (5ms) = 2s
#define SRUN_OVERTAKE_PASS_TIME			2000		//!< 2000* 5ms = 10s
#define SRUN_OVERTAKE_FIND_TIME			800			//!< 800 * 5ms = 4s
#define SRUN_OVERTAKE_SPEED_SLOW		15			//!< %
#define SRUN_OVERTAKE_SPEED_FAST		35			//!< %
#define SRUN_OVERTAKE_SERVO_ANGLE		20*PI/180	//!< rad

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Event flag (first bit) that indicates if we have left the maze.
extern EventGroupHandle_t event_MazeOut;
//! Flag that indicates if we are on the speed run track.
static bool	speedRunStarted;
//! Flag that indicates that the car is behind the safety car.
static bool behindSafetyCar;
//! During the Parade lap the car is allowed to try to overtake the safety car.
static bool tryToOvertake;

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

//! State of the speed run main state machine.
static eSTATE_MAIN smMainState;

//! The lap is divided into a given number of segments, and this many states has a lap state machine.
static uint8_t actLapSegment;

//! Control parameters of the actual state.
static cPD_CONTROLLER_PARAMS actualParams;
//! Contain all of the control parameters.
static cSRUN_PD_CONTROL_PARAM_LIST paramList;

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


static float line_prevPos;
static float line_pos;
static float line_diff;
static float servo_angle;
static float P_modifier;
static float D_modifier;
static float dist_front;
static float dist_frontPrev;
static float dist_diff;

static bool startGateFound;

static eSTATE_OVERTAKE overtakeState;
static uint32_t timeCounter;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void sRunMainStateMachine     (void);
static void sRunDriveStateMachine	 (void);
static void sRunOvertakeStateMachine (void);
static void sRunParadeLapAlgorithm	 (void);
static void	sRunCntrLineFollow 		 (void);
static void sRunCntrKeepDistance	 (void);
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

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_SpeedRun (void)
{
	btnHardRstSpeedRun = GPIO_PIN_SET;
	//btnHardRst_Port  = GPIOx;
	//btnHardRst_Pin   = GPIO_PIN_x;

	btnSoftRstSpeedRun = GPIO_PIN_SET;
	//btnSoftRst_Port  = GPIOx;
	//btnSoftRst_Pin   = GPIO_PIN_x;

	smMainState = eSTATE_MAIN_READY;

	// Init all unless car start when it shouldn't. TODO

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
		// Receive and process the data from the CDT application.
		sRunProcessRecCommands();

		// Wait until the maze task is running.
		if (speedRunStarted == false)
		{
			sRunCheckStartCondition();
		}

		// Main state machine that drive though the speed run track.
		if (speedRunStarted == true  && recStopCar == false)
		{
			sRunMainStateMachine();
		}
		else if (recStopCar == true)
		{
			// Stop signal is received, stop the car.
			actualParams.Speed = 0;
			motorSetDutyCycle(0);
		}

		// Check the buttons.
		sRunCheckButtonHardRst();
		sRunCheckButtonSoftRst();

		// Detect line and control the servo and the speed of the car.
		sRunCntrLineFollow();

		// Trace out the speed run informations.
		sRunTraceInformations();

		vTaskDelay(TASK_DELAY_5_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//**********************************************************************************************************************
//! State machine that manages the lap and holds the driving strategies.
//!
//! @return -
//**********************************************************************************************************************
static void sRunMainStateMachine (void)
{
	switch (smMainState)
	{
		case eSTATE_MAIN_READY:
		{
			// Check the distance change of the safety car.
			dist_frontPrev = dist_front;
			dist_front = sharpGetMeasurement().Distance;

			if (dist_front >= SRUN_FOLLOW_DISTANCE)
			{
				// Trigger: safety car starts.
				smMainState = eSTATE_MAIN_PARADE_LAP;
			}
			break;
		}
		case eSTATE_MAIN_PARADE_LAP:
		{
			sRunParadeLapAlgorithm();
			break;
		}
		case eSTATE_MAIN_OVERTAKING:
		{
			sRunOvertakeStateMachine();
			break;
		}
		case eSTATE_MAIN_LAP_1:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_LAP_2;
			}
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_LAP_3;
			}
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_STOP;
			}
			break;
		}
		case eSTATE_MAIN_STOP:
		{
			// Race is complete, stop the car.
			motorSetDutyCycle(0);
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
//! This state machine is responsible for how the car act at the specific segments of the lap.
//!
//! @return -
//**********************************************************************************************************************
static void sRunDriveStateMachine (void)
{

}

//**********************************************************************************************************************
//! TODO parameters
//!
//! @return -
//**********************************************************************************************************************
static void sRunOvertakeStateMachine (void)
{
	// Maneuver
	switch (overtakeState)
	{
		case eSTATE_OVERTAKE_LEAVE_LINE:
		{
			// Slow down.
			motorSetDutyCycle(SRUN_OVERTAKE_SPEED_SLOW);

			// Turn left for a bit
			servoSetAngle(SRUN_OVERTAKE_SERVO_ANGLE);

			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_TURN_TIME)
			{
				// Reset the time counter.
				timeCounter = 0;

				// Get back to parallel.
				overtakeState = eSTATE_OVERTAKE_GET_PARALLEL;
			}
			else
			{
				timeCounter++;
			}
			break;
		}
		case eSTATE_OVERTAKE_GET_PARALLEL:
		{
			// Turn right to get parallel with the line.
			servoSetAngle(-SRUN_OVERTAKE_SERVO_ANGLE);

			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_TURN_TIME)
			{
				// Reset the time counter.
				timeCounter = 0;

				// Pass the safety car.
				overtakeState = eSTATE_OVERTAKE_PASS_SAFETY_CAR;
			}
			else
			{
				timeCounter++;
			}
			break;
		}
		case eSTATE_OVERTAKE_PASS_SAFETY_CAR:
		{
			// Speed up.
			motorSetDutyCycle(paramList.overtaking.Speed);

			// After a given time slow down.
			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_PASS_TIME)
			{
				// Reset the time counter.
				timeCounter = 0;

				// Get back to the track
				overtakeState = eSTATE_OVERTAKE_FIND_LINE;
			}
			else
			{
				timeCounter++;
			}
			break;
		}
		case eSTATE_OVERTAKE_FIND_LINE:
		{
			// Slow down.
			motorSetDutyCycle(SRUN_OVERTAKE_SPEED_SLOW);

			// Turn right.
			servoSetAngle(-SRUN_OVERTAKE_SERVO_ANGLE);

			// Search for the line.
			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_FIND_TIME)
			{
				// Could not find the line, the car is lost.
				overtakeState = eSTATE_OVERTAKE_FAILED;
			}
			else
			{
				timeCounter++;

				// If the line is back, then it was successful. If not stop after a time (no collision).
				if (lineGetSingle() != 0)
				{
					// The car is back on track.
					overtakeState = eSTATE_OVERTAKE_SUCCESS;
				}
			}
			break;
		}
		case eSTATE_OVERTAKE_SUCCESS:
		{
			// Overtake is finished
			tryToOvertake = false;
			behindSafetyCar = false;

			// Resume the Parade lap.
			smMainState = eSTATE_MAIN_PARADE_LAP;
		}
		case eSTATE_OVERTAKE_FAILED:
		{
			// Stop the car.
			motorSetDutyCycle(0);

			if (lineGetSingle() != 0)
			{
				// If the line is back continue the Parade Lap.
				// Overtake is finished. The car is still behind the safety car.
				tryToOvertake = false;
				behindSafetyCar = true;

				// Resume the Parade lap.
				smMainState = eSTATE_MAIN_PARADE_LAP;
			}
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
//!
//!
//! @return -
//**********************************************************************************************************************
static void sRunParadeLapAlgorithm (void)
{
	if (behindSafetyCar == true)
	{
		// Load in control parameters.
		actualParams.P = paramList.lapParade.P;
		actualParams.Kp = paramList.lapParade.Kp;
		actualParams.Kd = paramList.lapParade.Kd;
		actualParams.Speed = paramList.lapParade.Speed;

		// Follow the safety car. WARNING: Keep distance calculates the speed, line follow set the speed.
		sRunCntrKeepDistance();
		sRunCntrLineFollow();

		// In the right place check if we can try overtaking.

		// Try to overtake if it is enabled.
		if (tryToOvertake == true && actLapSegment == SRUN_OVERTAKE_SEGMENT)
		{
			smMainState = eSTATE_MAIN_OVERTAKING;

			// Reset the time counter for the maneuver.
			timeCounter = 0;
		}
		else
		{
			// Follow the safety car until the last corner.

			// Start gate means a new lap.
			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_LAP_1;
			}
		}
	}
	else
	{
		// We overtook the safety car!!! :D
		// Speed up. Load in control parameters.
		actualParams.P = paramList.lapParade.P;
		actualParams.Kp = paramList.lapParade.Kp;
		actualParams.Kd = paramList.lapParade.Kd;
		actualParams.Speed = paramList.lapParade.Speed;

		// Follow the track.
		sRunCntrLineFollow();

		// Start gate means a new lap.
		if (startGateFound == true)
		{
			smMainState = eSTATE_MAIN_LAP_1;
		}
	}
}

//**********************************************************************************************************************
//!	Line follow PD controller.
//!
//! @return -
//**********************************************************************************************************************
static void	sRunCntrLineFollow (void)
{
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
}

//**********************************************************************************************************************
//! Distance P controller.
//!
//! @return -
//**********************************************************************************************************************
static void sRunCntrKeepDistance (void)
{
	dist_front = sharpGetMeasurement().Distance;
	dist_diff = SRUN_FOLLOW_DISTANCE - dist_front;

	actualParams.Speed = actualParams.P * dist_diff;
}

//**********************************************************************************************************************
//!
//!
//! @return -
//**********************************************************************************************************************
static void sRunSetLap1Parameters	(void)
{

}

//**********************************************************************************************************************
//!
//!
//! @return -
//**********************************************************************************************************************
static void sRunSetLap2Parameters	(void)
{

}

//**********************************************************************************************************************
//!
//!
//! @return -
//**********************************************************************************************************************
static void sRunSetLap3Parameters	(void)
{

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
	txMainSm   = smMainState;
	txActState = actLapSegment;
	txActP	   = actualParams.P;
	txActKp	   = actualParams.Kp;
	txActKd    = actualParams.Kp;
	txActSpeed = actualParams.Speed;

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
}

//**********************************************************************************************************************
//! This function collects the requested P, KP, KD, Speed parameters from a specific state.
//!
//! @return -
//**********************************************************************************************************************
static void sRunCollectGetParams (void)
{
	switch (smMainState)
	{
		case eSTATE_MAIN_PARADE_LAP:
		{
			txGetP		= paramList.lapParade.P;
			txGetKp		= paramList.lapParade.Kp;
			txGetKd		= paramList.lapParade.Kd;
			txGetSpeed	= paramList.lapParade.Speed;
			break;
		}
		case eSTATE_MAIN_OVERTAKING:
		{
			txGetP		= paramList.overtaking.P;
			txGetKp		= paramList.overtaking.Kp;
			txGetKd		= paramList.overtaking.Kd;
			txGetSpeed	= paramList.overtaking.Speed;
			break;
		}
		case eSTATE_MAIN_LAP_1:
		{
			txGetP		= paramList.lap1[recGetState].P;
			txGetKp		= paramList.lap1[recGetState].Kp;
			txGetKd		= paramList.lap1[recGetState].Kd;
			txGetSpeed	= paramList.lap1[recGetState].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			txGetP		= paramList.lap2[recGetState].P;
			txGetKp		= paramList.lap2[recGetState].Kp;
			txGetKd		= paramList.lap2[recGetState].Kd;
			txGetSpeed	= paramList.lap2[recGetState].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			txGetP		= paramList.lap3[recGetState].P;
			txGetKp		= paramList.lap3[recGetState].Kp;
			txGetKd		= paramList.lap3[recGetState].Kd;
			txGetSpeed	= paramList.lap3[recGetState].Speed;
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
	switch (smMainState)
	{
		case eSTATE_MAIN_PARADE_LAP:
		{
			paramList.lapParade.P     = recSetP;
			paramList.lapParade.Kp	  = recSetKp;
			paramList.lapParade.Kd	  =	recSetKd;
			paramList.lapParade.Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_OVERTAKING:
		{
			paramList.overtaking.P     = recSetP;
			paramList.overtaking.Kp	   = recSetKp;
			paramList.overtaking.Kd	   = recSetKd;
			paramList.overtaking.Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_LAP_1:
		{
			paramList.lap1[recSetState].P     = recSetP;
			paramList.lap1[recSetState].Kp	  = recSetKp;
			paramList.lap1[recSetState].Kd	  =	recSetKd;
			paramList.lap1[recSetState].Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			paramList.lap2[recSetState].P     = recSetP;
			paramList.lap2[recSetState].Kp	  = recSetKp;
			paramList.lap2[recSetState].Kd	  =	recSetKd;
			paramList.lap2[recSetState].Speed = recSetSpeed;
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			paramList.lap3[recSetState].P     = recSetP;
			paramList.lap3[recSetState].Kp	  = recSetKp;
			paramList.lap3[recSetState].Kd	  =	recSetKd;
			paramList.lap3[recSetState].Speed = recSetSpeed;
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
	if (btnHardRstSpeedRun == GPIO_PIN_RESET || recHardReset == true)
	{
		// Reset signal received. Skip the maze and signal to the speed run state machine.

		// Reset the state machine.
		smMainState = eSTATE_MAIN_PARADE_LAP;
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
	if (btnSoftRstSpeedRun == GPIO_PIN_RESET || recSoftReset == true)
	{
		// Reset signal received. Skip the maze and signal to the speed run state machine.

		if (recSoftReset == true)
		{
			actLapSegment = recSoftResetTo;
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
	// Wait for the event.
	if (xEventGroupGetBits(event_MazeOut) > 0)
	{
		speedRunStarted = true;
	}
}

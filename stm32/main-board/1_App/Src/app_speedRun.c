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
#include "app_common.h"
#include "main.h"
#include "trace.h"

// Defines -------------------------------------------------------------------------------------------------------------

//! Determines how many segments a lap has.
#define	SRUN_LAP_SEGMENTS	16

// Typedefs ------------------------------------------------------------------------------------------------------------

//**********************************************************************************************************************
//!
//**********************************************************************************************************************
typedef enum
{
	eSTATE_MAIN_READY       = 0,	//!< The car is waiting behind the safety car to start.
	eSTATE_MAIN_PARADE_LAP,			//!< The car is following the safety car  in this lap.
	eSTATE_MAIN_OVERTAKING,			//!< The car is overtaking the safety car.
	eSTATE_MAIN_LAP_1,				//!< First lap: The safest algorithm and slowest lap (secure race points).
	eSTATE_MAIN_LAP_2,				//!< Second lap: Lap with moderate speed.
	eSTATE_MAIN_LAP_3,				//!< Third lap: The fastest lap.
	eSTATE_MAIN_STOP				//!< The car has completed 3 laps and stops.
} eSTATE_MAIN;

//**********************************************************************************************************************
//!
//**********************************************************************************************************************
typedef struct
{
	cPD_CONTROLLER_PARAMS lapParade;				//!< Warm up lap, follow the safety car. Chance to overtake.
	cPD_CONTROLLER_PARAMS overtaking;				//!< Overtake the safety car.
	cPD_CONTROLLER_PARAMS lap1[SRUN_LAP_SEGMENTS];	//!< First lap, safest run.
	cPD_CONTROLLER_PARAMS lap2[SRUN_LAP_SEGMENTS];  //!< Second lap, moderate run.
	cPD_CONTROLLER_PARAMS lap3[SRUN_LAP_SEGMENTS];  //!< Third lap, fastest run.
} cSRUN_PD_CONTROL_PARAM_LIST;

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

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void sRunMainStateMachine    (void);
static void sRunDriveStateMachine	(void);
static void sRunProcessRecCommands  (void);
static void sRunTraceInformations   (void);
static void sRunCollectGetParams    (void);
static void sRunUpdateParams	    (void);
static void sRunCheckButtonHardRst  (void);
static void sRunCheckButtonSoftRst  (void);
static void sRunCheckStartCondition (void);

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
		}

		// Check the buttons.
		sRunCheckButtonHardRst();
		sRunCheckButtonSoftRst();

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
			// Trigger: safety car starts.
			smMainState = eSTATE_MAIN_PARADE_LAP;
			break;
		}
		case eSTATE_MAIN_PARADE_LAP:
		{
			// Follow the safety car

			// In the right place check if we can try overtaking.

			// Try to overtake if it is enabled.
			if (tryToOvertake == true)
			{
				smMainState = eSTATE_MAIN_OVERTAKING;
			}

			if (behindSafetyCar == true)
			{
				// Follow the car until it leaves the tack and finish lap.
			}
			else
			{
				// Finish lap.
			}

			smMainState = eSTATE_MAIN_LAP_1;
			break;
		}
		case eSTATE_MAIN_OVERTAKING:
		{
			// Maneuver

			// Overtake is finished
			tryToOvertake = false;
			behindSafetyCar = false;

			// Resume the Parade lap.
			smMainState = eSTATE_MAIN_PARADE_LAP;
			break;
		}
		case eSTATE_MAIN_LAP_1:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			smMainState = eSTATE_MAIN_LAP_2;
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			smMainState = eSTATE_MAIN_LAP_3;
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			smMainState = eSTATE_MAIN_STOP;
			break;
		}
		case eSTATE_MAIN_STOP:
		{
			// Race is complete, stop the car.

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

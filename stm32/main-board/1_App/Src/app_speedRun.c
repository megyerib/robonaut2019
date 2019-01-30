////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_speedRun.c
//!  \brief		Contains the logic, how to drive on the speed run path.
//!  \details	See in app_speedRun.h.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_speedRun.h"
#include "app_common.h"
#include "main.h"
#include "trace.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define	SRUN_LAP_SEGMENTS	16

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	eSTATE_MAIN_READY       = 0,	//! The car is waiting behind the safety car to start.
	eSTATE_MAIN_PARADE_LAP,			//! The car is following the safety car  in this lap.
	eSTATE_MAIN_OVERTAKING,			//! The car is overtaking the safety car.
	eSTATE_MAIN_LAP_1,				//! First lap: The safest algorithm and slowest lap (secure race points).
	eSTATE_MAIN_LAP_2,				//! Second lap: Lap with moderate speed.
	eSTATE_MAIN_LAP_3,				//! Third lap: The fastest lap.
	eSTATE_MAIN_STOP				//!	The car has completed 3 laps and stops.
} eSTATE_MAIN;

typedef struct
{
	cPD_CONTROLLER_PARAMS lapParade;
	cPD_CONTROLLER_PARAMS overtaking;
	cPD_CONTROLLER_PARAMS lap1[SRUN_LAP_SEGMENTS];
	cPD_CONTROLLER_PARAMS lap2[SRUN_LAP_SEGMENTS];
	cPD_CONTROLLER_PARAMS lap3[SRUN_LAP_SEGMENTS];
} cSRUN_PD_CONTROL_PARAM_LIST;

// Local (static) & extern variables -----------------------------------------------------------------------------------

//! This button is used in case the car can not complete the Maze. The car will start waiting behind the safety car
//! to start the speed run.
static GPIO_PinState btnHardRstSpeedRun;
//! GPIO port of the hard reset button.
static GPIO_TypeDef* btnHardRst_Port;
//! GPIO pin of the hard reset button.
static uint16_t      btnHardRst_Pin;

//! This button is used in that unfortunate case, if the car get lost in the speed run and must be replaced to the line.
//! IN CASE OF: car is lost, car has crashed, bad overtaking
static GPIO_PinState btnSoftRstSpeedRun;
//! GPIO port of the soft reset button.
static GPIO_TypeDef* btnSoftRst_Port;
//! GPIO pin of the soft reset button.
static uint16_t      btnSoftRst_Pin;

//! State of the speed run main state machine.
static eSTATE_MAIN smMainState;

//! The lap is divided into a given number of segments, and this many states has a lap state machine.
static uint8_t actLapSegment;

// TODO comment
static bool tryToOvertake;

// TODO comment
static cPD_CONTROLLER_PARAMS actualParams;
static cSRUN_PD_CONTROL_PARAM_LIST paramList;

// TODO comment
static cTRACE_RX_DATA rxData;
static bool		recStopCar;
static bool 	recTryOvertake;
static bool 	recHardReset;
static bool 	recSoftReset;
static uint32_t recSoftResetTo;
static uint32_t recGetState;
static uint32_t recSetState;
static float	recSetP;
static float	recSetKp;
static float	recSetKd;
static uint32_t recSetSpeed;

//TODO comment
static uint32_t txMainSm;
static uint32_t txActState;
static float	txActP;
static float	txActKp;
static float	txActKd;
static uint32_t txActSpeed;
static float	txGetP;
static float	txGetKp;
static float	txGetKd;
static uint32_t	txGetSpeed;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void ProcessReceivedCommands (void);
static void TraceSrunInformations   (void);
static void CollectGetParams        (void);
static void UpdateParams			(void);

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
		//________________________________________________RECEIVE PARAMETERS____________________________________________
		ProcessReceivedCommands();

		//__________________________________________________STATE MACHINE_______________________________________________

		switch (smMainState)
		{
			case eSTATE_MAIN_READY:
			{

				break;
			}
			case eSTATE_MAIN_PARADE_LAP:
			{

				break;
			}
			case eSTATE_MAIN_OVERTAKING:
			{

				break;
			}
			case eSTATE_MAIN_LAP_1:
			{

				break;
			}
			case eSTATE_MAIN_LAP_2:
			{

				break;
			}
			case eSTATE_MAIN_LAP_3:
			{

				break;
			}
			case eSTATE_MAIN_STOP:
			{

				break;
			}
			default:
			{
				break;
			}
		}

		//__________________________________________________RESET BUTTONS_______________________________________________

		// Check the button.
		btnHardRstSpeedRun = HAL_GPIO_ReadPin(btnHardRst_Port, btnHardRst_Pin);
		if (btnHardRstSpeedRun == GPIO_PIN_RESET || recHardReset == true)
		{
			// Reset signal received. Skip the maze and signal to the speed run state machine.

			// Reset the state machine.
			smMainState = eSTATE_MAIN_PARADE_LAP;
			actLapSegment = 0;

			// TODO signal to the other task: event bit.


			// Reset event is handled.
			recHardReset = false;
		}

		// Check the button.
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

			// TODO signal to the other task: event bit.

			// Reset event is handled.
			recSoftReset = false;
		}

		//_____________________________________________________TRACE____________________________________________________
		TraceSrunInformations();

		vTaskDelay(TASK_DELAY_5_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void ProcessReceivedCommands (void)
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

	// Emit hard reset in RESET BUTTON section.
	// Emit soft reset in RESET BUTTON section.

	// Get state parameters
	CollectGetParams();

	// Set parameters
	UpdateParams();
}

static void TraceSrunInformations  (void)
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

static void CollectGetParams (void)
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

static void UpdateParams (void)
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

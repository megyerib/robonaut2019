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

// Defines -------------------------------------------------------------------------------------------------------------
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

// Local (static) function prototypes ----------------------------------------------------------------------------------
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

		// Check the button.
		btnHardRstSpeedRun = HAL_GPIO_ReadPin(btnHardRst_Port, btnHardRst_Pin);
		if (btnHardRstSpeedRun == GPIO_PIN_RESET)
		{
			// Reset signal received. Skip the maze and signal to the speed run state machine.
			// TODO signal to the other task: event bit.
		}

		// Check the button.
		btnSoftRstSpeedRun = HAL_GPIO_ReadPin(btnSoftRst_Port, btnSoftRst_Pin);
		if (btnSoftRstSpeedRun == GPIO_PIN_RESET)
		{
			// Reset signal received. Skip the maze and signal to the speed run state machine.
			// TODO signal to the other task: event bit.
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

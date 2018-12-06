////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_QualificationStateMachine.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include "app_QualificationStateMachine.h"

#include "motor.h"
#include "line.h"
#include "steer.h"
#include "trace.h"
#include "sharp.h"

#include "servo.h"
#include "scm_SpeedControlModule.h"
#include "bsp_uart.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	RobotCarState_Reset,
	RobotCarState_WaitingStart,
	RobotCarState_Running,
	RobotCarState_Stop

} eRobotCarState;

// Local (static) & extern variables -----------------------------------------------------------------------------------

eRobotCarState state;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void Task_QSM (void* p);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_QSM (void)
{
	state = RobotCarState_Reset;

	motorInit();
	lineInit();
	steerInit(SRV_SRT_CH6012);
	traceInit();
	scmInitControllerPI();

	xTaskCreate(Task_QSM,
				"TASK_QSM",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_QSM_PRIO,
				NULL);

	state = RobotCarState_WaitingStart;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//! @brief
void Task_QSM (void* p)
{
	(void)p;

	double   servoAngle = 1.5707;
	uint8_t  motorDutyCycle = 0;
	uint32_t elapsedTime = 0;
	LINE     blackLine;
	uint32_t sharpDist = 0;
	bool	 collisionWaringing = false;
	uint8_t  motorBoardBuffer[3];
	uint32_t i = 0;

	while (1)
	{
		switch (state)
		{
			sharpDist = sharpGetDistance();

			if (sharpDist < 20)
			{
				collisionWaringing = true;
			}
			else
			{
				collisionWaringing = false;
			}

			case RobotCarState_WaitingStart:
				// Set the wheel in straight forward.
				servoAngle = 1.5707;			// 90�

				// Speed = 0.
				motorDutyCycle = 0;				// %

				// Trigger: Blue Button pressed.
				if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
				{
					// Time measurement reset.
					elapsedTime = 0;
					// Change state.
					state = RobotCarState_Running;
				}
				break;

			case RobotCarState_Running:
				// Set fix speed.
				motorDutyCycle = 35;

				// TODO move to steer.c
				// Get line informations.
				blackLine = lineGet();

				// TODO: Steer the wheel.
				// Dummy
				servoAngle = blackLine.theta;

				// Trigger: Blue Button pressed (after 50ms for prelling), sharp distance or 5s elapsed.
				if(
						sharpDist < 20 ||
						(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET && elapsedTime > 5) ||
						elapsedTime == 500)
				{
					// Change state.
					state = RobotCarState_Stop;
				}

				// Tick Time
				elapsedTime++;

				break;

			default:
				// Something funny happened, better stop.
				motorDutyCycle = 0;
				servoAngle = 1.5707;
				state = RobotCarState_Stop;
				break;
		}


		// int to uint8_t array
		for (i = 0; i < 3; i++)
		{
			motorBoardBuffer[3-1-i] = motorDutyCycle % 10;
			motorDutyCycle /= 10;
		}


		// Send set points to the actuators.
		bspUartTransmit_IT(Uart_Motor, motorBoardBuffer, 1);
		servoSetAngle(servoAngle);


		// BT logging
		traceBluetooth(BCM_LOG_LINE_D, &blackLine.d);
		traceBluetooth(BCM_LOG_LINE_THETA, &blackLine.theta);
		traceBluetooth(BCM_LOG_SERVO_ANGLE, &servoAngle);
		traceBluetooth(BCM_LOG_SHARP_DISTANCE, &sharpDist);
		traceBluetooth(BCM_LOG_SHARP_COLLISION_WARNING, &collisionWaringing);


		vTaskDelay(10);
	}
}

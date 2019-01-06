////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_SpeedControl.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <app_SpeedControlModule.h>
#include "app_common.h"
#include "app_SpeedControl.h"

#include "trace.h"
#include "motor.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

QueueHandle_t qAlkalmazasDemoMotor;
uint8_t pwm = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_SControl(void)
{
	//scmInitControllerPI();
	motorInit();

	qAlkalmazasDemoMotor = xQueueCreate( 1, sizeof( uint8_t ) );

	xTaskCreate(Task_SControl,
				"TASK_SCNTRL",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SC_PRIO,
				NULL);
}

void Task_SControl(void* p)
{
	(void)p;

	uint8_t qValue = 0;

	while(1)
	{
		qValue = xQueueReceive(qAlkalmazasDemoMotor, &qValue, 0);
		if( qValue != pdFALSE && qValue >= 0 && qValue < 100 )
		{
			pwm = qValue;
		}

		motorSetDutyCycle(pwm);

		traceBluetooth(BCM_LOG_ENC_VEL, &pwm);

		vTaskDelay(TASK_DELAY_5_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

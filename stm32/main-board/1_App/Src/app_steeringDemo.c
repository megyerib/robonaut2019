////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_steeringDemo.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include "app_steeringDemo.h"
#include "trace.h"
#include "servo.h"
#include "motor.h"
#include "line.h"
#include "bsp_common.h"
#include "bsp_uart.h"
#include "controller.h"
#include "remote.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define STTERINGDEMO_TASK_DELAY 5

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	corner_InWait,
	corner_OutCheck,
	fast_InWait,
	fast_OutCheck,
	brake_InWait,
	brake_OutCheck
}
QUALI_STATE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

/*static double p_a;		// m
static double p_meas;	// m
static double e;		// m
static double p;		// m
static double phi_a;	// deg
static double v;		// m/s
static double L;		// m

static double Kp;		//
static double Td;		// sec
static double T;		// sec

uint8_t pwm = 10;

static cFirstOrderTF contrPD;*/

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_steeringDemo(void)
{
    servoInit();

    lineInit();

    remoteInit();

    xTaskCreate(Task_steeringDemo,
                "TASK_DEMO",
                DEFAULT_STACK_SIZE,
                NULL,
                TASK_SRV_PRIO,
                NULL);
}

float angle;
float line_pos  = 0;
float line_diff = 0;
float prevline = 0;

int actuateEnabled;

int motor_d;

float P, D;

void Task_steeringDemo(void* p)
{
	while(1)
    {
		// REMOTE CONTROL __________________________________

		if (remoteGetState())
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			actuateEnabled = 1;
		}
		else
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			vTaskDelay(STTERINGDEMO_TASK_DELAY);
			actuateEnabled = 0;
		}

		// TRACTION ________________________________________

		motor_d = 12;

		// STEERING ________________________________________

		prevline = line_pos;

		line_pos = (float) lineGet().d;

		line_diff = line_pos - prevline;

		P = line_pos  * (1.0 / 90.0f);

		D = line_diff * 1.6f;

		angle = -0.53f * (P + D);

		// ACTUATE _________________________________________

		if (actuateEnabled)
		{
			motorSetDutyCycle(motor_d);
			servoSetAngle(angle);
		}

		// TRACE ___________________________________________

		//traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);
		//traceBluetooth(BCM_LOG_LINE_D, &linepos);
		//traceBluetooth(BCM_LOG_ENC_VEL, 10);

		// END DELAY _______________________________________

		vTaskDelay(STTERINGDEMO_TASK_DELAY);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

#define ROAD_SIGNAL_THRESHOLD  5
#define FAST_IN_CNTR          10
#define BRAKE_IN_CNTR         10
#define CORNER_IN_CNTR        10

static QUALI_STATE quali_state = corner_OutCheck;
int desiredRoadSignal = 0;
int countdown;

static void qualiStateMachine()
{
	switch (quali_state)
	{
		case corner_InWait:
		{
			if (countdown-- == 0)
			{
				quali_state = corner_OutCheck;
				desiredRoadSignal = 0;

				// Corner
				// ...
			}

			break;
		}
		case corner_OutCheck:
		{
			if (lineGetRoadSignal() == TripleLine)
			{
				desiredRoadSignal++;
			}
			else
			{
				desiredRoadSignal = 0;
			}

			if (desiredRoadSignal > ROAD_SIGNAL_THRESHOLD)
			{
				quali_state = fast_In;
				countdown = FAST_IN_CNTR;

				// Corner -> Fast
				// ...
			}

			break;
		}
		case fast_InWait:
		{
			if (countdown-- == 0)
			{
				quali_state = fast_OutCheck;
				desiredRoadSignal = 0;

				// Fast
				// ...
			}

			break;
		}
		case fast_OutCheck:
		{
			if (lineGetRoadSignal() == TripleLine)
			{
				desiredRoadSignal++;
			}
			else
			{
				desiredRoadSignal = 0;
			}

			if (desiredRoadSignal > ROAD_SIGNAL_THRESHOLD)
			{
				quali_state = brake_In;
				countdown = BRAKE_IN_CNTR;

				// Fast -> Brake
				// ...
			}

			break;
		}
		case brake_InWait:
		{
			if (countdown-- == 0)
			{
				quali_state = brake_OutCheck;
				desiredRoadSignal = 0;

				// Brake
				// ...
			}

			break;
		}
		case brake_OutCheck:
		{
			if (lineGetRoadSignal() == Nothing)
			{
				desiredRoadSignal++;
			}
			else
			{
				desiredRoadSignal = 0;
			}

			if (desiredRoadSignal > ROAD_SIGNAL_THRESHOLD)
			{
				quali_state = corner_In;
				countdown = CORNER_IN_CNTR;

				// Brake -> Corner
				// ...
			}

			break;
		}
	}
}

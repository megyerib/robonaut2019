////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_steeringDemo.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <bsp_servo.h>
#include "app_common.h"
#include "app_steeringDemo.h"
#include "trace.h"
#include "motor.h"
#include "line.h"
#include "bsp_common.h"
#include "bsp_uart.h"
#include "controller.h"
#include "remote.h"
#include "main.h" // LD2

// Defines -------------------------------------------------------------------------------------------------------------

#define STEERINGDEMO_TASK_DELAY 5

// State machine parameters
#define ROAD_SIGNAL_THRESHOLD      6   /* Ennyiszer kell látnunk egy jelet, hogy elhiggyük. (ld. fent) */
#define FAST_IN_CNTR          (500/5)  /* Ennyi cikluson keresztül készülünk rá a gyors szakaszra */
#define BRAKE_IN_CNTR         (650/5)  /* Ennyi ciklus a fékezés második szakaszáig */
#define CORNER_IN_CNTR            10   /* Ennyi idõt megyünk a kanyarba befele (nem érdekes) */

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

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void qualiStateMachine();

static void setParams_corner();
static void setParams_fastIn();
static void setParams_fast();
static void setParams_brake();
static void setParams_brakeIn();

//static cFirstOrderTF contrPD;

// Local (static) function prototypes ----------------------------------------------------------------------------------

cTRACE_RX_DATA btRxData;

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_steeringDemo(void)
{
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
float K_P;
float K_D;

float P, D;

float speed;

void Task_steeringDemo(void* p)
{
	setParams_corner();

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
			vTaskDelay(STEERINGDEMO_TASK_DELAY);
			actuateEnabled = 0;
		}

		// STATE MACHINE (parameter settings) ______________

		qualiStateMachine();

		// STEERING ________________________________________

		prevline = line_pos;

		line_pos = lineGetSingle() * 1000; // m -> mm

		line_diff = line_pos - prevline;

		P = line_pos  * K_P;

		D = line_diff * K_D;

		angle = -0.75f * (P + D);

		// ACTUATE _________________________________________

		servoSetAngle(angle); // Servo enabled anyway

		if (actuateEnabled)
		{
			motorSetDutyCycle(motor_d);
		}

		// TRACE ___________________________________________

		/*traceBluetooth(BCM_LOG_SERVO_ANGLE, &angle);
		traceBluetooth(BCM_LOG_LINE_D, &line_pos);
		traceBluetooth(BCM_LOG_ENC_VEL, (void*) 12);*/

		// END DELAY _______________________________________

		vTaskDelay(STEERINGDEMO_TASK_DELAY);
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

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
				setParams_corner();
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
				quali_state = fast_InWait;
				countdown = FAST_IN_CNTR;

				// Corner -> Fast
				setParams_fastIn();
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
				setParams_fast();
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
				quali_state = brake_InWait;
				countdown = BRAKE_IN_CNTR;

				// Fast -> Brake
				setParams_brakeIn();
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
				setParams_brake();
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
				quali_state = corner_InWait;
				countdown = CORNER_IN_CNTR;

				// Brake -> Corner
				setParams_corner();
			}

			break;
		}
	}
}

static void setParams_corner()
{
	// Kanyarodunk

	K_P     =  0.025f;
	K_D     =  3.68f;
	motor_d = 19;
}

static void setParams_fastIn()
{
	// Rámegyünk a gyors szakaszra. Érdemes valahogy egyenesbe állni.

	K_P     =  0.01f;
	K_D     =  0.1f;
	motor_d = 27;
}

static void setParams_fast()
{
	// Gyorsan megyünk

	K_P     =  0.01f;
	K_D     =  0.14f;
	motor_d = 35;
}

static void setParams_brakeIn()
{
	// Fékezés eleje (akár még várhatunk is a fékezéssel)

	K_P     =  0.01f;
	K_D     =  0.1f;
	motor_d = 20;
}

static void setParams_brake()
{
	// Fékezés vége (itt lehet érdemes nagyot fékezni)

	K_P     =  0.01f;
	K_D     =  0.1f;
	motor_d =  5;
}

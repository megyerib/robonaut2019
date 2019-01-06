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
#define ROAD_SIGNAL_THRESHOLD       6  /* Ennyiszer kell l�tnunk egy jelet, hogy elhiggy�k. (ld. fent) */
#define FAST_IN_CNTR          (500/5) /* Ennyi cikluson kereszt�l k�sz�l�nk r� a gyors szakaszra */
#define BRAKE_IN_CNTR         (650/5) /* Ennyi ciklus a f�kez�s m�sodik szakasz�ig */
#define CORNER_IN_CNTR        10 /* Ennyi id�t megy�nk a kanyarba befele (nem �rdekes) */

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

cTraceRxBluetoothStruct recData;

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
float K_P;
float K_D;

float P, D;

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

		line_pos = (float) lineGet().d;

		line_diff = line_pos - prevline;

		P = line_pos  * K_P;

		D = line_diff * K_D;

		angle = -0.75f * (P + D);

		// ACTUATE _________________________________________

		(void) actuateEnabled;

		servoSetAngle(angle);

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
	// R�megy�nk a gyors szakaszra. �rdemes valahogy egyenesbe �llni.

	K_P     =  0.01f;
	K_D     =  0.1f;
	motor_d = 27;
}

static void setParams_fast()
{
	// Gyorsan megy�nk

	K_P     =  0.01f;
	K_D     =  0.14f;
	motor_d = 35;
}

static void setParams_brakeIn()
{
	// F�kez�s eleje (ak�r m�g v�rhatunk is a f�kez�ssel)

	K_P     =  0.01f;
	K_D     =  0.1f;
	motor_d = 20;
}

static void setParams_brake()
{
	// F�kez�s v�ge (itt lehet �rdemes nagyot f�kezni)

	K_P     =  0.01f;
	K_D     =  0.1f;
	motor_d =  5;
}

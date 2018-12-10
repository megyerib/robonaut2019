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

float K_P;
float K_D;

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

		motor_d = 17;

		// STEERING ________________________________________

		K_P =  2.15 * (1.0 / 90.0f);
		K_D = 1.5 * 1.6f;

		prevline = line_pos;

		line_pos = (float) lineGet().d;

		line_diff = line_pos - prevline;

		P = line_pos  * K_P;

		D = line_diff * K_D;

		angle = -0.75f * (P + D);

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

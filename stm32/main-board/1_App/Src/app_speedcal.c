////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_speedcal.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_speedcal.h"
#include "remote.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_common.h"
#include "gpio.h"
#include "line.h"
#include "motor.h"
#include "bsp_servo.h"
#include "print.h"
#include "bsp_bluetooth.h"
#include "speed.h"
#include "trace.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define TASK_DELAY   (5u) /* ms */

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	s1line,
	s1line2,
	s3lines,
	s3lines2
}
SPEEDCAL_STATE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static int actuateEnabled;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void remoteHandle();
static void lineFollow();
static void traceCounterValue(SPEEDCAL_STATE state, int32_t counter);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_SpeedCalibration(void)
{
	xTaskCreate(Task_SpeedCalibration,
				"TASK_SPEED_CALIBRATION",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SPEED_CAL_PRIO,
				NULL);
}

const char greeting[] = "Speed calibration task started\r\n";
static int greetinglen = 32;
static float speed;

void Task_SpeedCalibration(void* p)
{
	static int32_t cntrval_start;
	static int32_t cntrval_end;
	static SPEEDCAL_STATE state = s3lines;

	RoadSignal signal;

	bspBtSend((uint8_t*) greeting, greetinglen);

	while(1)
	{
		remoteHandle();

		if (actuateEnabled)
			lineFollow();

		signal = lineGetRoadSignal();

		// LINE MEASURING & TRACE __________________________

		switch (state)
		{
			case (s1line):
			{
				if (signal == TripleLine)
				{
					state = s3lines2;
				}

				break;
			}
			case (s1line2):
			{
				if (lineGetRoadSignal() == TripleLine)
				{
					cntrval_start = speedGetCounter();
					traceCounterValue(state, cntrval_start);
					state = s3lines;
				}
				else
				{
					state = s1line;
				}

				break;
			}
			case (s3lines):
			{
				if (lineGetRoadSignal() == Nothing)
				{
					state = s1line2;
				}

				break;
			}
			case (s3lines2):
			{
				if (lineGetRoadSignal() == Nothing)
				{
					cntrval_end = speedGetCounter();
					traceCounterValue(state, cntrval_end);
					state = s1line;
				}
				else
				{
					state = s3lines;
				}

				break;
			}
		}

		// TRACE SPEED _____________________________________

		speed = speedGet();
		traceBluetooth(BT_LOG_ENC_V, &speed);

		// END DELAY _______________________________________

		vTaskDelay(TASK_DELAY);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void remoteHandle()
{
	if (remoteGetState())
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		actuateEnabled = 1;
	}
	else
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		actuateEnabled = 0;
	}
}

static void lineFollow()
{
	static float angle;
	static float line_pos  = 0;
	static float line_diff = 0;
	static float prevline  = 0;

	static float K_P       =  0.025f;
	static float K_D       =  3.68f;
	static int	 motor_d   = 19;

	static float P;
	static float D;

	// STEERING ________________________________________

	prevline = line_pos;

	line_pos = lineGetSingle() * 1000.0; // m -> mm

	line_diff = line_pos - prevline;

	P = line_pos  * K_P;

	D = line_diff * K_D;

	angle = -0.75f * (P + D);

	// ACTUATE _________________________________________

	// Note: Disabling must happen outside

	motorSetDutyCycle(motor_d);
	servoSetAngle(angle);
}

static void traceCounterValue(SPEEDCAL_STATE state, int32_t counter)
{
	static const char startText[] = "Start cntr value: ";
	static const char endText[]   = "End cntr value:   ";
	int textlen = 18;

	static char btBuffer[60];
	static int btBufferLen;

	switch (state)
	{
		case s1line:
		{
			memcpy(btBuffer, endText, textlen);
			break;
		}
		case s3lines:
		{
			memcpy(btBuffer, startText, textlen);
			break;
		}
		default:
		{
			return;
		}
	}

	btBufferLen = textlen;

	char num_buf[11];
	int num_buf_len;

	print_uint32_t(counter, num_buf, &num_buf_len);

	memcpy(&btBuffer[btBufferLen], num_buf, num_buf_len);

	btBufferLen += num_buf_len;

	btBuffer[btBufferLen] = '\r';
	btBufferLen++;
	btBuffer[btBufferLen] = '\n';
	btBufferLen++;
	btBuffer[btBufferLen] = '\0';
	//btBufferLen++;

	bspBtSend((uint8_t*) btBuffer, btBufferLen);
}

// END -----------------------------------------------------------------------------------------------------------------

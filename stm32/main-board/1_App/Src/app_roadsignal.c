////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze_roadsignal.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <app_roadsignal.h>
#include "app_common.h"
#include "gpio.h"
#include "line.h"
#include "remote.h"
#include "motor.h"
#include "bsp_servo.h"
#include <math.h>
#include "bsp_bluetooth.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define LINE_BUF_SIZE   (3u)
#define LINE_BUF_ACCEPT (2u)

#define K_P_VAL         (0.02f)
#define K_D_VAL         (3.5f)
#define MOTOR_D         (19u)

#define PREV_LINES      (3u)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	float lines[3];
	int cnt;
}
FILTERED_LINES;

typedef enum
{
	None = 0,
	Single,
	Double,
	Triple
}
LINE_TYPE;

// Local (static) variables --------------------------------------------------------------------------------------------

static int   actuateEnabled = 0;
static float prevLine = 0;
static LINE_TYPE lineType = None;

static int prevLineTypes[3] = {0,0,0};
static int pltIndex = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void remoteHandle();
static void lineFollow(float line_pos, float K_P, float K_D, int motor_d);
static float followPrevLine();
static float followRightLine();
static float followLeftLine();
static void examineRoadSignals(LSO_FLOAT SensorOut);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_roadSignal(void)
{
    xTaskCreate(Task_roadSignal,     // Task function
                "TASK_ROAD_SIGNAL",  // Task name (string)
                DEFAULT_STACK_SIZE,  // Stack size
                NULL,                // Parameters (void*)
				TEST_TASK_PRIO,      // Priority
                NULL);               // Task handle
}

void Task_roadSignal(void* p)
{
	float line;

	while (1)
	{
		remoteHandle();

		line = followPrevLine();

		examineRoadSignals(lineGetRawFrontFloat());

		lineFollow(
			line,
			K_P_VAL,
			K_D_VAL,
			actuateEnabled ? MOTOR_D : 0
		);

		// END DELAY _______________________________________

		vTaskDelay(5);
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

static void lineFollow(float line_pos, float K_P, float K_D, int motor_d)
{
	float angle;
	float line_diff = 0;

	static float prevline  = 0;

	float P;
	float D;

	// STEERING ________________________________________

	line_pos *= 1000;
	line_diff = line_pos - prevline;

	P = line_pos  * K_P;
	D = line_diff * K_D;
	angle = -0.75f * (P + D);

	prevline = line_pos;

	// ACTUATE _________________________________________

	motorSetDutyCycle(motor_d);
	servoSetAngle(angle);
}

static float followPrevLine()
{
	float newLine;
	float minDiff;

	LSO_FLOAT sensorOut = lineGetRawFrontFloat();

	if (sensorOut.cnt > 0)
	{
		float diff;

		minDiff = fabs(sensorOut.lines[0] - prevLine);
		newLine = sensorOut.lines[0];

		for (int i = 1; i < sensorOut.cnt; i++)
		{
			diff = fabs(sensorOut.lines[i] - prevLine);

			if (diff < minDiff)
			{
				minDiff = diff;
				newLine = sensorOut.lines[i];
			}
		}
	}
	else
	{
		newLine = prevLine;
	}

	prevLine = newLine;

	return newLine;
}

static float followLeftLine()
{
	float newLine;

	LSO_FLOAT sensorOut = lineGetRawFrontFloat();

	if (sensorOut.cnt > 0)
	{
		newLine = sensorOut.lines[sensorOut.cnt - 1];
	}
	else
	{
		newLine = prevLine;
	}

	prevLine = newLine;

	return newLine;
}

static float followRightLine()
{
	float newLine;

	LSO_FLOAT sensorOut = lineGetRawFrontFloat();

	if (sensorOut.cnt > 0)
	{
		newLine = sensorOut.lines[0];
	}
	else
	{
		newLine = prevLine;
	}

	prevLine = newLine;

	return newLine;
}

static void examineRoadSignals(LSO_FLOAT SensorOut)
{
	int prevLinesOk;

	prevLineTypes[pltIndex] = SensorOut.cnt;
	pltIndex++;
	pltIndex %= 3;

	prevLinesOk = (prevLineTypes[0] == prevLineTypes[1] && prevLineTypes[0] == prevLineTypes[2]);

	if (prevLinesOk && SensorOut.cnt != lineType)
	{
		switch (SensorOut.cnt)
		{
			case 0:
			{
				bspBtSend((uint8_t*)"No line\r\n", 9);
			}
			case 1:
			{
				bspBtSend((uint8_t*)"Single line\r\n", 13);
			}
			case 2:
			{
				bspBtSend((uint8_t*)"Double line\r\n", 13);
			}
			case 3:
			{
				bspBtSend((uint8_t*)"Triple line\r\n", 13);
			}
		}

		lineType = SensorOut.cnt;
	}
}

// END -----------------------------------------------------------------------------------------------------------------

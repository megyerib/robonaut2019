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

// Local (static) variables --------------------------------------------------------------------------------------------

static int   actuateEnabled;
static float prevLine;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void remoteHandle();
static void lineFollow(float line_pos, float K_P, float K_D, int motor_d);
static float followPrevLine();
static float followRightLine();
static float followLeftLine();
static FILTERED_LINES getFilteredLines();

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

		line = followLeftLine();

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

	float sensorLines[3];

	LINE_SENSOR_OUT sensorOut = lineGetRawFront();

	for (int i = 0; i < sensorOut.cnt; i++)
	{
		sensorLines[i] = sensorOut.lines[i] / 1000.0f; // mm -> m
	}

	if (sensorOut.cnt > 0)
	{
		float diff;

		minDiff = fabs(sensorLines[0] - prevLine);
		newLine = sensorLines[0];

		for (int i = 1; i < sensorOut.cnt; i++)
		{
			diff = fabs(sensorLines[i] - prevLine);

			if (diff < minDiff)
			{
				minDiff = diff;
				newLine = sensorLines[i];
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

	LINE_SENSOR_OUT sensorOut = lineGetRawFront();

	if (sensorOut.cnt > 0)
	{
		newLine = sensorOut.lines[sensorOut.cnt - 1] / 1000.0; // mm -> m
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

	LINE_SENSOR_OUT sensorOut = lineGetRawFront();

	if (sensorOut.cnt > 0)
	{
		newLine = sensorOut.lines[0] / 1000.0; // mm -> m
	}
	else
	{
		newLine = prevLine;
	}

	prevLine = newLine;

	return newLine;
}

static FILTERED_LINES getFilteredLines()
{
	static LINE_SENSOR_OUT LSOut[LINE_BUF_SIZE];
	static int LSOutIndex;

	FILTERED_LINES ret;
	int lineCntSeen[MAXLINES + 1] = {0,0,0,0};
	int howManyLines = 0;
	int sums[MAXLINES] = {0,0,0};

	// Beolvassuk a vonalszenzor kimenetét
	LSOut[LSOutIndex] = lineGetRawFront();
	LSOutIndex++;

	// Milyen vonalból (0,1,2,3) mennyit láttunk?
	for (int i = 0; i < LINE_BUF_SIZE; i++)
	{
		lineCntSeen[LSOut[i].cnt]++;
	}

	// Hány vonalat látunk a többségi szavazás alapján?
	for (int i = 0; i < MAXLINES + 1; i++)
	{
		if (lineCntSeen[i] >= LINE_BUF_ACCEPT)
		{
			howManyLines = i;
		}
	}

	// Kiátlagoljuk a jó méréseket
	for (int i = 0; i < LINE_BUF_SIZE; i++)
	{
		if (LSOut[i].cnt == howManyLines)
		{
			for (int j = 0; j < LSOut[i].cnt; j++)
			{
				sums[i] += LSOut[i].lines[j];
			}
		}
	}

	ret.cnt = howManyLines;

	for (int i = 0; i < MAXLINES; i++)
	{
		ret.lines[i] = sums[i] / (MAXLINES * 1000.0f);
	}

	return ret;
}

// END -----------------------------------------------------------------------------------------------------------------

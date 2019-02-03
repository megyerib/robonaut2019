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

#define DOUBLE_LINE_THRESHOLD  (0.09f)
#define DOUBLE_LINE_HYS_LOW    (0.06f)
#define DOUBLE_LINE_HYS_HIGH   (0.12f)

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
	SingleRight,
	SingleLeft,
	DoubleNearRight,
	DoubleNearLeft,
	DoubleFar,
	Triple
}
LINE_TYPE;

// Local (static) variables --------------------------------------------------------------------------------------------

static int   actuateEnabled = 0;
static float prevLine = 0;
static int prevLineCnt = 0;

static int prevLineTypes[3] = {0,0,0};
static int pltIndex = 0;
static LINE_TYPE lineType = None;
static LINE_TYPE prevLineType = None;
static LSO_FLOAT lastDoubleLine;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void  remoteHandle();
static void  lineFollow(float line_pos, float K_P, float K_D, int motor_d);
static float followPrevLine();
static float followRightLine();
static float followLeftLine();
static void  examineRoadSignals(LSO_FLOAT SensorOut);
static int   isLeft(LSO_FLOAT sensorData, float line);

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

	if (SensorOut.cnt == 2)
	{
		lastDoubleLine = SensorOut;
	}

	if (prevLinesOk && SensorOut.cnt != prevLineCnt)
	{
		switch (SensorOut.cnt)
		{
			case 1:
			{
				if (prevLineCnt == 2 && (prevLineType == DoubleNearRight || prevLineType == DoubleNearLeft))
				{
					if (isLeft(lastDoubleLine, SensorOut.lines[0]))
					{
						lineType = Single;
						bspBtSend((uint8_t*)"Single left\r\n", 13);
					}
					else
					{
						lineType = Single;
						bspBtSend((uint8_t*)"Single right\r\n", 14);
					}
				}
				else
				{
					lineType = Single;
					bspBtSend((uint8_t*)"Single\r\n", 8); // & unemployed
				}

				break;
			}
			case 2:
			{
				if (SensorOut.lines[0] - SensorOut.lines[1] < DOUBLE_LINE_THRESHOLD)
				{
					if (isLeft(SensorOut, prevLine))
					{
						lineType = DoubleNearLeft;
						bspBtSend((uint8_t*)"Double near left\r\n", 18);
					}
					else
					{
						lineType = DoubleNearRight;
						bspBtSend((uint8_t*)"Double near right\r\n", 19);
					}
				}
				else
				{
					lineType = DoubleFar;
					bspBtSend((uint8_t*)"Double far\r\n", 12);
				}

				break;
			}
			case 3:
			{
				bspBtSend((uint8_t*)"Triple\r\n", 8);
				break;
			}
		}

		prevLineType = lineType;
		prevLineCnt = SensorOut.cnt;
	}

	if (prevLinesOk && (prevLineType == DoubleNearRight || prevLineType == DoubleNearLeft))
	{
		if (SensorOut.lines[0] - SensorOut.lines[1] > DOUBLE_LINE_HYS_HIGH)
		{
			lineType = DoubleFar;
			bspBtSend((uint8_t*)"Double far\r\n", 12);
			prevLineType = lineType;
		}
	}

	if (prevLinesOk && lineType == DoubleFar)
	{
		if (SensorOut.lines[0] - SensorOut.lines[1] < DOUBLE_LINE_HYS_LOW)
		{
			if (isLeft(SensorOut, prevLine))
			{
				lineType = DoubleNearLeft;
				bspBtSend((uint8_t*)"Double near left\r\n", 18);
			}
			else
			{
				lineType = DoubleNearRight;
				bspBtSend((uint8_t*)"Double near right\r\n", 19);
			}

			prevLineType = lineType;
		}
	}
}

static int isLeft(LSO_FLOAT sensorData, float line)
{
	if (sensorData.cnt == 2)
	{
		float diffLeft  = fabs(sensorData.lines[1] - line);
		float diffRight = fabs(sensorData.lines[0] - line);

		if (diffLeft < diffRight)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

// END -----------------------------------------------------------------------------------------------------------------

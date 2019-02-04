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
#include "speed.h"
#include "print.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define LINE_BUF_SIZE   (3u)
#define LINE_BUF_ACCEPT (2u)

#define K_P_VAL         (0.02f)
#define K_D_VAL         (3.5f)
#define MOTOR_D         (18u)

#define PREV_LINES      (4u)

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
	Single = 0,
	SingleRight,
	SingleLeft,
	DoubleNearRight,
	DoubleNearLeft,
	DoubleFar,
	Triple
}
LINE_TYPE;

// Local (static) variables --------------------------------------------------------------------------------------------

static float prevLine = 0;

static LINE_TYPE prevSections[2];
static int sectionStart;

static int prevTrackLen;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int   remoteHandle();
static void  lineFollow(float line_pos, float K_P, float K_D, int motor_d);

static LINE_TYPE getLineType(LSO_FLOAT SensorOut);
static int   isLeft(LSO_FLOAT sensorData, float line);
static void traceLineType(LINE_TYPE ltp);
static void traceCrossing(CROSSING_TYPE ctype);

static int getDistance_cm();
static void traceDistance();

static void traceInt(int i);

static CROSSING_TYPE examineExit(int length, LINE_TYPE ltype);

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
	CROSSING_TYPE crossing;

	while (1)
	{
		crossing = getCrossingType();

		if (crossing != NoCrossing)
		{
			traceCrossing(crossing);
		}

		line = getPrevLine();

		lineFollow(
			line,
			K_P_VAL,
			K_D_VAL,
			remoteHandle() ? MOTOR_D : 0
		);

		// END DELAY _______________________________________

		vTaskDelay(5);
	}
}

CROSSING_TYPE getCrossingType()
{
	LINE_TYPE lineType = getLineType(lineGetRawFrontFloat());
	CROSSING_TYPE ret = NoCrossing;
	int diff = getDistance_cm() - sectionStart;

	if (diff > 25)
	{
		if ((lineType == DoubleNearLeft) && (prevSections[0] == Single || prevSections[0] == SingleLeft || prevSections[0] == SingleRight))
		{
			// Jobb keresztezõdés elõre
			ret = CrossingAtoRB;

			prevSections[0] = DoubleNearLeft;
			prevSections[1] = DoubleNearLeft;
		}

		if ((lineType == DoubleNearRight) && (prevSections[0] == Single || prevSections[0] == SingleLeft || prevSections[0] == SingleRight))
		{
			// Bal keresztezõdés elõre
			ret = CrossingAtoLB;

			prevSections[0] = DoubleNearRight;
			prevSections[1] = DoubleNearRight;
		}
	}

	if (lineType != prevSections[1])
	{
		LINE_TYPE sec1 = prevSections[0];
		LINE_TYPE sec2 = prevSections[1];
		LINE_TYPE sec3 = lineType;

		//traceDistance();

		if (sec1 == DoubleFar && sec2 == DoubleNearLeft && sec3 == SingleRight)
		{
			ret = CrossingRtoA;
		}
		else if (sec1 == DoubleFar && sec2 == DoubleNearRight && sec3 == SingleLeft)
		{
			ret = CrossingLtoA;
		}
		else if (sec1 == DoubleFar && sec2 == DoubleNearRight && sec3 == SingleRight)
		{
			ret = CrossingBtoA_R;
		}
		else if (sec1 == DoubleFar && sec2 == DoubleNearLeft && sec3 == SingleLeft)
		{
			ret = CrossingBtoA_L;
		}

		CROSSING_TYPE smallLineType = examineExit(prevTrackLen, prevSections[1]);

		//traceInt(prevTrackLen);

		if (smallLineType != NoCrossing)
		{
			ret = smallLineType;
		}

		prevSections[0] = prevSections[1];
		prevSections[1]  = lineType;
	}

	return ret;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static int remoteHandle()
{
	int actuateEnabled;

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

	return actuateEnabled;
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

float getPrevLine()
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

float getLeftLine()
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

float getRightLine()
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

static LINE_TYPE getLineType(LSO_FLOAT SensorOut)
{
	static int prevLineCnt = 0;
	static int prevLineTypes[3] = {0,0,0};
	static int pltIndex = 0;
	static LSO_FLOAT lastDoubleLine;

	int prevLinesOk;
	LINE_TYPE lineType = prevSections[1];
	LINE_TYPE prevLineType_local = lineType;

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
				if (prevLineCnt == 2 && (prevSections[1] == DoubleNearRight || prevSections[1] == DoubleNearLeft))
				{
					if (isLeft(lastDoubleLine, SensorOut.lines[0]))
					{
						lineType = SingleLeft;
					}
					else
					{
						lineType = SingleRight;
					}
				}
				else
				{
					lineType = Single;
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
					}
					else
					{
						lineType = DoubleNearRight;
					}
				}
				else
				{
					lineType = DoubleFar;
				}

				break;
			}
			case 3:
			{
				break;
			}
		}

		prevLineCnt = SensorOut.cnt;
	}

	if (prevLinesOk && (lineType == DoubleNearRight || lineType == DoubleNearLeft))
	{
		if (SensorOut.lines[0] - SensorOut.lines[1] > DOUBLE_LINE_HYS_HIGH)
		{
			lineType = DoubleFar;
		}
	}
	else if (prevLinesOk && lineType == DoubleFar)
	{
		if (SensorOut.lines[0] - SensorOut.lines[1] < DOUBLE_LINE_HYS_LOW)
		{
			if (isLeft(SensorOut, prevLine))
			{
				lineType = DoubleNearLeft;
			}
			else
			{
				lineType = DoubleNearRight;
			}
		}
	}

	// Ha változott a vonaltípus, távolságmentés
	if (prevLineType_local != lineType)
	{
		prevTrackLen = getDistance_cm() - sectionStart;
		sectionStart = getDistance_cm();
	}

	return lineType;
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

static void traceLineType(LINE_TYPE ltp)
{
	switch (ltp)
	{
		case Single:
		{
			bspBtSend((uint8_t*)"Single\r\n", 8); // & unemployed
			break;
		}
		case SingleRight:
		{
			bspBtSend((uint8_t*)"Single right\r\n", 14);
			break;
		}
		case SingleLeft:
		{
			bspBtSend((uint8_t*)"Single left\r\n", 13);
			break;
		}
		case DoubleNearRight:
		{
			bspBtSend((uint8_t*)"Double near right\r\n", 19);
			break;
		}
		case DoubleNearLeft:
		{
			bspBtSend((uint8_t*)"Double near left\r\n", 18);
			break;
		}
		case DoubleFar:
		{
			bspBtSend((uint8_t*)"Double far\r\n", 12);
			break;
		}
		case Triple:
		{
			break;
		}
	}
}

static void traceCrossing(CROSSING_TYPE ctype)
{
	switch (ctype)
	{
		case CrossingRtoA:
		{
			bspBtSend((uint8_t*) "> /|\r\n", 6);
			break;
		}
		case CrossingLtoA:
		{
			bspBtSend((uint8_t*) "|\\ <\r\n", 6);
			break;
		}
		case CrossingBtoA_R:
		{
			bspBtSend((uint8_t*) "/| <\r\n", 6);
			break;
		}
		case CrossingBtoA_L:
		{
			bspBtSend((uint8_t*) "> |\\\r\n", 6);
			break;
		}
		case CrossingAtoRB:
		{
			bspBtSend((uint8_t*) "|/\r\n", 4);
			break;
		}
		case CrossingAtoLB:
		{
			bspBtSend((uint8_t*) "\\|\r\n", 4);
			break;
		}
		case ExitForwardRight:
		{
			bspBtSend((uint8_t*) "EFR\r\n", 5);
			break;
		}
		case ExitForwardLeft:
		{
			bspBtSend((uint8_t*) "EFL\r\n", 5);
			break;
		}
		case ExitBackwardRight:
		{
			bspBtSend((uint8_t*) "EBR\r\n", 5);
			break;
		}
		case ExitBackwardLeft:
		{
			bspBtSend((uint8_t*) "EBL\r\n", 5);
			break;
		}
		default:
		{
			break;
		}
	}
}

static int getDistance_cm()
{
	float dst = speedGetDistance();
	dst *= 100; // m -> cm

	return (int) dst;
}

static void traceDistance()
{
	static char distanceBuffer[20];
	int numlen = 0;

	int diff = getDistance_cm() - sectionStart;
	sectionStart = getDistance_cm();

	print_uint32_t(diff, distanceBuffer, &numlen);

	distanceBuffer[numlen] = '\r';
	numlen++;
	distanceBuffer[numlen] = '\n';
	numlen++;

	bspBtSend((uint8_t*)distanceBuffer, numlen);
}

static void traceInt(int i)
{
	static char distanceBuffer[20];
	int numlen = 0;

	print_uint32_t(i, distanceBuffer, &numlen);

	distanceBuffer[numlen] = '\r';
	numlen++;
	distanceBuffer[numlen] = '\n';
	numlen++;

	bspBtSend((uint8_t*)distanceBuffer, numlen);
}

static int smallLines[9];
static int smallLineNum = 0;

static CROSSING_TYPE examineExit(int length, LINE_TYPE ltype)
{
	CROSSING_TYPE ret = NoCrossing;

	if (length <= 20)
	{
		//traceInt(length);

		smallLines[smallLineNum] = length;
		smallLineNum++;

		if (smallLineNum == 9)
		{
			if (ltype == DoubleNearLeft)
			{
				if (smallLines[0] > smallLines[8])
				{
					ret = ExitForwardRight;
				}
				else
				{
					ret = ExitBackwardLeft;
				}
			}
			else if (ltype == DoubleNearRight)
			{
				if (smallLines[0] > smallLines[8])
				{
					ret = ExitForwardLeft;
				}
				else
				{
					ret = ExitBackwardRight;
				}
			}
		}

		smallLineNum %= 9;
	}
	else
	{
		smallLineNum = 0;
	}

	return ret;
}

// END -----------------------------------------------------------------------------------------------------------------

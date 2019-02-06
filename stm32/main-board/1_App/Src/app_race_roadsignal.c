////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_race_roadsignal.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------
#include "app_race_roadsignal.h"
#include "line.h"
#include "bsp_bluetooth.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define LINE_SAMPLES (5u)
#define LINE_VALID   (4u)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	Single,
	Triple
}
LINE_NUM;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static LINE_NUM state = Single;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static LINE_NUM getLineNumFiltered();
static void traceRaceRoadSignal(RACE_RS rstype);

// Global function definitions -----------------------------------------------------------------------------------------

RACE_RS getRaceRs()
{
	LINE_NUM lineNum = getLineNumFiltered();
	RACE_RS ret = None;

	switch (state)
	{
		case Single:
		{
			if (lineNum == Triple)
			{
				state = Triple;
				bspBtSend((uint8_t*)"3\r\n", 3);
			}

			break;
		}
		case Triple:
		{
			if (lineNum == Single)
			{
				state = Single;
				bspBtSend((uint8_t*)"1\r\n", 3);
			}

			break;
		}
		default:
		{
			break;
		}
	}

	return ret;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static LINE_NUM getLineNumFiltered()
{
	//static LINE_NUM prevState;
	static int lineSampleNum[LINE_SAMPLES];
	static int lineSampleIndex;

	int voteWinner = 0;
	LINE_NUM ret;

	LINE_SENSOR_OUT sensorOut = lineGetRawFront();
	int sampleCnt[4] = {0,0,0,0};

	lineSampleNum[lineSampleIndex] = sensorOut.cnt;
	lineSampleIndex++;
	lineSampleIndex %= LINE_SAMPLES;

	for (int i = 0; i < LINE_SAMPLES; i++)
	{
		sampleCnt[lineSampleNum[i]]++;
	}

	for (int i = 0; i < 4; i++)
	{
		if (sampleCnt[i] >= LINE_VALID)
		{
			voteWinner = i;
		}
	}

	switch (voteWinner)
	{
		case 1:
		{
			ret = Single;
			break;
		}
		case 3:
		{
			ret = Triple;
			break;
		}
		default:
		{
			ret = None;
			break;
		}
	}

	return ret;
}

static void traceRaceRoadSignal(RACE_RS rstype)
{
	switch (rstype)
	{
		case Accelerate:
		{
			bspBtSend((uint8_t*) "Accelerate\r\n", 12);
			break;
		}
		case Brake:
		{
			bspBtSend((uint8_t*) "Brake\r\n", 7);
			break;
		}
		default:
		{
			break;
		}
	}
}

// END -----------------------------------------------------------------------------------------------------------------

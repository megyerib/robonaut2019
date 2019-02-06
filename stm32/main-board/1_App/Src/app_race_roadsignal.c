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
#include "speed.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define LINE_SAMPLES (5u)
#define LINE_VALID   (5u)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	Single,
	Triple
}
LINE_NUM;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static RACE_RS rsState = None;
static float prevTrackStart = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static LINE_NUM getLineNumFiltered();
static void traceRaceRoadSignal(RACE_RS rstype);

// Global function definitions -----------------------------------------------------------------------------------------

RACE_RS getRaceRs()
{
	LINE_NUM lineNum = getLineNumFiltered();
	RACE_RS ret = None;
	float diff = speedGetDistance() - prevTrackStart;

	if (lineNum == Triple)
	{
		if (rsState == None)
		{
			rsState = Fast;
			traceRaceRoadSignal(rsState);
			prevTrackStart = speedGetDistance();
		}
		else if (rsState == Slow)
		{
			if (diff > 4.0f)
			{
				rsState = Fast;
				traceRaceRoadSignal(rsState);
				prevTrackStart = speedGetDistance();
			}
		}
		else if (rsState == Fast)
		{
			if (diff > 2.0f)
			{
				rsState = Slow;
				traceRaceRoadSignal(rsState);
				prevTrackStart = speedGetDistance();
			}
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
	static LINE_NUM prevLineNum;

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
			ret = prevLineNum;
			break;
		}
	}

	prevLineNum = ret;

	return ret;
}

static void traceRaceRoadSignal(RACE_RS rstype)
{
	switch (rstype)
	{
		case Fast:
		{
			bspBtSend((uint8_t*) "Fast\r\n", 6);
			break;
		}
		case Slow:
		{
			bspBtSend((uint8_t*) "Slow\r\n", 6);
			break;
		}
		default:
		{
			break;
		}
	}
}

// END -----------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      line.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "line.h"
#include "bsp_uart.h"
#include "line_common.h"
#include "uart_frame.h"
#include <math.h>

// Defines -------------------------------------------------------------------------------------------------------------

#define Y_FRONT (120) /* mm */
#define Y_REAR  (-20) /* mm */

#define BUFMAXLEN 32

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static LINE_SENSOR_OUT front_tmp;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void lineRxStateMachine();

// Global function definitions -----------------------------------------------------------------------------------------

void lineInit()
{
	lineRxStateMachine();
}

LINE lineGet()
{
    LINE_SENSOR_OUT front;
    int16_t x_front = 0;
    int i;

    // Get temporary data
	front = front_tmp;

    // Get center line position
    for (i = 0; i < front.cnt; i++)
        x_front += front.lines[i];
    x_front /= front.cnt;

    LINE ret =
    {
        .d = x_front,
        .theta = 0
    };

    return ret;
}

Arc lineGetArc(uint16_t r_mm, ArcDir dir)
{
    Arc ret;

    return ret;
}

RoadSignal lineGetRoadSignal()
{
    RoadSignal ret = Nothing;

    return ret;
}

// Interrupt handler callbacks -----------------------------

void bspLineFrontRxCpltCallback (void)
{
	lineRxStateMachine();
}

// Local (static) function definitions ---------------------------------------------------------------------------------

UARTFRAME_STATE lineRxState = init;
uint8_t lineRxBuf[UFRAME_BUFMAX];
int     lineRxBufSize;
uint8_t uframeBuf[UFRAME_BUFMAX];
int     uframeBufSize;

int overruns      = 0;
int frameReceived = 0;
int lineReceived  = 0;

static void lineRxStateMachine()
{
	int breakLoop = 0;

	while (!breakLoop)
	{
		switch (lineRxState)
		{
			case init:
			{
				lineRxState = out_resetbuf;

				break;
			}
			case out_resetbuf:
			{
				lineRxBufSize = 0;

				lineRxState = out_rx;
				break;
			}
			case out_rx:
			{
				bspUartReceive_IT(Uart_LineFront, &lineRxBuf[lineRxBufSize], 1);
				lineRxBufSize++;

				lineRxState = out_begincheck;
				breakLoop = 1;
				break;
			}
			case out_begincheck:
			{
				if (
					lineRxBufSize >= 2                          &&
					lineRxBuf[lineRxBufSize - 2] == ESCAPE_CHAR &&
					lineRxBuf[lineRxBufSize - 1] == frameBegin
				)
				{
					lineRxState = in_resetbuf;
				}
				else
				{
					lineRxState = out_ovrcheck;
				}

				break;
			}
			case out_ovrcheck:
			{
				if (lineRxBufSize < UFRAME_BUFMAX)
				{
					lineRxState = out_rx;
				}
				else
				{
					overruns++;
					lineRxState = out_resetbuf;
				}

				break;
			}
			case in_resetbuf:
			{
				lineRxBufSize = 0;

				lineRxState = in_rx;
				break;
			}
			case in_rx:
			{
				bspUartReceive_IT(Uart_LineFront, &lineRxBuf[lineRxBufSize], 1);
				lineRxBufSize++;

				lineRxState = in_endcheck;
				breakLoop = 1;
				break;
			}
			case in_endcheck:
			{
				if (
					lineRxBufSize >= 2                          &&
					lineRxBuf[lineRxBufSize - 2] == ESCAPE_CHAR &&
					lineRxBuf[lineRxBufSize - 1] == frameEnd
				)
				{
					lineRxState = in_process;
				}
				else
				{
					lineRxState = in_ovrcheck;
				}

				break;
			}
			case in_ovrcheck:
			{
				if (lineRxBufSize < UFRAME_BUFMAX)
				{
					lineRxState = in_rx;
				}
				else
				{
					overruns++;
					lineRxState = out_resetbuf;
				}

				break;
			}
			case in_process:
			{
				LINE_SENSOR_OUT* receivedLine;

				frameReceived++;

				convertFromUartFrame(
					lineRxBuf,
					uframeBuf,
					lineRxBufSize,
					&uframeBufSize
				);

				if (uframeBufSize == sizeof(LINE_SENSOR_OUT))
				{
					receivedLine = (LINE_SENSOR_OUT*) uframeBuf;
					front_tmp = *receivedLine;

					lineReceived++;
				}

				lineRxState = out_resetbuf;

				break;
			}
			default:
			{
				lineRxState = out_resetbuf;
				break;
			}
		}
	}
}

// END -----------------------------------------------------------------------------------------------------------------

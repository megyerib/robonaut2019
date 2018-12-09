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
static LINE_SENSOR_OUT rear_tmp;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static LINE Descartes2Polar(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
static void lineRxStateMachine();

// Global function definitions -----------------------------------------------------------------------------------------

void lineInit()
{
	lineRxStateMachine();
}

LINE lineGet()
{
    LINE_SENSOR_OUT front;
    LINE_SENSOR_OUT rear;

    int16_t x_front = 0;
    int16_t x_rear  = 0;

    int i;

    // ---------------------------------

    /*if (valid[0] == 0 || valid[1] == 0)
    {
        LINE ret =
        {
            .d = 0,
            .theta = 0.0f
        };

        return ret;
    }*/

    // Get temporary data
    __disable_irq();
        front = front_tmp;
        rear  = rear_tmp;
    __enable_irq();

    // Get center line position
    for (i = 0; i < front.cnt; i++)
        x_front += front.lines[i];
    x_front /= front.cnt;

    for (i = 0; i < rear.cnt; i++)
        x_rear += rear.lines[i];
    x_rear /= rear.cnt;

    //return Descartes2Polar(x_rear, Y_REAR, x_front, Y_FRONT);

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
    RoadSignal ret = 0;

    return ret;
}

// Interrupt handler callbacks -----------------------------

int frameEnded = 0;
int frameSuccess = 0;

void bspLineFrontRxCpltCallback (void)
{
	lineRxStateMachine();
}

void bspLineRearRxCpltCallback (void)
{

}

// Local (static) function definitions ---------------------------------------------------------------------------------

/*              ^
                | x (y1;y2)
                |/
                /- b
               /|
              /d|
         |   / \|
 --------|th/---+---------->
         | /    |
         |/alpha|
 (x1;x2) x----- |
                |           */

static LINE Descartes2Polar(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    LINE ret;
    float x, y;
    float a, b; // y = ax + b
    float alpha;

    x = (float)x2 - (float)x1;
    y = (float)y2 - (float)y1;

    a = y / x; // tg alpha

    b = (-1 * x1 * a);

    ret.d = ((b/2) * sqrt(1 + 1/(a*a)));

    alpha = atanf(a);

    if (alpha >= 0.0)
    {
        ret.theta = alpha - 90.0;
    }
    else
    {
        ret.theta = alpha + 90.0;
    }

    ret.theta *= 180 / PI; // ° -> rad

    return ret;
}

UARTFRAME_STATE lineRxState = init;
uint8_t lineRxBuf[UFRAME_BUFMAX];
int     lineRxBufSize;

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
					lineRxState = out_resetbuf;
				}

				break;
			}
			case in_process:
			{
				// .....

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

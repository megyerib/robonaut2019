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
static int16_t prev_line = 0;

static UFRAME_RX_STM uframeStm;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void uframeReceive(uint8_t* nextchar);
static void uframeProcess(uint8_t* buf, uint8_t size);

// Global function definitions -----------------------------------------------------------------------------------------

void lineInit()
{
	uframeStm.state = init;
	uframeStm.receive = uframeReceive;
	uframeStm.process = uframeProcess;

	uartFrameRxStm(&uframeStm);
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

	// Lost line
	if (front.cnt == 0)
		ret.d = prev_line;

	prev_line = ret.d;

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

    switch (front_tmp.cnt)
    {
		case 3:
		{
			ret = TripleLine;
			break;
		}
		case 2:
		{
			ret = DoubleLine;
			break;
		}
    }

    return ret;
}

// Interrupt handler callbacks -----------------------------

void bspLineFrontRxCpltCallback (void)
{
	uartFrameRxStm(&uframeStm);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void uframeReceive(uint8_t* nextchar)
{
	bspUartReceive_IT(Uart_LineFront, nextchar, 1);
}

static void uframeProcess(uint8_t* buf, uint8_t size)
{
	if (size == sizeof(LINE_SENSOR_OUT))
	{
		front_tmp = *((LINE_SENSOR_OUT*) buf);
	}
}

// END -----------------------------------------------------------------------------------------------------------------

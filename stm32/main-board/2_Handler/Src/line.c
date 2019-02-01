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

static int16_t prev_line_front = 0; // For racing line only (which is front)

static UFRAME_RX_STM uframeStmFront;
static UFRAME_RX_STM uframeStmRear;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void uframeReceiveFront(uint8_t* nextchar);
static void uframeProcessFront(uint8_t* buf, uint8_t size);
static void uframeReceiveRear(uint8_t* nextchar);
static void uframeProcessRear(uint8_t* buf, uint8_t size);

// Global function definitions -----------------------------------------------------------------------------------------

void lineInit()
{
	uframeStmFront.state = init;
	uframeStmFront.receive = uframeReceiveFront;
	uframeStmFront.process = uframeProcessFront;

	uframeStmRear.state = init;
	uframeStmRear.receive = uframeReceiveRear;
	uframeStmRear.process = uframeProcessRear;

	uartFrameRxStm(&uframeStmFront);
	uartFrameRxStm(&uframeStmRear);
}

float lineGetSingle()
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

    int16_t ret_mm = x_front;

	// Lost line
	if (front.cnt == 0)
		ret_mm = prev_line_front;

	prev_line_front = ret_mm;

    return (ret_mm + 0.0f) / 1000.0f;
}

LINE_SENSOR_OUT lineGetRawFront()
{
	return front_tmp;
}

LINE_SENSOR_OUT lineGetRawRear()
{
	return rear_tmp;
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
	uartFrameRxStm(&uframeStmFront);
}

void bspLineRearRxCpltCallback (void)
{
	uartFrameRxStm(&uframeStmRear);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void uframeReceiveFront(uint8_t* nextchar)
{
	bspUartReceive_IT(Uart_LineFront, nextchar, 1);
}

static void uframeProcessFront(uint8_t* buf, uint8_t size)
{
	if (size == sizeof(LINE_SENSOR_OUT))
	{
		front_tmp = *((LINE_SENSOR_OUT*) buf);
	}
}

static void uframeReceiveRear(uint8_t* nextchar)
{
	bspUartReceive_IT(Uart_LineRear, nextchar, 1);
}

static void uframeProcessRear(uint8_t* buf, uint8_t size)
{
	if (size == sizeof(LINE_SENSOR_OUT))
	{
		rear_tmp = *((LINE_SENSOR_OUT*) buf);
	}
}

// END -----------------------------------------------------------------------------------------------------------------

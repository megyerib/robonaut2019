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
#include "math.h"

#include "line_common.h"
#include "uart_frame.h"

#include <string.h> // Memcpy

// Defines -------------------------------------------------------------------------------------------------------------

#define Y_FRONT (120) /* mm */
#define Y_REAR  (-20) /* mm */

#define BUFMAXLEN 256

// Typedefs ------------------------------------------------------------------------------------------------------------



// Local (static) & extern variables -----------------------------------------------------------------------------------

static uint8_t rxbuf_front[BUFMAXLEN];
static uint8_t rxcnt_front;

static LINE_SENSOR_OUT front_tmp;
static LINE_SENSOR_OUT rear_tmp;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static LINE Descartes2Polar(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

// Global function definitions -----------------------------------------------------------------------------------------

void lineInit()
{
    bspUartReceive_IT(Uart_LineFront, rxbuf_front, 1);
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

void bspLineFrontRxCpltCallback (void)
{
	uint8_t tmp[BUFMAXLEN];
	LINE_SENSOR_OUT* recentLine;
	int tmplen;

	if (rxcnt_front > BUFMAXLEN)
	{
		rxcnt_front = 0; // Something bad happened
	}
	else if (isUartFrameEnded(rxbuf_front, rxcnt_front))
	{
		convertFromUartFrame(rxbuf_front, tmp, rxcnt_front, &tmplen);

		if (tmplen == sizeof(LINE_SENSOR_OUT))
		{
			recentLine = (LINE_SENSOR_OUT*) tmp;

			if (recentLine->cnt != 0)
			{
				front_tmp = *recentLine;
			}
		}
		else
		{
			// Error
		}

		rxcnt_front = 0;
	}
	else
	{
		rxcnt_front++;
	}

	bspUartReceive_IT(Uart_LineFront, &rxbuf_front[rxcnt_front], 1);
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

// END -----------------------------------------------------------------------------------------------------------------

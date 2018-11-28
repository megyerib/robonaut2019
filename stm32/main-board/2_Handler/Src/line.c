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
#include "math.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define FRONT_Y (120) /* mm */
#define REAR_Y  (-20) /* mm */

// Typedefs ------------------------------------------------------------------------------------------------------------



// Local (static) & extern variables -----------------------------------------------------------------------------------

static LINE_SENSOR_OUT sensor_front_out;
static LINE_SENSOR_OUT sensor_rear_out;

static LINE_SENSOR_OUT front_tmp;
static LINE_SENSOR_OUT rear_tmp;

static uint8_t valid[2];

// Local (static) function prototypes ----------------------------------------------------------------------------------

static Line Descartes2Polar(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

// Global function definitions -----------------------------------------------------------------------------------------

void lineInit()
{
    bspUartReceive_IT(Uart_LineFront, (uint8_t*) &front_tmp, sizeof(sensor_front_out));
    bspUartReceive_IT(Uart_LineFront, (uint8_t*) &rear_tmp, sizeof(sensor_rear_out));

    valid[0] = valid[1] = 0;
}

Line lineGet()
{
    Line ret;

    LINE_SENSOR_OUT front;
    LINE_SENSOR_OUT rear;

    int16_t x_front = 0;
    int16_t x_rear  = 0;

    int i;

    // ---------------------------------

    // Get temporary data
    __disable_irq();
        front = sensor_front_out;
        rear  = sensor_rear_out;
    __enable_irq();

    // Get center line position
    for (i = 0; i < front.cnt; i++)
        x_front += front.lines[i];
    x_front /= front.cnt;

    for (i = 0; i < rear.cnt; i++)
        x_rear += rear.lines[i];
    x_rear /= rear.cnt;



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

void bspLineFrontTxCpltCallback (void)
{
    sensor_front_out = front_tmp;

    valid[0] = 1;
}

void bspLineRearTxCpltCallback (void)
{
    sensor_rear_out = rear_tmp;

    valid[1] = 1;
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

static Line Descartes2Polar(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    Line ret;
    float x;
    float y;
    float a; // y = ax + b
    float b;
    float alpha;

    x = (float)x2 - (float)x1;
    y = (float)y2 - (float)y1;

    a = y / x; // tg alpha

    b = (-1 * x1 * a);

    ret.d = ((b/2) * sqrt(1 + 1/(a*a)));

    alpha = atan2f(y, x);

    return ret;
}

// END -----------------------------------------------------------------------------------------------------------------

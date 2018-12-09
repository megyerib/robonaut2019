////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      motor.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "motor.h"
#include "bsp_uart.h"
#include <stdio.h>

/* Defines -------------------------------------------------------------------------------------------------------------
          _  _
       =        =    DIFF_IN_GEAR                       .--.
     =          .--.                                    |--|
    =           |--|____________________________________|--|
    =      O    |--|____________________________________|--| SHAFT_GEAR
    =           |--|                                    |--|
     =          '--'                  _______________   |--|
       =        =                    |               |  :==:
          ~  ~                       |               |__|--|
                                     |   M O T O R   |__|--| MOTOR_GEAR
     DIFF_OUT_GEAR                   |               |  |--|
                                     |_______________|  '--'
*/

#define MOTOR_GEAR     13
#define SHAFT_GEAR     48
#define DIFF_IN_GEAR   13
#define DIFF_OUT_GEAR  38

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static char d_buf[20];

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void motorInit()
{

}

void motorSetTorque(int16_t torqe)
{

}

void motorSetDutyCycle(uint8_t d)
{
    // sprintf screws everything
	d_buf[0] = '0' + ((d / 10) % 10);
    d_buf[1] = '0' + d % 10;
    d_buf[2] = '\r';
    d_buf[3] = '\n';

    bspUartTransmit_IT(Uart_Motor, (uint8_t*) d_buf, 4);
}

// Local (static) function definitions ---------------------------------------------------------------------------------


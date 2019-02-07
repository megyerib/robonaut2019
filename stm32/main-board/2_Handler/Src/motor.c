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

#define MOTOR_D_MAX   (0.85f)

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

void motorSetDutyCycle(float d)
{

	int d_int; // 5 digits max
	int i = 0;

	if (d >= MOTOR_D_MAX)
	{
		d = MOTOR_D_MAX;
	}
	else if (d <= -MOTOR_D_MAX)
	{
		d = -MOTOR_D_MAX;
	}

	d_int = (int)(d * 100000);

	if (d_int < 0)
    {
		d_buf[i] = '-';
		i++;
		d_int *= -1;
    }

	// sprintf screws everything
	d_buf[i] = '0' + ((d_int / 10000) % 10);
	i++;
	d_buf[i] = '0' + ((d_int / 1000) % 10);
	i++;
	d_buf[i] = '0' + ((d_int / 100) % 10);
	i++;
	d_buf[i] = '0' + ((d_int / 10) % 10);
	i++;
    d_buf[i] = '0' + d_int % 10;
    i++;
    d_buf[i] = '\r';
    i++;
    d_buf[i] = '\n';
    i++;

    bspUartTransmit_IT(Uart_Motor, (uint8_t*) d_buf, i);
}

// Local (static) function definitions ---------------------------------------------------------------------------------


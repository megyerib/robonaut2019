////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      radio.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "radio.h"
#include "bsp_uart.h"
#include "usart.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static RadioSignal state = rsNothing;
static uint8_t message;

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

// Local (static) function definitions ---------------------------------------------------------------------------------

void radioInit()
{
	bspUartReceive_IT(Uart_Radio, &message, 1);
}

RadioSignal radioGetState()
{
    return state;
}

void bspRadioRxCpltCallback()
{
    uint8_t newState;

	bspUartReceive_IT(Uart_Radio, &message, 1);

    if (message >= '0' && message <= '5')
    {
        newState = message - '0';
    }
    else
    {
        state = rsError;
    }
}

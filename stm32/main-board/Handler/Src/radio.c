#include "radio.h"

#include "bsp_uart.h"
#include "usart.h"

static RadioSignal state = rsNothing;
static uint8_t message;

static uint8_t debugBuf[2] = "X\n";

void radioInit()
{
	bspUartReceive_IT(&huart6, &message, 1);
}

RadioSignal radioGetState()
{
	return state;
}

void bspRadioRxCpltCallback()
{
	uint8_t newState;

	bspUartReceive_IT(&huart6, &message, 1);

	debugBuf[0] = message;
	bspUartTransmit_IT(&huart2, debugBuf, 2);

	if (message >= '0' && message <= '5')
	{
		newState = message - '0';

		if (newState < state)
		{
			state = newState;
		}
		else
		{
			state = rsError;
		}
	}
	else
	{
		state = rsError;
	}
}

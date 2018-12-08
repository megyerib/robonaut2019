////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      button.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO TODO

// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f4xx_hal.h"
#include "button.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
	GPIO_TypeDef* port;
	uint16_t      pin;
	uint8_t       polarity; // 0: positive; 1: negtive logic

	uint8_t       values;
	uint8_t       state;
	uint8_t       rising;
	uint8_t       falling;
}
BTNINS;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static BTNINS buttons[btnCount]; // Initializes to 0 <- Static

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions /////////////////////////////////////////////////////////////////////////////////////////

void buttonInit()
{
	int i;

	buttons[btnBlue].port = B1_GPIO_Port;
	buttons[btnBlue].pin  = B1_Pin;
	buttons[btnBlue].polarity = 0;

	for (i = 0; i < btnCount; i++)
	{
		buttons[i].values  = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);
		buttons[i].values ^= buttons[i].polarity;
		buttons[i].state   = buttons[i].values; // Initial state is the first read value
	}
}

void buttonTriggerRead()
{
	for (i = 0; i < btnCount; i++)
	{
		// Read
		buttons[i].values << 1;
		buttons[i].values += HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);
		buttons[i].values ^= buttons[i].polarity;

		// Evaluate
		if (buttons[i].values == 0x00) // 0
		{

		}
		else if (buttons[i].values == 0xFF) // 1
		{

		}
	}
}

uint8_t buttonGetState(BTN button)
{

}

uint8_t buttonGetEdge(BTN button, EDGE edge)
{

}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

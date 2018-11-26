////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_communication.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_communication.h"
#include "stm32f0xx_hal.h"
#include "usart.h"
#include "bsp_leds.h"
#include "bsp_pwm.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define MAIN_BOARD_UART &huart1

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

unsigned char response[] = "Got it!\r\n";
int CharCount;
unsigned char ReceiveBufferUART[24];

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void BSP_InitUART()
{
	CharCount = 0;
	HAL_UART_Receive_IT(MAIN_BOARD_UART, &ReceiveBufferUART[CharCount], 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i = 0;

	if (huart == MAIN_BOARD_UART)
	{
		if (ReceiveBufferUART[CharCount] == '\n')
		{

			CharCount = 0;

			int32_t Converted = 0;
			while(ReceiveBufferUART[i] != '\r' && ReceiveBufferUART[i] != '\n')
			{
				Converted *= 10;
				Converted += ReceiveBufferUART[i] - '0';
				i++;
			}

			ConvertedDutyCycle = Converted + 0.0;
			ConvertedDutyCycle /= 100;

			BSP_SetDutyCycle(&ConvertedDutyCycle);
			//BSP_SetLEDHeartbeatBlinkingDutyCyle(&ConvertedDutyCycle);
			HAL_UART_Transmit_IT(MAIN_BOARD_UART, response, 5);
		}
		else
		{
			CharCount++;
		}

		HAL_UART_Receive_IT(MAIN_BOARD_UART, &ReceiveBufferUART[CharCount], 1);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END -----------------------------------------------------------------------------------------------------------------
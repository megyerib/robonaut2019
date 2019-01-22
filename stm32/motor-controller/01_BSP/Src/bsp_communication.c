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
#include "bsp_emergency_stop.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define MAIN_BOARD_UART &huart1

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

unsigned char response[] = "Ping!\r\n";
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
	int valid = 1;

	if (huart == MAIN_BOARD_UART)
	{
		if (ReceiveBufferUART[CharCount] == '\n' || ReceiveBufferUART[CharCount] == '\r')
		{
			if (CharCount > 0)
			{
				int32_t Converted = 0;

				while(ReceiveBufferUART[i] != '\r' && ReceiveBufferUART[i] != '\n')
				{
					if (ReceiveBufferUART[i] < '0' || ReceiveBufferUART[i] > '9')
					{
						valid = 0;
						if (ReceiveBufferUART[i] == '-' && i == 0)		//ha az elso karakter "-", akkor tolatunk
						{
							polarity = NEGATIVE;
							valid = 1;
							i++;										//a kitoltesi tenyezo szamitasbol kihagyjuk a "-" jel ASCI erteket :)
						}
						else
						{
							break;
						}
					}

					Converted *= 10;
					Converted += ReceiveBufferUART[i] - '0';
					i++;
				}

				if (valid)
				{
					BSP_emergency_stop_reset_timer();
					ConvertedDutyCycle = Converted + 0.0;
					ConvertedDutyCycle /= 100;

					BSP_SetDutyCycle(&ConvertedDutyCycle);
					polarity = POSITIVE;							//alapesetben elore megyunk

					HAL_UART_Transmit_IT(MAIN_BOARD_UART, response, 5);
				}

					CharCount = 0;
			}
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

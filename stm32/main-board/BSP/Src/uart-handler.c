#include "uart-handler.h"
#include "usart.h"
#include "radio.h"

extern UART_HandleTypeDef* radioUart = &huart6;
extern UART_HandleTypeDef* pcUart = &huart2;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6)
	{
		radioUartCallback();
	}
}



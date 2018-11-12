#include "uart-handler.h"
#include "usart.h"
#include "radio.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart6)
	{
		radioUartCallback();
	}
}



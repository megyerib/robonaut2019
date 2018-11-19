////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_cdt.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"

#include "app_common.h"
#include "app_cdt.h"
#include "bcm_BluetoothCommunication.h"
#include <stdbool.h>

//TODO _Joci_ delete this
#include "usart.h"
#include "bsp_uart.h"
#include "gpio.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

/*uint8_t	txBtMsg0[16] = "AT+AB FWVersion\r";
uint8_t txBtMsg1[25] = "AT+AB LocalName Override\r";
uint8_t txBtMsg2[19] = "AT+AB GetBDAddress\r";
uint8_t txBtMsg3[16] = "AT+AB Discovery\r";
uint8_t txBtMsg4[30] = "AT+AB SPPConnect 00C2C674C132\r";
uint8_t txBtMsg5[30] = "AT+AB RemoteName 00C2C674C132\r";
uint8_t txBtMsg6[23] = "AT+AB RemoteName [BD Addr\r\n";

static uint8_t rxBtMsg0[100];
static uint8_t rxBtMsg1[100];
static uint8_t rxBtMsg2[100];
static uint8_t rxBtMsg3[200];
static uint8_t rxBtMsg4[100];
static uint8_t rxBtMsg5[100];

uint8_t txEnter[2] = "\r\n";*/
uint8_t sent = 0;
uint8_t flag = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------


//! @brief	Initializes the Task_CarDiagnosticsTool task.
void TaskInit_CarDiagnosticsTool(void)
{
	semBcm = xSemaphoreCreateBinary();

	if(semBcm != NULL)
	{
		xSemaphoreGive(semBcm);
		bcmInit();
	}

	xTaskCreate(Task_CarDiagnosticsTool,
				"TASK_CAR_DIAGNOSTICS_TOOL",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_CDT_PRIO,
				NULL);
}


//! @brief	Communicates with the CDT client application via bluetooth.
void Task_CarDiagnosticsTool(void* p)
{
	(void)p;

	//TODO Feature: configuration
	/*uint8_t bypassMsg[13] = "AT+AB Bypass\r";
	uint8_t bypassResp1[19];
	uint8_t bypassResp2[21];

	uint8_t connectRx[19];
	uint8_t namingMsg[25] = "AT+AB LocalName Override\r";

	bcmResetBluetooth();
	HAL_UART_Abort(&huart5);
	bspUartTransmit_IT(Uart_Bluetooth, namingMsg,  sizeof(namingMsg));
	bspUartReceive(Uart_Bluetooth, 	   connectRx,  sizeof(connectRx), 1000);
	bspUartTransmit_IT(Uart_Bluetooth, bypassMsg, sizeof(bypassMsg));
	bspUartReceive(Uart_Bluetooth, 	   bypassResp2,  sizeof(bypassResp2), 5000);
	bspUartTransmit(Uart_USB, 	bypassMsg, sizeof(bypassMsg), 1000);
	bspUartTransmit(Uart_USB, 	bypassResp2, sizeof(bypassResp2), 1000);*/

	uint8_t helloMsg[18] = "Hello Override\r\n";

	while(1)
	{
		if(sent == 10)
		{
			bcmSend(helloMsg, sizeof(helloMsg));

			sent = 0;
		}
		sent++;


		/*GPIO_PinState pin = HAL_GPIO_ReadPin(BT_CONN_GPIO_Port, BT_CONN_Pin);
		if(pin == GPIO_PIN_SET)
		{
			//bcmReceive(comBuff, 10);
			flag = 1;
		}

		valami = bspUartReceive_IT(Uart_USB, comBuff, sizeof(comBuff));

		if(valami == BSP_OK)
		{
			if(comBuff[0] == '\r')
			{
				bspUartTransmit_IT(Uart_USB, txEnter, sizeof(comBuff));
			}
			else
			{
				bspUartTransmit_IT(Uart_USB, comBuff, sizeof(comBuff));
			}
		}*/

		bcmBtBufferFlush();

		vTaskDelay(100);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------


// THIS ONE WORKS!!!
/*uint8_t uzenet[19] = "AT+AB GetBDAddress\r";
uint8_t valasz[30];

HAL_UART_Transmit_IT(&huart5, uzenet, sizeof(uzenet));
HAL_UART_Receive(&huart5, valasz, sizeof(valasz), 5000);


uint8_t txBtMsgRename[25] = "AT+AB LocalName Override\r";
uint8_t rxBtRename[30];

HAL_UART_Receive_IT(&huart5, rxBtRename, sizeof(rxBtRename));
HAL_UART_Transmit_IT(&huart5, txBtMsgRename, sizeof(txBtMsgRename));*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bcm_BluetoothCommunication.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "semphr.h"

#include "usart.h"
#include "bsp_uart.h"
#include "bcm_BluetoothCommunication.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

SemaphoreHandle_t semBcm;

cBluetoothLog btLog;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void bcmLogGet (cBluetoothLog* const log);

// Global function definitions -----------------------------------------------------------------------------------------

void bcmInit (void)
{
	bspUartInitDevice(Uart_Bluetooth);
	//TODO Earliest need for USB
	bspUartInitDevice(Uart_USB);

	// Reset pin to high -> enable Bluetooth module.
	HAL_GPIO_WritePin(BT_RST_GPIO_Port, BT_RST_Pin, GPIO_PIN_SET);

	bcmTryConnectToCar();
}

void bcmResetBluetooth (void)
{
	HAL_GPIO_WritePin(BT_CONN_GPIO_Port, BT_CONN_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BT_CONN_GPIO_Port, BT_CONN_Pin, GPIO_PIN_SET);
}

bool bcmTryConnectToCar (void)
{
	bool connSuccess = false;

	uint8_t crLfMsg[2] 		   = "\r\n";
	uint8_t welcomeMsg[23]     = "Starting Bluetooth...\r\n";
	uint8_t connectMsg[22]     = "Trying to connect...\r\n";
	uint8_t namingMsg[25]      = "AT+AB LocalName Override\r";
	uint8_t sppConnectMsg[30]  = "AT+AB SPPConnect 00C2C674C132\r";
	uint8_t	carFoundMsg[23]    = "Override car is found\r\n";
	uint8_t btCommReadyMsg[33] = "Bluetooth communication is live\r\n";

	uint8_t connectRx[19];
	uint8_t sppConnectRx[30];

	// Check the connection
	connected = HAL_GPIO_ReadPin(BT_CONN_GPIO_Port, BT_CONN_Pin);

	// Send welcome message
	bspUartTransmit(Uart_USB, crLfMsg,    sizeof(crLfMsg), 1000);
	bspUartTransmit(Uart_USB, welcomeMsg, sizeof(welcomeMsg), 1000);

	// If car connection is not live yet
	if( connected == false)
	{
		//bspUartTransmit(Uart_USB, crLfMsg,    sizeof(crLfMsg), 	  1000);
		bspUartTransmit(Uart_USB, connectMsg, sizeof(connectMsg), 1000);

		bspUartTransmit_IT(Uart_Bluetooth, namingMsg,  sizeof(namingMsg));
		bspUartReceive(Uart_Bluetooth, 	   connectRx,  sizeof(connectRx), 1000);
		bspUartTransmit(Uart_USB, namingMsg, sizeof(namingMsg), 1000);
		bspUartTransmit(Uart_USB, connectRx,  sizeof(connectRx),  1000);

		bspUartTransmit_IT(Uart_Bluetooth, sppConnectMsg, sizeof(sppConnectMsg));
		bspUartReceive(Uart_Bluetooth,     sppConnectRx,  sizeof(sppConnectRx), 2000);
		bspUartTransmit(Uart_USB, sppConnectMsg, sizeof(sppConnectMsg), 1000);
		bspUartTransmit(Uart_USB, sppConnectRx, sizeof(sppConnectRx),   1000);
	}

	// If it was connected, or connected during the previous effort
	if(connected == true)
	{
		bspUartTransmit(Uart_USB, carFoundMsg,    sizeof(carFoundMsg), 	  1000);
		bspUartTransmit(Uart_USB, btCommReadyMsg, sizeof(btCommReadyMsg), 1000);

		connSuccess = true;
	}

	return connSuccess;
}

void bcmBtBufferFlush (void)
{
	uint8_t btBuffer[2+2+BCM_LOG_SIZE+2];
	cBluetoothLog log;
	uint32_t index = 0;
	uint32_t header = 2;
	uint32_t msgSize = 2;

	// Empty the buffer.
	memset(btBuffer, 0, sizeof(btBuffer));

	// Get the bluetooth log.
	bcmLogGet(&log);

	btBuffer[index] = 'A';
	btBuffer[index+1] = 'A';
	index += header;

	hndlPlaceIntegerToAsciiMsg(btBuffer+header, sizeof(btBuffer), msgSize);
	index += msgSize;

	memcpy(btBuffer+index, log.sharpDistant, BCM_LOG_LENGHT_SHARP_DISTANCE);
	index += BCM_LOG_LENGHT_SHARP_DISTANCE;

	memcpy(btBuffer+index, log.sharpCollisionWarning, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);
	index += BCM_LOG_LENGHT_SHARP_COLLISION_WARNING;

	memcpy(btBuffer+index, log.servoAngle, BCM_LOG_LENGHT_SERVO_ANGLE);
	index += BCM_LOG_LENGHT_SERVO_ANGLE;

	btBuffer[index] = '\r';
	btBuffer[index+1] = '\n';

	bspUartTransmit(Uart_Bluetooth, btBuffer, sizeof(btBuffer), 1000);
}

bool bcmLogMemberUpdate (const eBluetoothLogMember member, uint8_t* const array, const uint32_t len)
{
	xSemaphoreTake(semBcm, portMAX_DELAY);

	bool success = true;

	switch (member) {
		case BCM_LOG_SHARP_DISTANCE:
			memcpy(btLog.sharpDistant, array, len);
			break;
		case BCM_LOG_SHARP_COLLISION_WARNING:
			memcpy(btLog.sharpCollisionWarning, array, len);
			break;
		case BCM_LOG_SERVO_ANGLE:
			memcpy(btLog.servoAngle, array, len);
			break;
		default:
			success = false;
			break;
	}
	xSemaphoreGive(semBcm);

	return success;
}

void bcmSend (uint8_t* const txBuffer, const uint16_t length)
{
	bspUartTransmit_IT(Uart_Bluetooth, txBuffer, length);
}

void bcmReceive (uint8_t* const rxBuffer, const uint16_t length)
{
	bspUartReceive_IT(Uart_Bluetooth, rxBuffer, length);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

void bcmLogGet (cBluetoothLog* const log)
{
	xSemaphoreTake(semBcm, portMAX_DELAY);
	memcpy(log->sharpDistant, btLog.sharpDistant, BCM_LOG_LENGHT_SHARP_DISTANCE);
	memcpy(log->sharpCollisionWarning, btLog.sharpCollisionWarning, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);
	memcpy(log->servoAngle, btLog.servoAngle, BCM_LOG_LENGHT_SERVO_ANGLE);
	xSemaphoreGive(semBcm);
}

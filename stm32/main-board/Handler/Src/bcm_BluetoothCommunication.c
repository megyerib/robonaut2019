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

#include <string.h>

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

SemaphoreHandle_t semBcm;

cBluetoothLog btLog;


static uint8_t btTxBuffer[200];
static uint8_t btRxBuffer[200];

static bool usbBusy = false;

// Local (static) function prototypes ----------------------------------------------------------------------------------

bool bcmConvertUintToUintArray (uint8_t* const array,const uint32_t value, const uint32_t len);

uint32_t bcmGetLenght (uint32_t number);

bool bcmLogMemberUpdate (const eBluetoothLogMember member, uint8_t* const array, const uint32_t len);

void bcmLogGet (cBluetoothLog* const log);

// Global function definitions -----------------------------------------------------------------------------------------

void bcmInit (void)
{
	bspUartInitDevice(Uart_Bluetooth);
	//TODO Earliest need for USB
	bspUartInitDevice(Uart_USB);

	// Reset pin to high -> enable Bluetooth module.
	HAL_GPIO_WritePin(BT_RST_GPIO_Port, BT_RST_Pin, GPIO_PIN_SET);

	bcmTryConnectToCar();
	//bspUartTransmit_IT(Uart_Bluetooth, txMsg, sizeof(txMsg));

	//HAL_UART_Transmit(&huart5, txMsg, sizeof(txMsg), HAL_MAX_DELAY);
}

void bcmSend (uint8_t* const txBuffer, const uint16_t length)
{
	bspUartTransmit_IT(Uart_Bluetooth, txBuffer, length);
}

void bcmReceive (uint8_t* const rxBuffer, const uint16_t length)
{
	bspUartReceive_IT(Uart_Bluetooth, rxBuffer, length);
}

void bcmResetBluetooth (void)
{
	HAL_GPIO_WritePin(BT_CONN_GPIO_Port, BT_CONN_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BT_CONN_GPIO_Port, BT_CONN_Pin, GPIO_PIN_SET);
}

void bspUsbTxCpltCallback()
{
	// callback tx
	usbBusy = true;
}

void bspUsbRxCpltCallback()
{
	// callback rx
	usbBusy = false;
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

bool bcmTraceSharpDistance (uint32_t dist)
{
	bool traced = false;
	uint8_t buffer[10];
	uint32_t temp = dist;
	uint32_t length ;
	uint32_t offset;

	memset(buffer, 0, sizeof(buffer));

	//TODO handle saturation
	length = bcmGetLenght(temp);

	offset = BCM_LOG_LENGHT_SHARP_DISTANCE - length;

	bcmConvertUintToUintArray(buffer+offset, temp, length);

	traced = bcmLogMemberUpdate(BCM_LOG_SHARP_DISTANCE, buffer, BCM_LOG_LENGHT_SHARP_DISTANCE);

	return traced;
}

bool bcmTraceSharpCollisionWarning (bool collision)
{
	xSemaphoreTake(semBcm, portMAX_DELAY);
	//btLog.sharpCollisionVarning = collision;
	xSemaphoreGive(semBcm);
}

bool bcmTraceServoAngle (double theta);

void bcmBtBufferFlush (void)
{
	uint8_t btBuffer[BCM_LOG_SIZE+2];
	cBluetoothLog log;
	uint16_t index = 0;
	//uint8_t enter[2] = "\r\n";

	memset(btBuffer, 0, sizeof(btBuffer));

	bcmLogGet(&log);

	memcpy(btBuffer, log.sharpDistant, BCM_LOG_LENGHT_SHARP_DISTANCE);
	index += BCM_LOG_LENGHT_SHARP_DISTANCE;

	memcpy(btBuffer+index, log.sharpCollisionVarning, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);
	index += BCM_LOG_LENGHT_SHARP_COLLISION_WARNING;

	memcpy(btBuffer+index, log.servoAngle, BCM_LOG_LENGHT_SERVO_ANGLE);
	index += BCM_LOG_LENGHT_SERVO_ANGLE;

	btBuffer[index] = '\r';
	btBuffer[index+1] = '\n';

	bspUartTransmit(Uart_Bluetooth, btBuffer, sizeof(btBuffer), 1000);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

bool bcmConvertUintToUintArray (uint8_t* const array, const uint32_t value, const uint32_t len)
{
	bool success = false;
	uint32_t i = 0;
	uint32_t temp;

	if (array != NULL)
	{
		temp = value;
		for(i = len; i > 0; i--)
		{
			//TODO function for conv to ascii
			array[i-1] = temp % 10 + 0x30;
			temp /= 10;
		}

		success = true;
	}

	return success;
}

uint32_t bcmGetLenght (uint32_t number)
{
	uint32_t lenght = 0;
	uint32_t temp = (uint32_t)number;

	while (temp > 0)
	{
		lenght++;
		temp /= 10;
	}

	return lenght;
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
			memcpy(btLog.sharpCollisionVarning, array, len);
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

void bcmLogGet (cBluetoothLog* const log)
{
	xSemaphoreTake(semBcm, portMAX_DELAY);
	memcpy(log->sharpDistant, btLog.sharpDistant, BCM_LOG_LENGHT_SHARP_DISTANCE);
	memcpy(log->sharpCollisionVarning, btLog.sharpCollisionVarning, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);
	memcpy(log->servoAngle, btLog.servoAngle, BCM_LOG_LENGHT_SERVO_ANGLE);
	xSemaphoreGive(semBcm);
}

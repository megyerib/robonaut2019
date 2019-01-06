////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bcm_BluetoothCommunication.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <bsp_bluetooth.h>
#include "usart.h"
#include "bsp_uart.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

cBluetoothLog btLog;

// Local (static) function prototypes ----------------------------------------------------------------------------------

//! Reads out the module structure that holds the bluetooth logs.
//!
//! @param log		this will get the values
static void bcmLogGet (cBluetoothLog* const log);

// Global function definitions -----------------------------------------------------------------------------------------

void bcmInit (void)
{
	bspUartInitDevice(Uart_Bluetooth);
	//TODO Earliest need for USB
	bspUartInitDevice(Uart_USB);

	// Reset pin to high -> enable Bluetooth module.
	HAL_GPIO_WritePin(BT_RST_GPIO_Port, BT_RST_Pin, GPIO_PIN_SET);

	//Empty the structure
	memset(&btLog, 0, sizeof(btLog));

	bcmTryConnectToCar();
}

bool bcmBluetoothConnected (void)
{
	// Check the connection and update flag
	connected = HAL_GPIO_ReadPin(BT_CONN_GPIO_Port, BT_CONN_Pin);

	return connected;
}

void bcmResetBluetooth (void)
{
	// Bluetooth module needs a 5ms long low active pulse to reset
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

	// Connection check.
	connected = bcmBluetoothConnected();

	// Send welcome message
	bspUartTransmit(Uart_USB, crLfMsg,    sizeof(crLfMsg), 1000);
	bspUartTransmit(Uart_USB, welcomeMsg, sizeof(welcomeMsg), 1000);

	// If car connection is not live yet
	if (connected == false)
	{
		//bspUartTransmit(Uart_USB, crLfMsg,    sizeof(crLfMsg), 	  1000);
		bspUartTransmit(Uart_USB, connectMsg, sizeof(connectMsg), 1000);

		bspUartReceive_IT(Uart_Bluetooth, 	   connectRx,  sizeof(connectRx));
		bspUartTransmit(Uart_Bluetooth, namingMsg,  sizeof(namingMsg), 1000);
		bspUartTransmit(Uart_USB, namingMsg, sizeof(namingMsg), 1000);
		bspUartTransmit(Uart_USB, connectRx,  sizeof(connectRx),  1000);

		bspUartReceive_IT(Uart_Bluetooth, sppConnectRx,  sizeof(sppConnectRx));
		bspUartTransmit(Uart_Bluetooth, sppConnectMsg, sizeof(sppConnectMsg), 1000);
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
	uint32_t header = 2;
	uint32_t msgSize = 3;
	uint32_t footer = 2;
	uint32_t bufSize = BCM_LOG_SIZE + header + msgSize + footer;
	uint8_t btBuffer[bufSize];
	cBluetoothLog log;
	uint32_t index = 0;

	// Empty the buffer.
	memset(btBuffer, 0, sizeof(btBuffer));

	// Get the bluetooth log.
	bcmLogGet(&log);

	// Set header.
	btBuffer[index] = 'A';
	btBuffer[index+1] = 'A';
	index += header;

	// Store the length of the whole message.
	hndlPlaceIntegerToAsciiMsg(btBuffer+index, bufSize, msgSize, false);
	index += msgSize;

	// Coppy the structure to the buffer
	memcpy(btBuffer+index, &log, sizeof(log));

	// Set up the footer.
	btBuffer[bufSize-2] = '\r';
	btBuffer[bufSize-1] = '\n';

	// Send out the message
	bspUartTransmit_IT(Uart_Bluetooth, btBuffer, bufSize);
	// TODO for qt debug
	bspUartTransmit_IT(Uart_USB, btBuffer, bufSize);
}

bool bcmLogMemberUpdate (const eBluetoothLogMember member, uint8_t* const array, const uint32_t len)
{
	bool success = true;

	// Save the value into right slot of the structure.
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
		case BCM_LOG_INERT_ACCEL_X:
			memcpy(btLog.inertAccelX, array, len);
			break;
		case BCM_LOG_INERT_ACCEL_Y:
			memcpy(btLog.inertAccelY, array, len);
			break;
		case BCM_LOG_INERT_ACCEL_Z:
			memcpy(btLog.inertAccelZ, array, len);
			break;
		case BCM_LOG_INERT_ANG_VEL_X:
			memcpy(btLog.inertAngVelX, array, len);
			break;
		case BCM_LOG_INERT_ANG_VEL_Y:
			memcpy(btLog.inertAngVelY, array, len);
			break;
		case BCM_LOG_INERT_ANG_VEL_Z:
			memcpy(btLog.inertAngVelZ, array, len);
			break;
		case BCM_LOG_NAVI_N:
			memcpy(btLog.naviN, array, len);
			break;
		case BCM_LOG_NAVI_E:
			memcpy(btLog.naviE, array, len);
			break;
		case BCM_LOG_NAVI_THETA:
			memcpy(btLog.naviTheta, array, len);
			break;
		case BCM_LOG_ENC_VEL:
			memcpy(btLog.encVel, array, len);
			break;
		case BCM_LOG_TOF_1_DISTANCE:
			memcpy(btLog.tof1Distance, array, len);
			break;
		case BCM_LOG_TOF_2_DISTANCE:
			memcpy(btLog.tof2Distance, array, len);
			break;
		case BCM_LOG_TOF_3_DISTANCE:
			memcpy(btLog.tof3Distance, array, len);
			break;
		case BCM_LOG_MTR_MAIN_BAT_VOLT:
			memcpy(btLog.mtrMainBatVolt, array, len);
			break;
		case BCM_LOG_MTR_SEC_BAT_VOLT:
			memcpy(btLog.mtrSecBatVolt, array, len);
			break;
		case BCM_LOG_MTR_CURR:
			memcpy(btLog.mtrCurr, array, len);
			break;
		case BCM_LOG_MTR_SYS_CURR:
			memcpy(btLog.mtrSysCurr, array, len);
			break;
		case BCM_LOG_MTR_SRV_CURR:
			memcpy(btLog.mtrSrvCurr, array, len);
			break;
		case BCM_LOG_MTR_CMD_STOP_ENGINE:
			memcpy(btLog.mtrCmdStopEngine, array, len);
			break;
		case BCM_LOG_CTR_MTR_CURR:
			memcpy(btLog.ctrMtrCurr, array, len);
			break;
		default:
			success = false;
			break;
	}

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
	// Copy the log structure into the pointer
	memcpy(log, &btLog, sizeof(btLog));
}

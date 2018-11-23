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

	memset(btLog.sharpDistant, 0, BCM_LOG_LENGHT_SHARP_DISTANCE);
	memset(btLog.sharpCollisionWarning, 0, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);
	memset(btLog.servoAngle, 0, BCM_LOG_LENGHT_SERVO_ANGLE);
	memset(btLog.inertAccelX, 0, BCM_LOG_LENGHT_INERT_ACCEL_X);
	memset(btLog.inertAccelY, 0, BCM_LOG_LENGHT_INERT_ACCEL_Y);
	memset(btLog.inertAccelZ, 0, BCM_LOG_LENGHT_INERT_ACCEL_Z);
	memset(btLog.inertAngVelX, 0, BCM_LOG_LENGHT_INERT_ANG_VEL_X);
	memset(btLog.inertAngVelY, 0, BCM_LOG_LENGHT_INERT_ANG_VEL_Y);
	memset(btLog.inertAngVelZ, 0, BCM_LOG_LENGHT_INERT_ANG_VEL_Z);
	memset(btLog.naviN, 0, BCM_LOG_LENGHT_NAVI_E);
	memset(btLog.naviE, 0, BCM_LOG_LENGHT_NAVI_E);
	memset(btLog.naviTheta, 0, BCM_LOG_LENGHT_NAVI_THETA);
	memset(btLog.encVel, 0, BCM_LOG_LENGHT_ENC_VEL);
	memset(btLog.tof1Distance, 0, BCM_LOG_LENGHT_TOF_1_DISTANCE);
	memset(btLog.tof2Distance, 0, BCM_LOG_LENGHT_TOF_2_DISTANCE);
	memset(btLog.tof3Distance, 0, BCM_LOG_LENGHT_TOF_3_DISTANCE);
	memset(btLog.mtrMainBatVolt, 0, BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT);
	memset(btLog.mtrSecBatVolt, 0, BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT);
	memset(btLog.mtrCurr, 0, BCM_LOG_LENGHT_MTR_CURR);
	memset(btLog.mtrSysCurr, 0, BCM_LOG_LENGHT_MTR_SYS_CURR);
	memset(btLog.mtrSrvCurr, 0, BCM_LOG_LENGHT_MTR_SRV_CURR);
	memset(btLog.mtrCmdStopEngine, 0, BCM_LOG_LENGHT_MTR_CMD_STOP_ENGINE);
	memset(btLog.ctrMtrCurr, 0, BCM_LOG_LENGHT_CTRL_MTR_CURR);

	bcmTryConnectToCar();
}

bool bcmBluetoothConnected (void)
{
	// Check the connection
	return HAL_GPIO_ReadPin(BT_CONN_GPIO_Port, BT_CONN_Pin);
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

	btBuffer[index] = 'A';
	btBuffer[index+1] = 'A';
	index += header;

	hndlPlaceIntegerToAsciiMsg(btBuffer+index, bufSize, msgSize, false);
	index += msgSize;

	memcpy(btBuffer+index, log.sharpDistant, BCM_LOG_LENGHT_SHARP_DISTANCE);
	index += BCM_LOG_LENGHT_SHARP_DISTANCE;

	memcpy(btBuffer+index, log.sharpCollisionWarning, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);
	index += BCM_LOG_LENGHT_SHARP_COLLISION_WARNING;

	memcpy(btBuffer+index, log.servoAngle, BCM_LOG_LENGHT_SERVO_ANGLE);
	index += BCM_LOG_LENGHT_SERVO_ANGLE;

	memcpy(btBuffer+index, log.inertAccelX, BCM_LOG_LENGHT_INERT_ACCEL_X);
	index += BCM_LOG_LENGHT_INERT_ACCEL_X;

	memcpy(btBuffer+index, log.inertAccelY, BCM_LOG_LENGHT_INERT_ACCEL_Y);
	index += BCM_LOG_LENGHT_INERT_ACCEL_Y;

	memcpy(btBuffer+index, log.inertAccelZ, BCM_LOG_LENGHT_INERT_ACCEL_Z);
	index += BCM_LOG_LENGHT_INERT_ACCEL_Z;

	memcpy(btBuffer+index, log.inertAngVelX, BCM_LOG_LENGHT_INERT_ANG_VEL_X);
	index += BCM_LOG_LENGHT_INERT_ANG_VEL_X;

	memcpy(btBuffer+index, log.inertAngVelY, BCM_LOG_LENGHT_INERT_ANG_VEL_Y);
	index += BCM_LOG_LENGHT_INERT_ANG_VEL_Y;

	memcpy(btBuffer+index, log.inertAngVelZ, BCM_LOG_LENGHT_INERT_ANG_VEL_Z);
	index += BCM_LOG_LENGHT_INERT_ANG_VEL_Z;

	memcpy(btBuffer+index, log.naviN, BCM_LOG_LENGHT_NAVI_N);
	index += BCM_LOG_LENGHT_NAVI_N;

	memcpy(btBuffer+index, log.naviE, BCM_LOG_LENGHT_NAVI_E);
	index += BCM_LOG_LENGHT_NAVI_E;

	memcpy(btBuffer+index, log.naviTheta, BCM_LOG_LENGHT_NAVI_THETA);
	index += BCM_LOG_LENGHT_NAVI_THETA;

	memcpy(btBuffer+index, log.encVel, BCM_LOG_LENGHT_ENC_VEL);
	index += BCM_LOG_LENGHT_ENC_VEL;

	memcpy(btBuffer+index, log.tof1Distance, BCM_LOG_LENGHT_TOF_1_DISTANCE);
	index += BCM_LOG_LENGHT_TOF_1_DISTANCE;

	memcpy(btBuffer+index, log.tof2Distance, BCM_LOG_LENGHT_TOF_2_DISTANCE);
	index += BCM_LOG_LENGHT_TOF_2_DISTANCE;

	memcpy(btBuffer+index, log.tof3Distance, BCM_LOG_LENGHT_TOF_3_DISTANCE);
	index += BCM_LOG_LENGHT_TOF_3_DISTANCE;

	memcpy(btBuffer+index, log.mtrMainBatVolt, BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT);
	index += BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT;

	memcpy(btBuffer+index, log.mtrSecBatVolt, BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT);
	index += BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT;

	memcpy(btBuffer+index, log.mtrCurr, BCM_LOG_LENGHT_MTR_CURR);
	index += BCM_LOG_LENGHT_MTR_CURR;

	memcpy(btBuffer+index, log.mtrSysCurr, BCM_LOG_LENGHT_MTR_SYS_CURR);
	index += BCM_LOG_LENGHT_MTR_SYS_CURR;

	memcpy(btBuffer+index, log.mtrSrvCurr, BCM_LOG_LENGHT_MTR_SRV_CURR);
	index += BCM_LOG_LENGHT_MTR_SRV_CURR;

	memcpy(btBuffer+index, log.mtrCmdStopEngine, BCM_LOG_LENGHT_MTR_CMD_STOP_ENGINE);
	index += BCM_LOG_LENGHT_MTR_CMD_STOP_ENGINE;

	memcpy(btBuffer+index, log.ctrMtrCurr, BCM_LOG_LENGHT_CTRL_MTR_CURR);
	index += BCM_LOG_LENGHT_CTRL_MTR_CURR;

	btBuffer[bufSize-2] = '\r';
	btBuffer[bufSize-1] = '\n';

	bspUartTransmit_IT(Uart_Bluetooth, btBuffer, bufSize);
}

bool bcmLogMemberUpdate (const eBluetoothLogMember member, uint8_t* const array, const uint32_t len)
{
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
	memcpy(log->sharpDistant, btLog.sharpDistant, BCM_LOG_LENGHT_SHARP_DISTANCE);
	memcpy(log->sharpCollisionWarning, btLog.sharpCollisionWarning, BCM_LOG_LENGHT_SHARP_COLLISION_WARNING);
	memcpy(log->servoAngle, btLog.servoAngle, BCM_LOG_LENGHT_SERVO_ANGLE);
	memcpy(log->inertAccelX, btLog.inertAccelX, BCM_LOG_LENGHT_INERT_ACCEL_X);
	memcpy(log->inertAccelY, btLog.inertAccelY, BCM_LOG_LENGHT_INERT_ACCEL_Y);
	memcpy(log->inertAccelZ, btLog.inertAccelZ, BCM_LOG_LENGHT_INERT_ACCEL_Z);
	memcpy(log->inertAngVelX, btLog.inertAngVelX, BCM_LOG_LENGHT_INERT_ANG_VEL_X);
	memcpy(log->inertAngVelY, btLog.inertAngVelY, BCM_LOG_LENGHT_INERT_ANG_VEL_Y);
	memcpy(log->inertAngVelZ, btLog.inertAngVelZ, BCM_LOG_LENGHT_INERT_ANG_VEL_Z);
	memcpy(log->naviN, btLog.naviN, BCM_LOG_LENGHT_NAVI_N);
	memcpy(log->naviE, btLog.naviE, BCM_LOG_LENGHT_NAVI_E);
	memcpy(log->naviTheta, btLog.naviTheta, BCM_LOG_LENGHT_NAVI_THETA);
	memcpy(log->encVel, btLog.encVel, BCM_LOG_LENGHT_ENC_VEL);
	memcpy(log->tof1Distance, btLog.tof1Distance, BCM_LOG_LENGHT_TOF_1_DISTANCE);
	memcpy(log->tof2Distance, btLog.tof2Distance, BCM_LOG_LENGHT_TOF_2_DISTANCE);
	memcpy(log->tof3Distance, btLog.tof3Distance, BCM_LOG_LENGHT_TOF_3_DISTANCE);
	memcpy(log->mtrMainBatVolt, btLog.mtrMainBatVolt, BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT);
	memcpy(log->mtrSecBatVolt, btLog.mtrSecBatVolt, BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT);
	memcpy(log->mtrCurr, btLog.mtrCurr, BCM_LOG_LENGHT_MTR_CURR);
	memcpy(log->mtrSysCurr, btLog.mtrSysCurr, BCM_LOG_LENGHT_MTR_SYS_CURR);
	memcpy(log->mtrSrvCurr, btLog.mtrSrvCurr, BCM_LOG_LENGHT_MTR_SRV_CURR);
	memcpy(log->mtrCmdStopEngine, btLog.mtrCmdStopEngine, BCM_LOG_LENGHT_MTR_CMD_STOP_ENGINE);
	memcpy(log->ctrMtrCurr, btLog.ctrMtrCurr, BCM_LOG_LENGHT_CTRL_MTR_CURR);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_cdt.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_cdt.h"

#include "app_common.h"
#include "trace.h"
#include "bsp_uart.h"
#include "servo.h"
#include "sharp.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

QueueHandle_t qSharpDistance_u32;		// 1
QueueHandle_t qSharpCollWarn_x;
QueueHandle_t qServoAngle_d;
QueueHandle_t qInertAccelX_d;
QueueHandle_t qInertAccelY_d;
QueueHandle_t qInertAccelZ_d;
QueueHandle_t qInertAngVelX_d;
QueueHandle_t qInertAngVelY_d;
QueueHandle_t qInertAngVelZ_d;
QueueHandle_t qNaviN_d;					// 10
QueueHandle_t qNaviE_d;
QueueHandle_t qNaviTheta_d;
QueueHandle_t qEncVel_d;
QueueHandle_t qTof1Distance_u32;
QueueHandle_t qTof2Distance_u32;
QueueHandle_t qTof3Distance_u32;
QueueHandle_t qMtrMainBatVolt_d;
QueueHandle_t qMtrSecBatVolt_d;
QueueHandle_t qMtrCurr_d;
QueueHandle_t qMtrSysCurr_u32;			// 20
QueueHandle_t qMtrSrvCurr_u32;
QueueHandle_t qMtrCmdStopEngine_x;
QueueHandle_t qCtrlMtrCurr_d;
QueueHandle_t qLineD_u32;
QueueHandle_t qLineTheta_u32;

uint8_t btRxBuffer[TRACE_REC_MSG_SIZE];

QueueHandle_t qRecData;
cTraceRxBluetoothStruct recData;
bool btReceived = false;
double cntr = 0;

// USB______________________________________

bool usbSent = false;
bool usbRec = false;

extern UART_HandleTypeDef huart5;
uint8_t usbRxBuffer[TRACE_REC_MSG_SIZE];
uint8_t usbTxBuffer[BCM_LOG_SIZE+5];
cMEASUREMENT_DIST sharp;
double servo = 60;
double encVel = 0;
double motor = 0;

cTraceRxBluetoothStruct usbStruct;

// END___________________________-

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------


//! @brief	Initializes the Task_CarDiagnosticsTool task.
void TaskInit_CarDiagnosticsTool(void)
{
	traceInit();

	qSharpDistance_u32  = xQueueCreate( 1, sizeof( uint32_t ) );	// 1
	qSharpCollWarn_x    = xQueueCreate( 1, sizeof( bool ) );
	qServoAngle_d       = xQueueCreate( 1, sizeof( double ) );
	qInertAccelX_d	    = xQueueCreate( 1, sizeof( double ) );
	qInertAccelY_d 	    = xQueueCreate( 1, sizeof( double ) );
	qInertAccelZ_d	    = xQueueCreate( 1, sizeof( double ) );
	qInertAngVelX_d	    = xQueueCreate( 1, sizeof( double ) );
	qInertAngVelY_d     = xQueueCreate( 1, sizeof( double ) );
	qInertAngVelZ_d	    = xQueueCreate( 1, sizeof( double ) );
	qNaviN_d		    = xQueueCreate( 1, sizeof( double ) );		// 10
	qNaviE_d		    = xQueueCreate( 1, sizeof( double ) );
	qNaviTheta_d	    = xQueueCreate( 1, sizeof( double ) );
	qEncVel_d		    = xQueueCreate( 1, sizeof( double ) );
	qTof1Distance_u32   = xQueueCreate( 1, sizeof( uint32_t ) );
	qTof2Distance_u32   = xQueueCreate( 1, sizeof( uint32_t ) );
	qTof3Distance_u32   = xQueueCreate( 1, sizeof( uint32_t ) );
	qMtrMainBatVolt_d   = xQueueCreate( 1, sizeof( double ) );
	qMtrSecBatVolt_d    = xQueueCreate( 1, sizeof( double ) );
	qMtrCurr_d		    = xQueueCreate( 1, sizeof( double ) );
	qMtrSysCurr_u32	    = xQueueCreate( 1, sizeof( uint32_t ) );	// 20
	qMtrSrvCurr_u32     = xQueueCreate( 1, sizeof( uint32_t ) );
	qMtrCmdStopEngine_x = xQueueCreate( 1, sizeof( bool ) );
	qCtrlMtrCurr_d		= xQueueCreate( 1, sizeof( double ) );
	qLineD_u32			= xQueueCreate( 1, sizeof( uint32_t ) );
	qLineTheta_u32		= xQueueCreate( 1, sizeof( uint32_t ) );

	// Atollic debug queues
	vQueueAddToRegistry(qSharpDistance_u32, "SharpDistance");
	vQueueAddToRegistry(qSharpCollWarn_x,   "SharpColWarn");
	vQueueAddToRegistry(qEncVel_d, 		    "EncVel");
	vQueueAddToRegistry(qTof1Distance_u32,  "Tof1Dist");
	vQueueAddToRegistry(qTof2Distance_u32,  "Tof2Dist");
	vQueueAddToRegistry(qTof3Distance_u32,  "Tof3Dist");
	vQueueAddToRegistry(qMtrSrvCurr_u32,    "MtrSrvCurr");
	vQueueAddToRegistry(qCtrlMtrCurr_d,     "CtrlMtrCurr");

	qRecData = xQueueCreate( 1, sizeof( cTraceRxBluetoothStruct ) );

	vQueueAddToRegistry(qRecData, "RecData");

	sharpTriggerAdc();

	//TODO
	sharp.Distance = 40;

	xTaskCreate(Task_CarDiagnosticsTool,
				"TASK_CAR_DIAGNOSTICS_TOOL",
				DEFAULT_STACK_SIZE+180 ,
				NULL,
				TASK_CDT_PRIO,
				NULL);
}


//! @brief	Communicates with the CDT client application via bluetooth.
void Task_CarDiagnosticsTool(void* p)
{
	(void)p;

	bspUartReceive_IT(Uart_USB, usbRxBuffer, TRACE_REC_MSG_SIZE);
	bspUartReceive_IT(Uart_Bluetooth, btRxBuffer, TRACE_REC_MSG_SIZE);


	sharp = sharpGetMeasurement();

	while(1)
	{
		if (true)//bcmBluetoothConnected())
		{
			if (usbRec == true)
			{
				usbRec = false;

				usbStruct = traceProcessRxData(usbRxBuffer);
			}

			if (btReceived == true)
			{
				btReceived = false;

				recData = traceProcessRxData(btRxBuffer);

				xQueueOverwrite(qRecData, (void*) &recData);

				btReceived = false;

				// TODO debug?
				bspUartReceive_IT(Uart_Bluetooth, btRxBuffer, TRACE_REC_MSG_SIZE);
				bspUartTransmit_IT(Uart_USB, usbTxBuffer, TRACE_REC_MSG_SIZE);
			}

			// SEND bt and USB TODO
			traceFlushData();

			if(recData.RecCmdAccelerate && recData.RecDataAccelerate >= 0 && recData.RecDataAccelerate < 100)
			{
				motor = (double)recData.RecDataAccelerate;
//				xQueueOverwrite(qAlkalmazasDemoMotor, &motor);
				recData.RecCmdAccelerate = false;
			}

			if(recData.RecCmdSteer && recData.RecDataSteer >= 0 && recData.RecDataSteer < 180)
			{
				servo = (double)recData.RecDataSteer * 3.14/180 - 3.14/2;
				servoSetAngle(servo);
				recData.RecCmdSteer = false;
			}

			sharp = sharpGetMeasurement();

			if(sharp.Distance < 40)
			{
				uint8_t motor_stop = 0;
				traceBluetooth(BCM_LOG_SHARP_COLLISION_WARNING, true);
	//			xQueueOverwrite(qAlkalmazasDemoMotor, &motor_stop);
			}
			else
			{
				traceBluetooth(BCM_LOG_SHARP_COLLISION_WARNING, true);
			}

			traceBluetooth(BCM_LOG_SHARP_DISTANCE, &sharp);
			traceBluetooth(BCM_LOG_ENC_VEL, &motor);
			traceBluetooth(BCM_LOG_SERVO_ANGLE, &servo);

			// TODO debug?
			bspUartReceive_IT(Uart_Bluetooth, btRxBuffer, TRACE_REC_MSG_SIZE);
			bspUartReceive_IT(Uart_USB, usbRxBuffer, TRACE_REC_MSG_SIZE);
		}


		if(recData.RecDataSteer > 0 && recData.RecDataSteer < 180)
		{
			servo = (double)usbStruct.RecDataSteer;
			servoSetAngle(usbStruct.RecDataSteer);
		}


		cntr++;

		sharpTriggerAdc();


		traceBluetooth(BCM_LOG_SERVO_ANGLE, &servo);
		traceBluetooth(BCM_LOG_ENC_VEL, &encVel);
		traceBluetooth(BCM_LOG_SHARP_DISTANCE, &sharp);

		vTaskDelay(200);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

void bspUsbTxCpltCallback()
{
	usbSent = true;
}

void bspUsbRxCpltCallback ()
{
	usbRec = true;
}

void bspBluetoothRxCpltCallback ()
{
	btReceived = true;

	//memcpy(usbRxBuffer, btRxBuffer, TRACE_REC_MSG_SIZE);
}

// THIS ONE WORKS!!!
/*uint8_t uzenet[19] = "AT+AB GetBDAddress\r";
uint8_t valasz[30];

HAL_UART_Transmit_IT(&huart5, uzenet, sizeof(uzenet));
HAL_UART_Receive(&huart5, valasz, sizeof(valasz), 5000);


uint8_t txBtMsgRename[25] = "AT+AB LocalName Override\r";
uint8_t rxBtRename[30];

HAL_UART_Receive_IT(&huart5, rxBtRename, sizeof(rxBtRename));
HAL_UART_Transmit_IT(&huart5, txBtMsgRename, sizeof(txBtMsgRename));*/

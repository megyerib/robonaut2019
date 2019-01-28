////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_cdt.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <bsp_servo.h>
#include <bsp_sharp.h>
#include "app_cdt.h"

#include "app_common.h"
#include "trace.h"
#include "bsp_uart.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

QueueHandle_t qNaviN_f;
QueueHandle_t qNaviE_f;
QueueHandle_t qNaviPSI_f;
QueueHandle_t qEncV_f;
QueueHandle_t qDistToF1_u32;
QueueHandle_t qDistToF2_u32;
QueueHandle_t qDistToF3_u32;
QueueHandle_t qDistSharp1_u32;
QueueHandle_t qInertAccelX_f;
QueueHandle_t qInertAccelY_f;
QueueHandle_t qInertAccelZ_f;
QueueHandle_t qInertAngVelX_f;
QueueHandle_t qInertAngVelY_f;
QueueHandle_t qInertAngVelZ_f;
QueueHandle_t qSteerWheelAngle_f;
QueueHandle_t qServoAngle_f;

QueueHandle_t qMtrMainBatVolt_f;
QueueHandle_t qMtrSecBatVolt_f;
QueueHandle_t qMtrCurr_f;
QueueHandle_t qMtrSysCurr_u32;
QueueHandle_t qMtrServoCurr_u32;

QueueHandle_t qLineLineNbr_u32;
QueueHandle_t qLineMainLinePos_f;
QueueHandle_t qLineSecLinePos_f;

QueueHandle_t qMazeMainSM_u32;
QueueHandle_t qMazeGetKp_f;
QueueHandle_t qMazeGetKd_f;
QueueHandle_t qMazeGetSpeed_u32;
QueueHandle_t qMazeSegments_u32;
QueueHandle_t qMazeActState_u32;
QueueHandle_t qMazeActKp_f;
QueueHandle_t qMazeActKd_f;
QueueHandle_t qMazeActSpeed_u32;
QueueHandle_t qMazeInclinSegment_u32;

QueueHandle_t qSRunMainSM_u32;
QueueHandle_t qSRunActState_u32;
QueueHandle_t qSRunActP_f;
QueueHandle_t qSRunActKp_f;
QueueHandle_t qSRunActKd_f;
QueueHandle_t qSRunActSpeed_u32;
QueueHandle_t qSRunGetP_f;
QueueHandle_t qSRunGetKp_f;
QueueHandle_t qSRunGetKd_f;
QueueHandle_t qSRunGetSpeed_u32;

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
uint8_t usbTxBuffer[BT_LOG_SIZE+5];
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

	qNaviN_f       			= xQueueCreate( 1, sizeof( float ) );		// 1
	qNaviE_f       			= xQueueCreate( 1, sizeof( float ) );
	qNaviPSI_f     		 	= xQueueCreate( 1, sizeof( float ) );
	qEncV_f  				= xQueueCreate( 1, sizeof( float ) );
	qDistToF1_u32   	 	= xQueueCreate( 1, sizeof( uint32_t ) );
	qDistToF2_u32       	= xQueueCreate( 1, sizeof( uint32_t ) );
	qDistToF3_u32	    	= xQueueCreate( 1, sizeof( uint32_t ) );
	qDistSharp1_u32 	    = xQueueCreate( 1, sizeof( uint32_t ) );
	qInertAccelX_f		    = xQueueCreate( 1, sizeof( float ) );
	qInertAccelY_f	   		= xQueueCreate( 1, sizeof( float ) );		// 10
	qInertAccelZ_f	    	= xQueueCreate( 1, sizeof( float ) );
	qInertAngVelX_f	   		= xQueueCreate( 1, sizeof( float ) );
	qInertAngVelY_f     	= xQueueCreate( 1, sizeof( float ) );
	qInertAngVelZ_f	    	= xQueueCreate( 1, sizeof( float ) );
	qSteerWheelAngle_f	    = xQueueCreate( 1, sizeof( float ) );
	qServoAngle_f		    = xQueueCreate( 1, sizeof( float ) );		// 16

	qMtrMainBatVolt_f	    = xQueueCreate( 1, sizeof( float ) );		// 17
	qMtrSecBatVolt_f   		= xQueueCreate( 1, sizeof( float ) );
	qMtrCurr_f		    	= xQueueCreate( 1, sizeof( float ) );
	qMtrSysCurr_u32	    	= xQueueCreate( 1, sizeof( uint32_t ) );
	qMtrServoCurr_u32     	= xQueueCreate( 1, sizeof( uint32_t ) );	// 21

	qLineLineNbr_u32		= xQueueCreate( 1, sizeof( uint32_t ) );	// 22
	qLineMainLinePos_f		= xQueueCreate( 1, sizeof( float ) );
	qLineSecLinePos_f		= xQueueCreate( 1, sizeof( float ) );		// 24

	qMazeMainSM_u32		    = xQueueCreate( 1, sizeof( uint32_t ) );	// 25
	qMazeGetKp_f		    = xQueueCreate( 1, sizeof( float ) );
	qMazeGetKd_f		    = xQueueCreate( 1, sizeof( float ) );
	qMazeGetSpeed_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );
	qMazeSegments_u32		= xQueueCreate( 1, sizeof( uint32_t ) );
	qMazeActState_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );	// 30
	qMazeActKp_f		    = xQueueCreate( 1, sizeof( float ) );
	qMazeActKd_f		    = xQueueCreate( 1, sizeof( float ) );
	qMazeActSpeed_u32  		= xQueueCreate( 1, sizeof( uint32_t ) );
	qMazeInclinSegment_u32  = xQueueCreate( 1, sizeof( uint32_t ) );	// 34

	qSRunMainSM_u32   		= xQueueCreate( 1, sizeof( uint32_t ) );		// 35
	qSRunActState_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );
	qSRunActP_f		    	= xQueueCreate( 1, sizeof( float ) );
	qSRunActKp_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunActKd_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunActSpeed_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );
	qSRunGetP_f		    	= xQueueCreate( 1, sizeof( float ) );
	qSRunGetKp_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunGetKd_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunGetSpeed_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );		// 44

	// Atollic debug queues
	/*vQueueAddToRegistry(qNaviN_f,		   "qNaviN_f");
	vQueueAddToRegistry(qNaviE_f,  		   "qNaviE_f");
	vQueueAddToRegistry(qNaviPhi_f,   	   "qNaviPhi_f");
	vQueueAddToRegistry(qEncVel_f, 		   "EncVel");
	vQueueAddToRegistry(qTof1Distance_u32, "Tof1Dist");
	vQueueAddToRegistry(qTof2Distance_u32, "Tof2Dist");
	vQueueAddToRegistry(qTof3Distance_u32, "Tof3Dist");
	vQueueAddToRegistry(qMtrSrvCurr_u32,   "MtrSrvCurr");
	vQueueAddToRegistry(qCtrlMtrCurr_f,    "CtrlMtrCurr");*/

	qRecData = xQueueCreate( 1, sizeof( cTraceRxBluetoothStruct ) );

	vQueueAddToRegistry(qRecData, "RecData");

	// TODO sharpTriggerAdc();

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

	uint8_t btSentMessage[BT_LOG_SIZE+7];

	while (1)
	{
		traceFlushData();
		bspUartTransmit_IT(Uart_USB, btSentMessage, BT_LOG_SIZE+7);

		vTaskDelay(200);
	}

/*	sharp = sharpGetMeasurement();

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
				//uint8_t motor_stop = 0;
				traceBluetooth(BCM_LOG_SHARP_COLLISION_WARNING, (void*) true);
	//			xQueueOverwrite(qAlkalmazasDemoMotor, &motor_stop);
			}
			else
			{
				traceBluetooth(BCM_LOG_SHARP_COLLISION_WARNING, (void*) true);
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
	}*/
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_cdt.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../1_App/Inc/app_cdt.h"

#include "../../1_App/Inc/app_common.h"
#include "../../2_Handler/Inc/trace.h"
#include "../../3_BSP/Inc/bsp_uart.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

QueueHandle_t qSharpDistance_u32;	// 1
QueueHandle_t qSharpCollWarn_x;
QueueHandle_t qServoAngle_d;
QueueHandle_t qInertAccelX_d;
QueueHandle_t qInertAccelY_d;
QueueHandle_t qInertAccelZ_d;
QueueHandle_t qInertAngVelX_d;
QueueHandle_t qInertAngVelY_d;
QueueHandle_t qInertAngVelZ_d;
QueueHandle_t qNaviN_d;				// 10
QueueHandle_t qNaviE_d;
QueueHandle_t qNaviTheta_d;
QueueHandle_t qEncVel_d;
QueueHandle_t qTof1Distance_u32;
QueueHandle_t qTof2Distance_u32;
QueueHandle_t qTof3Distance_u32;
QueueHandle_t qMtrMainBatVolt_d;
QueueHandle_t qMtrSecBatVolt_d;
QueueHandle_t qMtrCurr_d;
QueueHandle_t qMtrSysCurr_u32;		// 20
QueueHandle_t qMtrSrvCurr_u32;
QueueHandle_t qMtrCmdStopEngine_x;
QueueHandle_t qCtrlMtrCurr_d;

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
	bcmInit();

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

	// Atollic debug queues
	vQueueAddToRegistry(qSharpDistance_u32, "SharpDistance");
	vQueueAddToRegistry(qSharpCollWarn_x, "SharpColWarn");
	vQueueAddToRegistry(qEncVel_d, "EncVel");
	vQueueAddToRegistry(qTof1Distance_u32, "Tof1Dist");
	vQueueAddToRegistry(qTof2Distance_u32, "Tof2Dist");
	vQueueAddToRegistry(qTof3Distance_u32, "Tof3Dist");
	vQueueAddToRegistry(qMtrSrvCurr_u32, "MtrSrvCurr");
	vQueueAddToRegistry(qCtrlMtrCurr_d, "CtrlMtrCurr");

	xTaskCreate(Task_CarDiagnosticsTool,
				"TASK_CAR_DIAGNOSTICS_TOOL",
				DEFAULT_STACK_SIZE+100,
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

	//TODO remove
	//uint16_t bufferShrpDstQ;
	//bool bufferShrColp;

	while(1)
	{
		if (bcmBluetoothConnected())
		{
			//TODO DEBUG
			if(sent == 10)
			{
				bcmSend(helloMsg, sizeof(helloMsg));
				vTaskDelay(10);

				sent = 0;
			}
			sent++;
			// END_DEBUG

			traceFlushData();
		}

		vTaskDelay(200);
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

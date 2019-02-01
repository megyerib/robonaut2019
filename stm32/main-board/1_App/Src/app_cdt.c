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

// Defines -------------------------------------------------------------------------------------------------------------

#define	APP_TRACE_BT_ON			1
#define APP_TRACE_USB_ON		0

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//______________________________________________________________________ TX SIDE
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

//______________________________________________________________________ RX SIDE


QueueHandle_t qRecData;

uint8_t btRxBuffer[TRACE_REC_MSG_SIZE];

cTRACE_RX_DATA btRxData;

bool btMsgReceived = false;

//______________________________________________________________________ USB

extern UART_HandleTypeDef huart5;

uint8_t usbRxBuffer[TRACE_REC_MSG_SIZE];

cTRACE_RX_DATA usbRxStruct;

bool usbRec = false;

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_CarDiagnosticsTool(void)
{
	traceInit();

	//______________________________________________________________________ TX SIDE
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

	qSRunMainSM_u32   		= xQueueCreate( 1, sizeof( uint32_t ) );	// 35
	qSRunActState_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );
	qSRunActP_f		    	= xQueueCreate( 1, sizeof( float ) );
	qSRunActKp_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunActKd_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunActSpeed_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );
	qSRunGetP_f		    	= xQueueCreate( 1, sizeof( float ) );
	qSRunGetKp_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunGetKd_f		    = xQueueCreate( 1, sizeof( float ) );
	qSRunGetSpeed_u32   	= xQueueCreate( 1, sizeof( uint32_t ) );	// 44

	// Atollic debug queues
	vQueueAddToRegistry(qNaviN_f,		    "qNaviN_f");
	vQueueAddToRegistry(qNaviE_f,  		    "qNaviE_f");
	vQueueAddToRegistry(qNaviPSI_f,   	    "qNaviPsi_f");
	vQueueAddToRegistry(qEncV_f, 		    "EncV");
	vQueueAddToRegistry(qDistToF1_u32, 	    "DistToF1");
	vQueueAddToRegistry(qDistSharp1_u32,    "DistSharp1");
	vQueueAddToRegistry(qLineMainLinePos_f, "MainLinePos");
	vQueueAddToRegistry(qMazeActState_u32,  "MazeActState");
	vQueueAddToRegistry(qSRunGetP_f,    	"SRunGetP");

	//______________________________________________________________________ RX SIDE
	qRecData = xQueueCreate( 1, sizeof( cTRACE_RX_DATA ) );

	vQueueAddToRegistry(qRecData, "RecData");

	cTRACE_RX_DATA init;
	init.StopCar 			= 0;	// 1
	init.MazeMainSMReset	= 0;
	init.MazeMainSMResetTo 	= 0;
	init.MazeGetState		= 0;
	init.MazeSetState		= 0;
	init.MazeSetKp			= 0;
	init.MazeSetKd			= 0;
	init.MazeSetSpeed		= 0;
	init.SRunTryOvertake	= 0;
	init.SRunHardReset		= 0;	// 10
	init.SRunSoftReset		= 0;
	init.SRunSoftResetTo	= 0;
	init.SRunGetState		= 0;
	init.SRunSetState		= 0;
	init.SRunSetP			= 0;
	init.SRunSetKp			= 0;
	init.SRunSetKd			= 0;
	init.SRunSetSpeed		= 0;	// 18
	xQueueOverwrite(qRecData, &init);

	//______________________________________________________________________ TASK CREATE

	xTaskCreate(Task_CarDiagnosticsTool,
				"TASK_CAR_DIAGNOSTICS_TOOL",
				DEFAULT_STACK_SIZE+350 ,
				NULL,
				TASK_CDT_PRIO,
				NULL);
}

void Task_CarDiagnosticsTool(void* p)
{
	(void)p;

	//_______________________________________________________________________ TRACE SETTINGS
	if (APP_TRACE_BT_ON == 1)
	{
		bspUartReceive_IT(Uart_Bluetooth, btRxBuffer, TRACE_REC_MSG_SIZE);
	}
	else
	{
		bspUartReceive_IT(Uart_USB, usbRxBuffer, TRACE_REC_MSG_SIZE);
	}

	//_______________________________________________________________________ TRACE TASK
	while (1)
	{
		if (bspBluetoothConnected() == true && APP_TRACE_BT_ON == 1)
		{
			// Bluetooth trace to Car Diagnostics Tool.

			traceFlushData();

			if (btMsgReceived == true)
			{
				btMsgReceived = false;

				traceProcessRxData(btRxBuffer);

				btRxData = traceGetRxData();
			}

			bspUartReceive_IT(Uart_Bluetooth, btRxBuffer, TRACE_REC_MSG_SIZE);
		}
		else if (APP_TRACE_USB_ON == 1)
		{
			// USB trace.

			traceFlushData();

			if (usbRec == true)
			{
				usbRec = false;

				traceProcessRxData(usbRxBuffer);

				usbRxStruct = traceGetRxData();
			}

			bspUartReceive_IT(Uart_USB, usbRxBuffer, TRACE_REC_MSG_SIZE);
		}
		else
		{
			// Trace is turned off.
		}

		vTaskDelay(100);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

void bspUsbRxCpltCallback ()
{
	usbRec = true;
}

void bspBluetoothRxCpltCallback ()
{
	btMsgReceived = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_uart.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "usart.h"
#include "bsp_uart.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define		BSP_UART_1_HANDLER			huart1
#define		BSP_UART_1_INSTANCE			USART1

#define		BSP_UART_3_HANDLER			huart3
#define		BSP_UART_3_INSTANCE			USART3

#define		BSP_UART_4_HANDLER			huart4
#define		BSP_UART_4_INSTANCE			UART4

#define		BSP_UART_BT_HANDLER			huart5
#define		BSP_UART_BT_INSTANCE		UART5

#define		BSP_UART_RADIO_HANDLER		huart6
#define		BSP_UART_RADIO_INSTANCE		USART6

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

//! @brief	 Initializes the UART 1 peripheral.
//! @retval	 Returns BSP_OK upon successful init.
static const BspStatus bspUart1Init (void);

//! @brief	 Initializes the UART 3 peripheral.
//! @retval	 Returns BSP_OK upon successful init.
static const BspStatus bspUart3Init (void);

//! @brief	 Initializes the UART 4 peripheral.
//! @retval	 Returns BSP_OK upon successful init.
static const BspStatus bspUart4Init (void);

//! @brief	 Initializes the UART 5 peripheral.
//! @retval	 Returns BSP_OK upon successful init.
static const BspStatus bspUartBluetoothInit (void);

//! @brief	 Initializes the UART 6 peripheral.
//! @retval	 Returns BSP_OK upon successful init.
static const BspStatus bspUartRadioInit (void);

// Global function definitions -----------------------------------------------------------------------------------------

const BspStatus bspUartInitAll (void)
{
	BspStatus ret = BSP_OK;

	BspStatus status1  = bspUart1Init();
	BspStatus status3  = bspUart3Init();
	BspStatus status4  = bspUart4Init();
	BspStatus statusBt = bspUartBluetoothInit();
	BspStatus statusRd = bspUartRadioInit();

	// Check if all of the Init was successful.
	if( status1 != BSP_OK || status3 != BSP_OK || status4 != BSP_OK
			|| statusBt != BSP_OK || statusRd != BSP_OK)
	{
		ret = BSP_ERROR;
	}

	return ret;
}

const BspStatus bspUartInitDevice (const BspUartDevice device)
{
	BspStatus status = BSP_ERROR;

	// Initialize the given periphery.
	switch (device)
	{
		case Uart_1:
			status = bspUart1Init();
			break;

		case Uart_3:
			status = bspUart3Init();
			break;

		case Uart_4:
			status = bspUart4Init();
			break;

		case Uart_Bluetooth:
			status = bspUartBluetoothInit();
			break;

		case Uart_Radio:
			status = bspUartRadioInit();
			break;

		default:
			status = BSP_ERROR;
			break;
	}

	return status;
}

const BspStatus bspUartReceive_IT (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size)
{
	BspStatus status = BSP_ERROR;

	status = HAL_UART_Receive_IT(huart, pData, Size);

	return status;
}

const BspStatus bspUartTransmit_IT (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size)
{
	BspStatus status = BSP_ERROR;

	status = HAL_UART_Transmit_IT(huart, pData, Size);

	return status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Decide which peripheral owns the interrupt.
	if(huart->Instance == BSP_UART_1_INSTANCE)
	{
		bspUart1RxCpltCallback();
	}
	else if(huart->Instance == BSP_UART_3_INSTANCE)
	{
		bspUart3RxCpltCallback();
	}
	else if(huart->Instance == BSP_UART_4_INSTANCE)
	{
		bspUart4RxCpltCallback();
	}
	else if(huart->Instance == BSP_UART_BT_INSTANCE)
	{
		bspBluetoothRxCpltCallback();
	}
	else if(huart->Instance == BSP_UART_RADIO_INSTANCE)
	{
		bspRadioRxCpltCallback();
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static const BspStatus bspUart1Init (void)
{
	BspStatus status = BSP_ERROR;

	BSP_UART_1_HANDLER.Instance 				= BSP_UART_1_INSTANCE;
	BSP_UART_1_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_1_HANDLER.Init.WordLength	 		= UART_WORDLENGTH_8B;
	BSP_UART_1_HANDLER.Init.StopBits 			= UART_STOPBITS_1;
	BSP_UART_1_HANDLER.Init.Parity 				= UART_PARITY_NONE;
	BSP_UART_1_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_1_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_1_HANDLER.Init.OverSampling 		= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&BSP_UART_1_HANDLER) == HAL_OK)
	{
		status = BSP_OK;
	}

	return status;
}

static const BspStatus bspUart3Init (void)
{
	BspStatus status = BSP_ERROR;

	BSP_UART_3_HANDLER.Instance 				= BSP_UART_3_INSTANCE;
	BSP_UART_3_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_3_HANDLER.Init.WordLength	 		= UART_WORDLENGTH_8B;
	BSP_UART_3_HANDLER.Init.StopBits 			= UART_STOPBITS_1;
	BSP_UART_3_HANDLER.Init.Parity 				= UART_PARITY_NONE;
	BSP_UART_3_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_3_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_3_HANDLER.Init.OverSampling 		= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&BSP_UART_3_HANDLER) == HAL_OK)
	{
		status = BSP_OK;
	}

	return status;
}

static const BspStatus bspUart4Init (void)
{
	BspStatus status = BSP_ERROR;

	BSP_UART_4_HANDLER.Instance 				= BSP_UART_4_INSTANCE;
	BSP_UART_4_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_4_HANDLER.Init.WordLength	 		= UART_WORDLENGTH_8B;
	BSP_UART_4_HANDLER.Init.StopBits 			= UART_STOPBITS_1;
	BSP_UART_4_HANDLER.Init.Parity 				= UART_PARITY_NONE;
	BSP_UART_4_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_4_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_4_HANDLER.Init.OverSampling 		= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&BSP_UART_4_HANDLER) == HAL_OK)
	{
		status = BSP_OK;
	}

	return status;
}

static const BspStatus bspUartBluetoothInit (void)
{
	BspStatus status = BSP_ERROR;

	BSP_UART_BT_HANDLER.Instance 				= BSP_UART_BT_INSTANCE;
	BSP_UART_BT_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_BT_HANDLER.Init.WordLength 		= UART_WORDLENGTH_8B;
	BSP_UART_BT_HANDLER.Init.StopBits			= UART_STOPBITS_1;
	BSP_UART_BT_HANDLER.Init.Parity 			= UART_PARITY_NONE;
	BSP_UART_BT_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_BT_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_BT_HANDLER.Init.OverSampling		= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&BSP_UART_BT_HANDLER) == HAL_OK)
	{
		status = BSP_OK;
	}

	return status;
}

static const BspStatus bspUartRadioInit (void)
{
	BspStatus status = BSP_ERROR;

	BSP_UART_RADIO_HANDLER.Instance 			= BSP_UART_RADIO_INSTANCE;
	BSP_UART_RADIO_HANDLER.Init.BaudRate 		= 115200;
	BSP_UART_RADIO_HANDLER.Init.WordLength	 	= UART_WORDLENGTH_8B;
	BSP_UART_RADIO_HANDLER.Init.StopBits 		= UART_STOPBITS_1;
	BSP_UART_RADIO_HANDLER.Init.Parity 			= UART_PARITY_NONE;
	BSP_UART_RADIO_HANDLER.Init.Mode 			= UART_MODE_TX_RX;
	BSP_UART_RADIO_HANDLER.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	BSP_UART_RADIO_HANDLER.Init.OverSampling 	= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&BSP_UART_RADIO_HANDLER) == HAL_OK)
	{
		status = BSP_OK;
	}

	return status;
}

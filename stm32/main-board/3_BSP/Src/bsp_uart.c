////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_uart.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../3_BSP/Inc/bsp_uart.h"

#include "usart.h"
#include "gpio.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define		BSP_UART_1_HANDLER			huart1
#define		BSP_UART_1_INSTANCE			USART1

#define		BSP_UART_USB_HANDLER		huart2
#define		BSP_UART_USB_INSTANCE		USART2

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

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

// Local (static) function prototypes ----------------------------------------------------------------------------------


//! @brief	Initializes the UART 1 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUart1Init (void);


//! @brief	Initializes the UART 2 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartUsbInit (void);


//! @brief	Initializes the UART 3 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUart3Init (void);


//! @brief	Initializes the UART 4 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUart4Init (void);


//! @brief	Initializes the UART 5 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartBluetoothInit (void);


//! @brief	Initializes the UART 6 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartRadioInit (void);

// Global function definitions -----------------------------------------------------------------------------------------

eBspStatus bspUartInitAll (void)
{
	eBspStatus ret = BSP_OK;

	eBspStatus status1   = bspUart1Init();
	eBspStatus statusUsb = bspUartUsbInit();
	eBspStatus status3   = bspUart3Init();
	eBspStatus status4   = bspUart4Init();
	eBspStatus statusBt  = bspUartBluetoothInit();
	eBspStatus statusRd  = bspUartRadioInit();

	// Check if all of the Init was successful.
	if( status1 != BSP_OK || statusUsb != BSP_OK || status3 != BSP_OK || status4 != BSP_OK
			|| statusBt != BSP_OK || statusRd != BSP_OK)
	{
		ret = BSP_ERROR;
	}

	return ret;
}

eBspStatus bspUartInitDevice (const eBspUartDevice uartDevice)
{
	eBspStatus status = BSP_ERROR;

	// Initialize the given periphery.
	switch (uartDevice)
	{
		case Uart_1:
			status = bspUart1Init();
			break;

		case Uart_USB:
			status = bspUartUsbInit();
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

eBspStatus bspUartReceive (const eBspUartDevice uartDevice,
								 uint8_t* const pData,
								 const uint16_t Size,
							     const uint32_t Timeout
							)
{
	eBspStatus status = BSP_ERROR;

	switch (uartDevice)
	{
		case Uart_1:
			status = HAL_UART_Receive(&BSP_UART_1_HANDLER, pData, Size, Timeout);
			break;

		case Uart_USB:
			status = HAL_UART_Receive(&BSP_UART_USB_HANDLER, pData, Size, Timeout);
			break;

		case Uart_3:
			status = HAL_UART_Receive(&BSP_UART_3_HANDLER, pData, Size, Timeout);
			break;

		case Uart_4:
			status = HAL_UART_Receive(&BSP_UART_4_HANDLER, pData, Size, Timeout);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Receive(&BSP_UART_BT_HANDLER, pData, Size, Timeout);
			break;

		case Uart_Radio:
			status = HAL_UART_Receive(&BSP_UART_RADIO_HANDLER, pData, Size, Timeout);
			break;

		default:
			status = BSP_ERROR;
			break;
	}

	return status;
}

eBspStatus bspUartTransmit (
								 const eBspUartDevice uartDevice,
								  uint8_t* const 	  pData,
								  const uint16_t 	  Size,
								  const uint32_t 	  Timeout
							)
{
	eBspStatus status = BSP_ERROR;

	switch (uartDevice)
	{
		case Uart_1:
			status = HAL_UART_Transmit(&BSP_UART_1_HANDLER, pData, Size, Timeout);
			break;

		case Uart_USB:
			status = HAL_UART_Transmit(&BSP_UART_USB_HANDLER, pData, Size, Timeout);
			break;

		case Uart_3:
			status = HAL_UART_Transmit(&BSP_UART_3_HANDLER, pData, Size, Timeout);
			break;

		case Uart_4:
			status = HAL_UART_Transmit(&BSP_UART_4_HANDLER, pData, Size, Timeout);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Transmit(&BSP_UART_BT_HANDLER, pData, Size, Timeout);
			break;

		case Uart_Radio:
			status = HAL_UART_Transmit(&BSP_UART_RADIO_HANDLER, pData, Size, Timeout);
			break;

		default:
			status = BSP_ERROR;
			break;
	}

	return status;
}

eBspStatus bspUartReceive_IT (const eBspUartDevice uartDevice, uint8_t* const pData, const uint16_t Size)
{
	eBspStatus status = BSP_ERROR;

	switch (uartDevice)
	{
		case Uart_1:
			status = HAL_UART_Receive_IT(&BSP_UART_1_HANDLER, pData, Size);
			break;

		case Uart_USB:
			status = HAL_UART_Receive_IT(&BSP_UART_USB_HANDLER, pData, Size);
			break;

		case Uart_3:
			status = HAL_UART_Receive_IT(&BSP_UART_3_HANDLER, pData, Size);
			break;

		case Uart_4:
			status = HAL_UART_Receive_IT(&BSP_UART_4_HANDLER, pData, Size);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Receive_IT(&BSP_UART_BT_HANDLER, pData, Size);
			break;

		case Uart_Radio:
			status = HAL_UART_Receive_IT(&BSP_UART_RADIO_HANDLER, pData, Size);
			break;

		default:
			status = BSP_ERROR;
			break;
	}

	return status;
}

eBspStatus bspUartTransmit_IT (const eBspUartDevice uartDevice, uint8_t* const pData, const uint16_t Size)
{
	eBspStatus status = BSP_ERROR;

	switch (uartDevice)
	{
		case Uart_1:
			status = HAL_UART_Transmit_IT(&BSP_UART_1_HANDLER, pData, Size);
			break;

		case Uart_USB:
			status = HAL_UART_Transmit_IT(&BSP_UART_USB_HANDLER, pData, Size);
			break;

		case Uart_3:
			status = HAL_UART_Transmit_IT(&BSP_UART_3_HANDLER, pData, Size);
			break;

		case Uart_4:
			status = HAL_UART_Transmit_IT(&BSP_UART_4_HANDLER, pData, Size);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Transmit_IT(&BSP_UART_BT_HANDLER, pData, Size);
			break;

		case Uart_Radio:
			status = HAL_UART_Transmit_IT(&BSP_UART_RADIO_HANDLER, pData, Size);
			break;

		default:
			status = BSP_ERROR;
			break;
	}

	return status;
}

void USART1_IRQHandler (void)
{
	HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler (void)
{
	HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler (void)
{
	HAL_UART_IRQHandler(&huart3);
}

void UART4_IRQHandler (void)
{
	HAL_UART_IRQHandler(&huart4);
}

void UART5_IRQHandler (void)
{
	HAL_UART_IRQHandler(&huart5);
}

void USART6_IRQHandler (void)
{
	HAL_UART_IRQHandler(&huart6);
}


void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
	// Decide which periphery owns the interrupt.
	if (huart->Instance == BSP_UART_1_INSTANCE)
	{
		bspUart1TxCpltCallback();
	}
	else if (huart->Instance == BSP_UART_USB_INSTANCE)
	{
		bspUartUsbTxCpltCallback();
	}
	else if (huart->Instance == BSP_UART_3_INSTANCE)
	{
		bspUart3TxCpltCallback();
	}
	else if (huart->Instance == BSP_UART_4_INSTANCE)
	{
		bspUart4TxCpltCallback();
	}
	else if(huart->Instance == BSP_UART_BT_INSTANCE)
	{
		bspBluetoothTxCpltCallback();
	}
	else if(huart->Instance == BSP_UART_RADIO_INSTANCE)
	{
		bspRadioTxCpltCallback();
	}
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	// Decide which periphery owns the interrupt.
	if(huart->Instance == BSP_UART_1_INSTANCE)
	{
		bspUart1RxCpltCallback();
	}
	else if(huart->Instance == BSP_UART_USB_INSTANCE)
	{
		bspUartUsbRxCpltCallback();
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}

// Local (static) function definitions ---------------------------------------------------------------------------------

static eBspStatus bspUart1Init (void)
{
	eBspStatus status = BSP_OK;

	BSP_UART_1_HANDLER.Instance 				= BSP_UART_1_INSTANCE;
	BSP_UART_1_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_1_HANDLER.Init.WordLength	 		= UART_WORDLENGTH_8B;
	BSP_UART_1_HANDLER.Init.StopBits 			= UART_STOPBITS_1;
	BSP_UART_1_HANDLER.Init.Parity 				= UART_PARITY_NONE;
	BSP_UART_1_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_1_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_1_HANDLER.Init.OverSampling 		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&BSP_UART_1_HANDLER) != HAL_OK)
	{
		status = BSP_ERROR;
	}

    HAL_UART_MspInit(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    NVIC_EnableIRQ(USART1_IRQn);

	return status;
}

static eBspStatus bspUartUsbInit (void)
{
	eBspStatus status = BSP_OK;

	BSP_UART_USB_HANDLER.Instance 				= BSP_UART_USB_INSTANCE;
	BSP_UART_USB_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_USB_HANDLER.Init.WordLength	 	= UART_WORDLENGTH_8B;
	BSP_UART_USB_HANDLER.Init.StopBits 			= UART_STOPBITS_1;
	BSP_UART_USB_HANDLER.Init.Parity 			= UART_PARITY_NONE;
	BSP_UART_USB_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_USB_HANDLER.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	BSP_UART_USB_HANDLER.Init.OverSampling 		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&BSP_UART_USB_HANDLER) != HAL_OK)
	{
		status = BSP_ERROR;
	}

    HAL_UART_MspInit(&huart2);

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 1);
    NVIC_EnableIRQ(USART2_IRQn);

	return status;
}

static eBspStatus bspUart3Init (void)
{
	eBspStatus status = BSP_OK;

	BSP_UART_3_HANDLER.Instance 				= BSP_UART_3_INSTANCE;
	BSP_UART_3_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_3_HANDLER.Init.WordLength	 		= UART_WORDLENGTH_8B;
	BSP_UART_3_HANDLER.Init.StopBits 			= UART_STOPBITS_1;
	BSP_UART_3_HANDLER.Init.Parity 				= UART_PARITY_NONE;
	BSP_UART_3_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_3_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_3_HANDLER.Init.OverSampling 		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&BSP_UART_3_HANDLER) != HAL_OK)
	{
		status = BSP_ERROR;
	}

	HAL_UART_MspInit(&huart3);

	HAL_NVIC_SetPriority(USART3_IRQn, 2, 1);
	NVIC_EnableIRQ(USART3_IRQn);

	return status;
}

static eBspStatus bspUart4Init (void)
{
	eBspStatus status = BSP_OK;

	BSP_UART_4_HANDLER.Instance 				= BSP_UART_4_INSTANCE;
	BSP_UART_4_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_4_HANDLER.Init.WordLength	 		= UART_WORDLENGTH_8B;
	BSP_UART_4_HANDLER.Init.StopBits 			= UART_STOPBITS_1;
	BSP_UART_4_HANDLER.Init.Parity 				= UART_PARITY_NONE;
	BSP_UART_4_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_4_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_4_HANDLER.Init.OverSampling 		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&BSP_UART_4_HANDLER) != HAL_OK)
	{
		status = BSP_ERROR;
	}

    HAL_UART_MspInit(&huart4);

    HAL_NVIC_SetPriority(UART4_IRQn, 2, 2);
    NVIC_EnableIRQ(UART4_IRQn);

	return status;
}

static eBspStatus bspUartBluetoothInit (void)
{
	eBspStatus status = BSP_OK;

	BSP_UART_BT_HANDLER.Instance 				= BSP_UART_BT_INSTANCE;
	BSP_UART_BT_HANDLER.Init.BaudRate 			= 115200;
	BSP_UART_BT_HANDLER.Init.WordLength 		= UART_WORDLENGTH_8B;
	BSP_UART_BT_HANDLER.Init.StopBits			= UART_STOPBITS_1;
	BSP_UART_BT_HANDLER.Init.Parity 			= UART_PARITY_NONE;
	BSP_UART_BT_HANDLER.Init.Mode 				= UART_MODE_TX_RX;
	BSP_UART_BT_HANDLER.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
	BSP_UART_BT_HANDLER.Init.OverSampling		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&BSP_UART_BT_HANDLER) != HAL_OK)
	{
		status = BSP_ERROR;
	}

	HAL_UART_MspInit(&huart5);

	HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
	NVIC_EnableIRQ(UART5_IRQn);

	return status;
}

static eBspStatus bspUartRadioInit (void)
{
	eBspStatus status = BSP_OK;

	BSP_UART_RADIO_HANDLER.Instance 			= BSP_UART_RADIO_INSTANCE;
	BSP_UART_RADIO_HANDLER.Init.BaudRate 		= 115200;
	BSP_UART_RADIO_HANDLER.Init.WordLength	 	= UART_WORDLENGTH_8B;
	BSP_UART_RADIO_HANDLER.Init.StopBits 		= UART_STOPBITS_1;
	BSP_UART_RADIO_HANDLER.Init.Parity 			= UART_PARITY_NONE;
	BSP_UART_RADIO_HANDLER.Init.Mode 			= UART_MODE_TX_RX;
	BSP_UART_RADIO_HANDLER.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	BSP_UART_RADIO_HANDLER.Init.OverSampling 	= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&BSP_UART_RADIO_HANDLER) != HAL_OK)
	{
		status = BSP_ERROR;
	}

    HAL_UART_MspInit(&huart6);

    HAL_NVIC_SetPriority(USART6_IRQn, 2, 3);
    NVIC_EnableIRQ(USART6_IRQn);

	return status;
}


__weak void bspUart1RxCpltCallback     (void) {}
__weak void bspUartUsbRxCpltCallback   (void) {}
__weak void bspUart3RxCpltCallback     (void) {}
__weak void bspUart4RxCpltCallback     (void) {}
__weak void bspBluetoothRxCpltCallback (void) {}
__weak void bspRadioRxCpltCallback     (void) {}
__weak void bspUart1TxCpltCallback     (void) {}
__weak void bspUartUsbTxCpltCallback   (void) {}
__weak void bspUart3TxCpltCallback     (void) {}
__weak void bspUart4TxCpltCallback     (void) {}
__weak void bspBluetoothTxCpltCallback (void) {}
__weak void bspRadioTxCpltCallback     (void) {}

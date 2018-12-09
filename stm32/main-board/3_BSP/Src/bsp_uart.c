////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_uart.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_uart.h"
#include "usart.h"
#include "gpio.h"

// Defines -------------------------------------------------------------------------------------------------------------

/*#define		BSP_UART_1_HANDLER		huart1
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
#define		BSP_UART_RADIO_INSTANCE		USART6*/

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

// For dynamic device assignment
static UART_HandleTypeDef* huart_motor;
static UART_HandleTypeDef* huart_line_front;
static UART_HandleTypeDef* huart_line_rear;
static UART_HandleTypeDef* huart_usb;
static UART_HandleTypeDef* huart_bluetooth;
static UART_HandleTypeDef* huart_radio;

static USART_TypeDef* usart_motor;
static USART_TypeDef* usart_line_front;
static USART_TypeDef* usart_line_rear;
static USART_TypeDef* usart_usb;
static USART_TypeDef* usart_bluetooth;
static USART_TypeDef* usart_radio;

// Local (static) function prototypes ----------------------------------------------------------------------------------


//! @brief	Initializes the UART 1 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartMotorInit (void);


//! @brief	Initializes the UART 2 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartUsbInit (void);


//! @brief	Initializes the UART 3 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartLineFrontInit (void);


//! @brief	Initializes the UART 4 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartLineRearInit (void);


//! @brief	Initializes the UART 5 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartBluetoothInit (void);


//! @brief	Initializes the UART 6 periphery.
//! @retval	Returns BSP_OK upon successful init.
static eBspStatus bspUartRadioInit (void);

//! @brief  Assign UART 1-3-4 to line sensors & motor controller
static void bspUartAssignDevices(void);

// Global function definitions -----------------------------------------------------------------------------------------

eBspStatus bspUartInit (void)
{
	eBspStatus ret = BSP_OK;

	bspUartAssignDevices();

	eBspStatus statusMotor     = bspUartMotorInit();
	eBspStatus statusUsb       = bspUartUsbInit();
	eBspStatus statusLineFront = bspUartLineFrontInit();
	eBspStatus statusLineRear  = bspUartLineRearInit();
	eBspStatus statusBt        = bspUartBluetoothInit();
	eBspStatus statusRd        = bspUartRadioInit();

	// Check if all of the Init was successful.
	if( statusMotor     != BSP_OK ||
	    statusUsb       != BSP_OK ||
	    statusLineFront != BSP_OK ||
	    statusLineRear  != BSP_OK ||
	    statusBt        != BSP_OK ||
	    statusRd        != BSP_OK )
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
		case Uart_Motor:
			status = bspUartMotorInit();
			break;

		case Uart_USB:
			status = bspUartUsbInit();
			break;

		case Uart_LineFront:
			status = bspUartLineFrontInit();
			break;

		case Uart_LineRear:
			status = bspUartLineRearInit();
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
		case Uart_Motor:
			status = HAL_UART_Receive(huart_motor, pData, Size, Timeout);
			break;

		case Uart_USB:
			status = HAL_UART_Receive(huart_usb, pData, Size, Timeout);
			break;

		case Uart_LineFront:
			status = HAL_UART_Receive(huart_line_front, pData, Size, Timeout);
			break;

		case Uart_LineRear:
			status = HAL_UART_Receive(huart_line_rear, pData, Size, Timeout);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Receive(huart_bluetooth, pData, Size, Timeout);
			break;

		case Uart_Radio:
			status = HAL_UART_Receive(huart_radio, pData, Size, Timeout);
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
		case Uart_Motor:
			status = HAL_UART_Transmit(huart_motor, pData, Size, Timeout);
			break;

		case Uart_USB:
			status = HAL_UART_Transmit(huart_usb, pData, Size, Timeout);
			break;

		case Uart_LineFront:
			status = HAL_UART_Transmit(huart_line_front, pData, Size, Timeout);
			break;

		case Uart_LineRear:
			status = HAL_UART_Transmit(huart_line_rear, pData, Size, Timeout);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Transmit(huart_bluetooth, pData, Size, Timeout);
			break;

		case Uart_Radio:
			status = HAL_UART_Transmit(huart_radio, pData, Size, Timeout);
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
		case Uart_Motor:
			status = HAL_UART_Receive_IT(huart_motor, pData, Size);
			break;

		case Uart_USB:
			status = HAL_UART_Receive_IT(huart_usb, pData, Size);
			break;

		case Uart_LineFront:
			status = HAL_UART_Receive_IT(huart_line_front, pData, Size);
			break;

		case Uart_LineRear:
			status = HAL_UART_Receive_IT(huart_line_rear, pData, Size);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Receive_IT(huart_bluetooth, pData, Size);
			break;

		case Uart_Radio:
			status = HAL_UART_Receive_IT(huart_radio, pData, Size);
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
		case Uart_Motor:
			status = HAL_UART_Transmit_IT(huart_motor, pData, Size);
			break;

		case Uart_USB:
			status = HAL_UART_Transmit_IT(huart_usb, pData, Size);
			break;

		case Uart_LineFront:
			status = HAL_UART_Transmit_IT(huart_line_front, pData, Size);
			break;

		case Uart_LineRear:
			status = HAL_UART_Transmit_IT(huart_line_rear, pData, Size);
			break;

		case Uart_Bluetooth:
			status = HAL_UART_Transmit_IT(huart_bluetooth, pData, Size);
			break;

		case Uart_Radio:
			status = HAL_UART_Transmit_IT(huart_radio, pData, Size);
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
	if (huart->Instance == usart_motor)
	{
		bspMotorTxCpltCallback();
	}
	else if (huart->Instance == usart_usb)
	{
		bspUsbTxCpltCallback();
	}
	else if (huart->Instance == usart_line_front)
	{
		bspLineFrontTxCpltCallback();
	}
	else if (huart->Instance == usart_line_rear)
	{
		bspLineRearTxCpltCallback();
	}
	else if(huart->Instance == usart_bluetooth)
	{
		bspBluetoothTxCpltCallback();
	}
	else if(huart->Instance == usart_radio)
	{
		bspRadioTxCpltCallback();
	}
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	// Decide which periphery owns the interrupt.
	if(huart->Instance == usart_motor)
	{
		bspMotorRxCpltCallback();
	}
	else if(huart->Instance == usart_usb)
	{
		bspUsbRxCpltCallback();
	}
	else if(huart->Instance == usart_line_front)
	{
		bspLineFrontRxCpltCallback();
	}
	else if(huart->Instance == usart_line_rear)
	{
		bspLineRearRxCpltCallback();
	}
	else if(huart->Instance == usart_bluetooth)
	{
		bspBluetoothRxCpltCallback();
	}
	else if(huart->Instance == usart_radio)
	{
		bspRadioRxCpltCallback();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}

// Local (static) function definitions ---------------------------------------------------------------------------------

static eBspStatus bspUartMotorInit (void)
{
	eBspStatus status = BSP_OK;

	huart_motor->Instance 				= usart_motor;

	huart_motor->Init.BaudRate 			= 115200;
	huart_motor->Init.WordLength	 	= UART_WORDLENGTH_8B;
	huart_motor->Init.StopBits 			= UART_STOPBITS_1;
	huart_motor->Init.Parity 			= UART_PARITY_NONE;
	huart_motor->Init.Mode 				= UART_MODE_TX_RX;
	huart_motor->Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart_motor->Init.OverSampling 		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huart_motor) != HAL_OK)
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

	huart_usb->Instance 			= usart_usb;

	huart_usb->Init.BaudRate 		= 115200;
	huart_usb->Init.WordLength	 	= UART_WORDLENGTH_8B;
	huart_usb->Init.StopBits 		= UART_STOPBITS_1;
	huart_usb->Init.Parity 			= UART_PARITY_NONE;
	huart_usb->Init.Mode 			= UART_MODE_TX_RX;
	huart_usb->Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart_usb->Init.OverSampling 	= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huart_usb) != HAL_OK)
	{
		status = BSP_ERROR;
	}

    HAL_UART_MspInit(&huart2);

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 1);
    NVIC_EnableIRQ(USART2_IRQn);

	return status;
}

static eBspStatus bspUartLineFrontInit (void)
{
	eBspStatus status = BSP_OK;

	huart_line_front->Instance 				= usart_line_front;

	huart_line_front->Init.BaudRate 		= 115200;
	huart_line_front->Init.WordLength	 	= UART_WORDLENGTH_8B;
	huart_line_front->Init.StopBits 		= UART_STOPBITS_1;
	huart_line_front->Init.Parity 			= UART_PARITY_NONE;
	huart_line_front->Init.Mode 			= UART_MODE_TX_RX;
	huart_line_front->Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart_line_front->Init.OverSampling 	= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huart_line_front) != HAL_OK)
	{
		status = BSP_ERROR;
	}

	HAL_UART_MspInit(&huart3);

	HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
	NVIC_EnableIRQ(USART3_IRQn);

	return status;
}

static eBspStatus bspUartLineRearInit (void)
{
	eBspStatus status = BSP_OK;

	huart_line_rear->Instance 				= usart_line_rear;

	huart_line_rear->Init.BaudRate 			= 115200;
	huart_line_rear->Init.WordLength	 	= UART_WORDLENGTH_8B;
	huart_line_rear->Init.StopBits 			= UART_STOPBITS_1;
	huart_line_rear->Init.Parity 			= UART_PARITY_NONE;
	huart_line_rear->Init.Mode 				= UART_MODE_TX_RX;
	huart_line_rear->Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart_line_rear->Init.OverSampling 		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huart_line_rear) != HAL_OK)
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

	huart_bluetooth->Instance 				= usart_bluetooth;

	huart_bluetooth->Init.BaudRate 			= 115200;
	huart_bluetooth->Init.WordLength 		= UART_WORDLENGTH_8B;
	huart_bluetooth->Init.StopBits			= UART_STOPBITS_1;
	huart_bluetooth->Init.Parity 			= UART_PARITY_NONE;
	huart_bluetooth->Init.Mode 				= UART_MODE_TX_RX;
	huart_bluetooth->Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart_bluetooth->Init.OverSampling		= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huart_bluetooth) != HAL_OK)
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

	huart_radio->Instance 			    = usart_radio;

	huart_radio->Init.BaudRate 		    = 115200;
	huart_radio->Init.WordLength	    = UART_WORDLENGTH_8B;
	huart_radio->Init.StopBits 		    = UART_STOPBITS_1;
	huart_radio->Init.Parity 		    = UART_PARITY_NONE;
	huart_radio->Init.Mode 			    = UART_MODE_TX_RX;
	huart_radio->Init.HwFlowCtl 	    = UART_HWCONTROL_NONE;
	huart_radio->Init.OverSampling 	    = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huart_radio) != HAL_OK)
	{
		status = BSP_ERROR;
	}

    HAL_UART_MspInit(&huart6);

    HAL_NVIC_SetPriority(USART6_IRQn, 2, 3);
    NVIC_EnableIRQ(USART6_IRQn);

	return status;
}

static void bspUartAssignDevices(void)
{
    // TODO Auto device identification

    // Fix
    huart_usb        = &huart2;
    huart_bluetooth  = &huart5;
    huart_radio      = &huart6;

    usart_usb        = USART2;
    usart_bluetooth  = UART5;
    usart_radio      = USART6;

    // Dynamically assigned
    huart_motor      = &huart1;
    huart_line_front = &huart3;
    huart_line_rear  = &huart4;

    usart_motor      = USART1;
    usart_line_front = USART3;
    usart_line_rear  = UART4;

}

// RxCpltCallbacks -----------------------------

__weak void bspUsbRxCpltCallback   (void) {} // Fix
__weak void bspBluetoothRxCpltCallback (void) {}
__weak void bspRadioRxCpltCallback     (void) {}

__weak void bspMotorRxCpltCallback     (void) {} // Dynamically assigned
__weak void bspLineFrontRxCpltCallback (void) {}
__weak void bspLineRearRxCpltCallback  (void) {}


// TxCpltCallbacks -----------------------------

__weak void bspUsbTxCpltCallback   (void) {} // Fix
__weak void bspBluetoothTxCpltCallback (void) {}
__weak void bspRadioTxCpltCallback     (void) {}

__weak void bspMotorTxCpltCallback     (void) {} // Dynamically assigned
__weak void bspLineFrontTxCpltCallback (void) {}
__weak void bspLineRearTxCpltCallback  (void) {}

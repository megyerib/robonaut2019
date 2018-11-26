////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_uart.h
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../3_BSP/Inc/bsp_common.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

//! @brief	Enums for the application of the available UART devices.
typedef enum
{
	Uart_1 			= 0,
	Uart_USB,
	Uart_3,
	Uart_4,
	Uart_Bluetooth,
	Uart_Radio
} eBspUartDevice;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

//! @brief	Initializes all of the UART devices.
//!
//! @retval	Successful or unsuccessful init.
eBspStatus bspUartInitAll (void);


//! @brief	Initializes a given UART periphery
//!
//! @param	uartDevice	Enum of a possible UART periphery.
//!
//! @retval	Returns BSP_OK if the init was successful.
eBspStatus bspUartInitDevice (const eBspUartDevice uartDevice);

//! @brief	Receives an amount of data in blocking mode. This function encapsulates a HAL function.
//!
//! @param	uartDevice	Enum to the Handler of the UART periphery that will receive the message.
//! @param	pData		Pointer to a buffer in which the message will be copied.
//! @param	Size		Amount of data to be received.
//!
//! @retval	Returns BSP_OK if the receiving was successful.
eBspStatus bspUartReceive (const eBspUartDevice uartDevice,
								 uint8_t* const pData,
								 const uint16_t Size,
							     const uint32_t Timeout
							);

//!
//! @brief	Sends an amount of data in blocking mode. This function encapsulates a HAL function.
//!
//! @param	uartDevice	Enum to the handler of the UART periphery that will transmit the message.
//! @param	pData		Pointer to a buffer that will be send out.
//! @param	Size		Amount of data to be sent.
//!
//! @retval	Returns BSP_OK if the sending was successful.
eBspStatus bspUartTransmit (const eBspUartDevice uartDevice,
								  uint8_t* const pData,
								  const uint16_t Size,
								  const uint32_t Timeout
							);


//! @brief	Receives an amount of data in non blocking mode. This function encapsulates a HAL function.
//!
//! @param	uartDevice	Enum to the Handler of the UART periphery that will receive the message.
//! @param	pData		Pointer to a buffer in which the message will be copied.
//! @param	Size		Amount of data to be received.
//!
//! @retval	Returns BSP_OK if the receiving was successful.
eBspStatus bspUartReceive_IT (const eBspUartDevice uartDevice, uint8_t* const pData, const uint16_t Size);

//!
//! @brief	Sends an amount of data in non blocking mode. This function encapsulates a HAL function.
//!
//! @param	uartDevice	Enum to the handler of the UART periphery that will transmit the message.
//! @param	pData		Pointer to a buffer that will be send out.
//! @param	Size		Amount of data to be sent.
//!
//! @retval	Returns BSP_OK if the sending was successful.
eBspStatus bspUartTransmit_IT (const eBspUartDevice uartDevice, uint8_t* const pData, const uint16_t Size);


//! @brief	You can implement this callback function in other parts of the code.
//! 		This function will be called if the Uart1 RX interrupt is present.
void bspUart1RxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//!			This function will be called if the USB RX interrupt is present.
void bspUartUsbRxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//!			This function will be called if the Uart3 RX interrupt is present.
void bspUart3RxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//!			This function will be called if the Uart4 RX interrupt is present.
void bspUart4RxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//! 		This function will be called if the Bluetooth RX interrupt is present.
void bspBluetoothRxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//! 		This function will be called if the Radio RX interrupt is present.
void bspRadioRxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//! 		This function will be called if the Uart1 RX interrupt is present.
void bspUart1TxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//! 		This function will be called if the USB TX interrupt is present.
void bspUartUsbTxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//! 		This function will be called if the Uart3 TX interrupt is present.
void bspUart3TxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//!			This function will be called if the Uart4 TX interrupt is present.
void bspUart4TxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//!			This function will be called if the Bluetooth TX interrupt is present.
void bspBluetoothTxCpltCallback (void);


//! @brief	You can implement this callback function in other parts of the code.
//! 			This function will be called if the Radio TX interrupt is present.
void bspRadioTxCpltCallback (void);


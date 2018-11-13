/*
 * bsp_uart.h
 *
 *  Created on: 2018. nov. 13.
 *      Author: Joci
 */

#pragma once

// ------------------------------- Includes -------------------------------- //

#include "stm32f4xx_hal.h"
#include "bsp_common.h"

// -------------------------------- Defines ---------------------------------//
// -------------------------------- Typedefs --------------------------------//

/*
 * @brief	Enums for the application of the available UART devices.
 */
typedef enum
{
	Uart_1 			= 0,
	Uart_3,
	Uart_4,
	Uart_Bluetooth,
	Uart_Radio
} BspUartDevice;

// ------------------------------ Declarations ------------------------------//

/*
 * @brief	You can implement this callback function in other parts of the code.
 * 			This function will be called if the Uart1 RX interrupt is present.
 */
__weak void bspUart1RxCpltCallback (void);

/*
 * @brief	You can implement this callback function in other parts of the code.
 * 			This function will be called if the Uart3 RX interrupt is present.
 */
__weak void bspUart3RxCpltCallback (void);

/*
 * @brief	You can implement this callback function in other parts of the code.
 * 			This function will be called if the Uart4 RX interrupt is present.
 */
__weak void bspUart4RxCpltCallback (void);

/*
 * @brief	You can implement this callback function in other parts of the code.
 * 			This function will be called if the Bluetooth RX interrupt is present.
 */
__weak void bspBluetoothRxCpltCallback (void);

/*
 * @brief	You can implement this callback function in other parts of the code.
 * 			This function will be called if the Radio RX interrupt is present.
 */
__weak void bspRadioRxCpltCallback (void);

/*
 * @brief	Initializes all of the UART devices.
 * @retval	Successful or unsuccessful init.
 */
const BspStatus bspUartInitAll (void);

/*
 * @brief	Initializes a given UART periphery
 * @param	device	Enum of a possible UART periphery.
 * @retval	Returns BSP_OK if the init was successful.
 */
const BspStatus bspUartInitDevice (const BspUartDevice device);

/*
 * @brief	Receives an amount of data in non blocking mode. This function encapsulates a HAL function.
 * @param	huart	Handler of the UART periphery that will be used to receive the message.
 * @param	pData	Pointer to a buffer in which the message will be copied.
 * @param	Size	Amount of data to be received.
 * @retval	Returns BSP_OK if the receiving was successful.
 */
const BspStatus bspUartReceive_IT (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);

/*
 * @brief	Sends an amount of data in non blocking mode. This function encapsulates a HAL function.
 * @param	huart	Handler of the UART periphery that will be used to receive the message.
 * @param	pData	Pointer to a buffer that will be send out.
 * @param	Size	Amount of data to be sent.
 * @retval	Returns BSP_OK if the sending was successful.
 */
const BspStatus bspUartTransmit_IT (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);

// --------------------------------------------------------------------------//



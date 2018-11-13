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

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// --------------------------------- Enums ----------------------------------//

typedef enum
{
	Uart_1 			= 0,
	Uart_3,
	Uart_4,
	Uart_Bluetooth,
	Uart_Radio
} BspUartDevice;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

__weak void bspUart1RxCpltCallback (void);

__weak void bspUart3RxCpltCallback (void);

__weak void bspUart4RxCpltCallback (void);

__weak void bspBluetoothRxCpltCallback (void);

__weak void bspRadioRxCpltCallback (void);

const BspStatus bspUartInitAll (void);

const BspStatus bspUartInitDevice (const BspUartDevice device);

const BspStatus bspUartReceive_IT (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);

const BspStatus bspUartTransmit_IT (UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);

// --------------------------------------------------------------------------//



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_uart_common.h
//!  \brief     Common stuff for intelligent UART devices
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
    mainBoard = 0,
    lineSensorFront,
    lineSensorRear,
    motorController,

    uartDeviceNum
}
UART_DEVICE;

typedef enum
{
    identifyYourself = 0,
    identification,

    uartMsgTypeNum
}
UART_MSG_TYPE;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------
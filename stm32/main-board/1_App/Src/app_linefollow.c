////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_linefollow.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_common.h"
#include "line.h"

#include "bsp_uart.h" // TODO remove

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void Task_LineFollow (void* p);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_LineFollow (void)
{
    lineInit();

    xTaskCreate(Task_LineFollow,
                "TASK_LINE_FOLLOW",
                DEFAULT_STACK_SIZE,
                NULL,
                TASK_LINE_FOLLOW_PRIO,
                NULL);
}

uint8_t rxBuf[3];

static void Task_LineFollow (void* p)
{
    (void)p;



    LINE line;

    bspUartReceive_IT(0, rxBuf, 3);
    bspUartReceive_IT(1, rxBuf, 3);
    bspUartReceive_IT(2, rxBuf, 3);
    bspUartReceive_IT(3, rxBuf, 3);
    bspUartReceive_IT(4, rxBuf, 3);
    bspUartReceive_IT(5, rxBuf, 3);

    while (1)
    {
        bspUartTransmit(Uart_Motor, (uint8_t*) "asd", 3, HAL_MAX_DELAY);

        line = lineGet();
        vTaskDelay(1000);
    }
}

void bspLineRearRxCpltCallback (void)
{
    bspUartReceive_IT(Uart_LineRear, rxBuf, 3);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

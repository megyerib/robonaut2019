////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      comm.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../2_BSP/Inc/comm.h"

#include "usart.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define ENTRY_LEN  5
#define ENTRY_NUM 32

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static uint8_t buf[ENTRY_LEN*ENTRY_NUM+1];

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

// Local (static) function definitions ---------------------------------------------------------------------------------

void sendFullMeasurment(uint32_t* meas)
{
    int i;

    for (i = 0; i < ENTRY_NUM; i++)
        sprintf((char*) &buf[i*ENTRY_LEN], "%04X\n", (unsigned int) meas[i]);

    buf[ENTRY_LEN*ENTRY_NUM] = '\n';

    HAL_UART_Transmit_IT(&huart1, buf, ENTRY_LEN*ENTRY_NUM+1);
}

// We have only 1 UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}
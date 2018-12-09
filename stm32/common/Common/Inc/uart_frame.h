////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      uart_frame.h
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include <stdint.h>

// Defines -------------------------------------------------------------------------------------------------------------

#define ESCAPE_CHAR (0xFF)
#define UFRAME_BUFMAX     (  32)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
    frameBegin = 0,
    frameEnd
}
SPECIAL_CHAR;

typedef enum
{
	init,
	out_resetbuf,
	out_rx,
	out_begincheck,
	out_ovrcheck,
	in_resetbuf,
	in_rx,
	in_endcheck,
	in_ovrcheck,
	in_process
}
UARTFRAME_STATE;

/*typedef struct
{
	UARTFRAME_STATE state;
	uint8_t         rxBuf[BUFSIZE];
	uint8_t         rxBufSize;
	uint8_t         payloadBuf[BUFSIZE];
	uint8_t         payloadBufSize;
}
UART_FRAME_PROCESSOR;*/

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void convertToUartFrame(uint8_t* payload, uint8_t* frame, int payloadLen, int* framelen);
void convertFromUartFrame(uint8_t* frame, uint8_t* payload, int framelen, int* payloadLen);
int isUartFrameEnded(uint8_t* frame, int framelen);

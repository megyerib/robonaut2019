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

#define ESCAPE_CHAR     (0xFF)
#define UFRAME_BUFMAX     (32)

// Typedefs ------------------------------------------------------------------------------------------------------------

//! Receive 1 character (or prepares the reception)
typedef void (*RxChar)(uint8_t* nextchar);

//! Pass result buffer to the target module
typedef void (*ProcessResult)(uint8_t* buf, uint8_t size);

typedef enum
{
    frameBegin = 0,
    frameEnd
}
SPECIAL_CHAR;

typedef enum
{
	init,

	// Outside a frame
	out_resetbuf,
	out_rx,
	out_begincheck,
	out_ovrcheck,

	// Inside a frame
	in_resetbuf,
	in_rx,
	in_endcheck,
	in_ovrcheck,
	in_process
}
UARTFRAME_STATE;

//! Needs to be set:
//! - state (init)
//! - receive
//! - process
typedef struct
{
	UARTFRAME_STATE state;
	uint8_t         rxBuf[UFRAME_BUFMAX];
	int             rxBufSize;
	uint8_t         uframeBuf[UFRAME_BUFMAX];
	int             uframeBufSize;
	RxChar          receive;
	ProcessResult   process;
}
UFRAME_RX_STM;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void convertToUartFrame(uint8_t* payload, uint8_t* frame, int payloadLen, int* framelen);
void uartFrameRxStm(UFRAME_RX_STM* stm);

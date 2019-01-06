////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      uart-frame.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "uart_frame.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define RX_BUF_MAX 20 /* for safety reasons */

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void convertToUartFrame(uint8_t* payload, uint8_t* frame, int payloadLen, int* framelen)
{
    int i, j;

    // Begin (2 begin flags)
    frame[0] = ESCAPE_CHAR;
    frame[1] = frameBegin;
    frame[2] = ESCAPE_CHAR;
	frame[3] = frameBegin;
    j = 4;

    // Payload
    for (i = 0; i < payloadLen; i++)
    {
    	// Safety
		if (i >= RX_BUF_MAX)
		{
			*framelen = 0;
			return;
		}

    	if (payload[i] == ESCAPE_CHAR)
        {
            frame[j] = ESCAPE_CHAR;
            j++;
            frame[j] = ESCAPE_CHAR;
        }
        else
        {
            frame[j] = payload[i];
        }

        j++;
    }

    // End
    frame[j] = ESCAPE_CHAR;
    j++;
    frame[j] = frameEnd;
    j++;

    *framelen = j;
}

// Buffer length is ok
// Begin flag is not present as it was once split by the state machine.
// Last 2 bytes are the end flag
// The function just removes the escape flags
void convertFromUartFrame(uint8_t* frame, uint8_t* payload, int framelen, int* payloadLen)
{
    int i;
    int j = 0;

	for (i = 0; i < framelen - 2; i++)
    {
    	if (frame[i] != ESCAPE_CHAR)
    	{
    		payload[j] = frame[i];
    	}
    	else
    	{
    		payload[j] = ESCAPE_CHAR;
    		i++;
    	}

    	j++;
    }

	*payloadLen = j;
}

void uartFrameRxStm(UFRAME_RX_STM* stm)
{
	int breakLoop = 0;

	while (!breakLoop)
	{
		switch (stm->state)
		{
			case init:
			{
				stm->state = out_resetbuf;

				break;
			}
			case out_resetbuf:
			{
				stm->rxBufSize = 0;

				stm->state = out_rx;
				break;
			}
			case out_rx:
			{
				//bspUartReceive_IT(Uart_LineFront, &lineRxBuf[lineRxBufSize], 1);
				stm->receive(&stm->rxBuf[stm->rxBufSize]);
				stm->rxBufSize++;

				stm->state = out_begincheck;
				breakLoop = 1;
				break;
			}
			case out_begincheck:
			{
				if (
					stm->rxBufSize >= 2                          &&
					stm->rxBuf[stm->rxBufSize - 2] == ESCAPE_CHAR &&
					stm->rxBuf[stm->rxBufSize - 1] == frameBegin
				)
				{
					stm->state = in_resetbuf;
				}
				else
				{
					stm->state = out_ovrcheck;
				}

				break;
			}
			case out_ovrcheck:
			{
				if (stm->rxBufSize < UFRAME_BUFMAX)
				{
					stm->state = out_rx;
				}
				else
				{
					stm->state = out_resetbuf;
				}

				break;
			}
			case in_resetbuf:
			{
				stm->rxBufSize = 0;

				stm->state = in_rx;
				break;
			}
			case in_rx:
			{
				stm->receive(&stm->rxBuf[stm->rxBufSize]);
				stm->rxBufSize++;

				stm->state = in_endcheck;
				breakLoop = 1;
				break;
			}
			case in_endcheck:
			{
				if (
					stm->rxBufSize >= 2                          &&
					stm->rxBuf[stm->rxBufSize - 2] == ESCAPE_CHAR &&
					stm->rxBuf[stm->rxBufSize - 1] == frameEnd
				)
				{
					stm->state = in_process;
				}
				else
				{
					stm->state = in_ovrcheck;
				}

				break;
			}
			case in_ovrcheck:
			{
				if (stm->rxBufSize < UFRAME_BUFMAX)
				{
					stm->state = in_rx;
				}
				else
				{
					stm->state = out_resetbuf;
				}

				break;
			}
			case in_process:
			{
				//LINE_SENSOR_OUT* receivedLine;

				convertFromUartFrame(
					stm->rxBuf,
					stm->uframeBuf,
					stm->rxBufSize,
					&stm->uframeBufSize
				);

				stm->process(stm->uframeBuf, stm->uframeBufSize);
				/*if (uframeBufSize == sizeof(LINE_SENSOR_OUT))
				{
					receivedLine = (LINE_SENSOR_OUT*) uframeBuf;
					front_tmp = *receivedLine;
				}*/

				stm->state = out_resetbuf;

				break;
			}
			default:
			{
				stm->state = out_resetbuf;
				break;
			}
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

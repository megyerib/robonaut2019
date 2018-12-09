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
// Begin flag no present as it was once split by the state machine.
// Last 2 bytes are the end flag
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

int isUartFrameEnded(uint8_t* frame, int framelen)
{
	int ret = 0;

	if (frame[framelen-2] == ESCAPE_CHAR && frame[framelen-1] == frameEnd)
	{
		// Has frame began?
		for (int i = 0; i < framelen - 1; i++)
		{
			if (frame[i] == ESCAPE_CHAR && frame[i+1] == frameBegin)
			{
				ret = 1;
				break;
			}
		}
	}

	return ret;
}

int isUartFrameValid(uint8_t* frame, int framelen)
{
	int begin = 0;
	int end   = 0;

	int i;

	for (i = 0; i < framelen; i++)
	{
		// Safety
		if (i >= RX_BUF_MAX)
			return 0;

		if (frame[i] == ESCAPE_CHAR)
		{
			if (frame[i+1] == frameBegin)
			{
				begin = 1;
			}

			if (frame[i+1] == frameEnd)
			{
				if (begin)
				{
					end = 1;
					break;
				}
			}
		}
	}

	return (begin && end);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

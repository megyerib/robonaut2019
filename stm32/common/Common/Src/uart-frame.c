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

void convertFromUartFrame(uint8_t* frame, uint8_t* payload, int framelen, int* payloadLen)
{
    int i;
    int j     = 0;
    int end   = 0;

    *payloadLen = 0;

    // For safety reasons
	if (framelen >= RX_BUF_MAX)
		return;

    // Search for frame begin
    for (i = 0; i < framelen - 1; i++)
    {
    	if (frame[i] == ESCAPE_CHAR && frame[i+1] == frameBegin)
    	{
    		i += 2;
    		break;
    	}
    }

    // Process message
    for (/* i is set */; i < framelen; i++)
    {
    	if (frame[i] == ESCAPE_CHAR)
        {
            switch (frame[i+1])
            {
                case ESCAPE_CHAR:
                {
                    payload[j] = ESCAPE_CHAR;
                    j++;
                    break;
                }
                case frameBegin:
                {
                	j = 0;
                    break;
                }
                case frameEnd:
                {
                    end = 1;
                    break;
                }
            }

            i++; // Skip 1 character

            if (end)
                break;
        }
        else
        {
			// Ordinary character
			payload[j] = frame[i];
			j++;
        }
    }

    if (end)
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

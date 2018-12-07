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

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void convertToUartFrame(uint8_t* payload, uint8_t* frame, int payloadLen, int* framelen)
{
    int i, j;

    // Begin
    frame[0] = ESCAPE_CHAR;
    frame[1] = frameBegin;
    j = 2;

    // Payload
    for (i = 0; i < payloadLen; i++)
    {
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
    int i, j = 0, end = 0;

    for (i = 0; i < framelen; i++)
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

    *payloadLen = j;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      feedback.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../1_App/Inc/feedback.h"

#include "../../2_BSP/Inc/ldriver.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

static uint8_t mmToLedPos(int16_t mm);

// Global function definitions -----------------------------------------------------------------------------------------

void ledFeedback(LINE_SENSOR_OUT* line)
{
    uint32_t ledval = 0, i, ledpos = 0;

    for (i = 0; i < line->cnt; i++)
    {
        ledpos = mmToLedPos(line->lines[i]);

        ledval |= 1 << ledpos;
        ledval |= 1 << (ledpos + 1);
    }

    writeLed(ledval);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static uint8_t mmToLedPos(int16_t mm)
{
    uint8_t ret = (uint8_t)(32 - (mm + MID_IR_POS_MM + 15 * IR_DIST_MM) / IR_DIST_MM); // TODO Why?

    //uint8_t ret = (uint8_t)((mm - MID_IR_POS_MM) / IR_DIST_MM + 16);
    return ret;
}

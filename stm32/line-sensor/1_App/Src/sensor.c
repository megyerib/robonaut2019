////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      sensor.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "eval.h"
#include "feedback.h"
#include "measure.h"
#include "ldriver.h"
#include "mux.h"
#include "comm.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

// Local (static) function definitions ---------------------------------------------------------------------------------

void startSensor()
{
    uint32_t measVals[32];
    LINE_SENSOR_OUT line;

    // Init
    initLDriver();
    initMux();

    enableIr();
    enableLed();

    // Loop
    while (1)
    {
        measure(measVals);
        line = getLine(measVals);
        ledFeedback(&line);
        sendLine(&line);
        HAL_Delay(10);
    }
}

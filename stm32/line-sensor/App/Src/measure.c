////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      measure.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "measure.h"
#include "adc.h"
#include "ldriver.h" // for IR
#include "mux.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define SENSOR_NUM 32

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void measureSet(uint8_t set, uint32_t dst[SENSOR_NUM]);

// Global function definitions -----------------------------------------------------------------------------------------

void measure(uint32_t dst[SENSOR_NUM])
{
    // Késõbb lehet szórakozni azzal, hogy minden 3. vagy 2. infra led világít egyszerre.
    uint32_t irval = 0x88888888, i;

    for (i = 0; i < 4; i++)
    {
        writeIr(irval);
        HAL_Delay(1); // Beállási idõ. Ezt lehet majd egy timerrel csökkenteni.

        measureSet(i, dst);
        measureSet(i + 4, dst);

        irval >>= 1;
    }
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// Beolvassa az összes ADC értékét egy adott MUX beállítás (set) mellett.
static void measureSet(uint8_t set, uint32_t dst[SENSOR_NUM])
{
    int i;

    setMux(set);

    HAL_ADC_Start(&hadc);

    for (i = 0; i < 4; i++)
    {
        HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
        dst[8*i+(7-set)] = HAL_ADC_GetValue(&hadc);
    }

    HAL_ADC_Stop(&hadc);
}

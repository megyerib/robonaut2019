////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      math_common.c
//!  \brief     
//!  \details   
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <math.h>

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

float invSqrt( float number )
{
    union {
        float f;
        uint32_t i;
    } conv;

    float x2;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    conv.f  = number;
    conv.i  = 0x5f3759df - ( conv.i >> 1 );
    conv.f  = conv.f * ( threehalfs - ( x2 * conv.f * conv.f ) );
    return conv.f;
}

uint32_t mean(uint32_t* data, uint32_t num)
{
    uint32_t sum = 0, i;

    for (i = 0; i < num; i++)
        sum += data[i];

    return sum / num;
}

uint32_t standardDeviation(uint32_t* data, uint32_t num, uint32_t avg)
{
    uint32_t sqsum = 0, var, i;

    for (i = 0; i < num; i++)
        sqsum += (data[i] - avg) * (data[i] - avg);

    var = sqsum / num;

    return sqrt(var);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      eval.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../1_App/Inc/eval.h"

#include "../../2_BSP/Inc/math_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define THRESHOLD 200
#define SENSOR_NUM 32

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

static uint32_t max4_pos(int32_t arg1, int32_t arg2, int32_t arg3, int32_t arg4);
static void     magicDiff(uint32_t* src, uint32_t* dst);
static int16_t  ledPosToMm(uint8_t ledPos);
static int32_t evalWeightedMean(uint32_t* arr, uint32_t i);
static uint32_t evalIsPeak(uint32_t* arr, uint32_t i, uint32_t mean, uint32_t stdDev);

// Global function definitions -----------------------------------------------------------------------------------------

LINE_SENSOR_OUT getLine(uint32_t measData[SENSOR_NUM])
{
    uint32_t filtered[32];
    uint32_t i;
    uint32_t avg;
    uint32_t stdDev;

    LINE_SENSOR_OUT ret =
    {
        .cnt   = 0,
        .cross = 0
    };

    // Filtering
    magicDiff(measData, filtered);

    // Average, standard deviation
    avg = mean(filtered, 32);
    stdDev = standardDeviation(filtered, 32, avg);

    // Szenzorsor
    for (i = 0; i < 32; i++)
    {
        if (ret.cnt < MAXLINES && evalIsPeak(filtered, i, avg, stdDev))
        {
            ret.lines[ret.cnt] = evalWeightedMean(filtered, i);
            ret.cnt++;
        }
    }

    // Visszatérés
    return ret;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// A 4 legközelebbi szomszédtól vett legnagyobb különbség, ha pozitív, egyébként 0.
static void magicDiff(uint32_t* src, uint32_t* dst)
{
    int i;

    dst[0] = max4_pos(
        0,
        0,
        src[0] - src[1],
        src[0] - src[2]
    );

    dst[1] = max4_pos(
        0,
        src[1] - src[1],
        src[1] - src[2],
        src[1] - src[3]
    );

    for (i = 2; i < 30; i++)
    {
        dst[i] = max4_pos(
            src[i] - src[i-2],
            src[i] - src[i-1],
            src[i] - src[i+1],
            src[i] - src[i+2]
        );
    }

    dst[30] = max4_pos(
        src[30] - src[28],
        src[30] - src[29],
        src[30] - src[31],
        0
    );

    dst[31] = max4_pos(
        src[31] - src[29],
        src[31] - src[30],
        0,
        0
    );
}

// A 4 érték közül a legnagyobbal tér vissza, ha negatív, akkor 0-val.
static uint32_t max4_pos(int32_t arg1, int32_t arg2, int32_t arg3, int32_t arg4)
{
    int32_t max = 0;

    max = arg1 > max ? arg1 : max;
    max = arg2 > max ? arg2 : max;
    max = arg3 > max ? arg3 : max;
    max = arg4 > max ? arg4 : max;

    return (uint32_t) max;
}

static int16_t ledPosToMm(uint8_t ledPos)
{
    int16_t ret = (16 - (int16_t)ledPos) * IR_DIST_MM + MID_IR_POS_MM;
    return ret;
}

static int32_t evalWeightedMean(uint32_t arr[SENSOR_NUM], uint32_t i) // TODO itt a hiba
{
    int32_t w1, w2, w3, div, ret;

    if (i == 0)
    {
       w1 = 0;
       w2 = arr[0] * ledPosToMm(0);
       w3 = arr[1] * ledPosToMm(1);

       div = arr[0] + arr[1];
    }
    else if (i == 31)
    {
        w1 = arr[30] * ledPosToMm(30);
        w2 = arr[31] * ledPosToMm(31);
        w3 = 0;

        div = arr[30] + arr[31];
    }
    else
    {
        w1 = arr[i-1] * ledPosToMm(i-1);
        w2 = arr[i  ] * ledPosToMm(i  );
        w3 = arr[i+1] * ledPosToMm(i+1);

        div = arr[i-1] + arr[i] + arr[i+1];
    }

    ret = (w1 + w2 + w3) / div;

    return ret;
}

static uint32_t evalIsPeak(uint32_t* arr, uint32_t i, uint32_t mean, uint32_t stdDev)
{
    // Threshold
    if (arr[i] < mean + stdDev)
    {
        return 0;
    }

    // Peak
    if (i == 0)
    {
        return (arr[1]  + THRESHOLD) < arr[0];
    }
    else if (i == 31)
    {
        return (arr[30] + THRESHOLD) < arr[31];
    }
    else
    {
        return
            ( ((arr[i-1] + THRESHOLD) < arr[i]) && (arr[i+1] < arr[i]) )
            ||
            ( ((arr[i+1] + THRESHOLD) < arr[i]) && (arr[i-1] < arr[i]) );
    }
}

// END -----------------------------------------------------------------------------------------------------------------

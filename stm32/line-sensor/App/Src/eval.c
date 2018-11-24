////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      eval.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "eval.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define THRESHOLD 500

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

static uint32_t max4_pos(int32_t arg1, int32_t arg2, int32_t arg3, int32_t arg4);
static void     magicDiff(uint32_t* src, uint32_t* dst);
static int16_t  ledPosToMm(uint8_t ledPos);

// Global function definitions -----------------------------------------------------------------------------------------

LINE getLine(uint32_t* measData)
{
    uint32_t filtered[32], i;
    LINE ret = {
        .cnt = 0,
        .cross = 0
    };

    // Szûrés
    magicDiff(measData, filtered);

    // Szenzorsor eleje
    if (filtered[1] + THRESHOLD < filtered[0])
    {
        ret.lines[0] =
            (filtered[0] * ledPosToMm(0) + filtered[1] * ledPosToMm(1)) /
            (filtered[0] + filtered[1]);

        ret.cnt++;
    }

    // Szenzorsor közepe
    for (i = 1; i < 31; i++)
    {
        if ((filtered[i-1] + THRESHOLD < filtered[i] && filtered[i+1] < filtered[i]) ||
            (filtered[i+1] + THRESHOLD < filtered[i] && filtered[i-1] < filtered[i]))
        {
            if (ret.cnt < MAXLINES)
            {
                ret.lines[ret.cnt] =
                    (filtered[i-1] * ledPosToMm(i-1) +
                     filtered[i]   * ledPosToMm(i)   +
                     filtered[i+1] * ledPosToMm(i+1))
                     /
                    (filtered[i-1] + filtered[i] + filtered[i+1]);

                ret.cnt++;
            }
        }
    }

    // Szenzorsor vége
    if (filtered[30] + THRESHOLD < filtered[31])
    {
        if (ret.cnt < MAXLINES)
        {
            ret.lines[ret.cnt] =
                (filtered[30] * ledPosToMm(30) + filtered[31] * ledPosToMm(31)) /
                (filtered[30] + filtered[31]);

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
    return ((int16_t)ledPos - 16) * IR_DIST_MM + MID_IR_POS_MM;
}

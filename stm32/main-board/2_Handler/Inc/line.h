////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      line.h
//!  \brief     Getting line position
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f4xx_hal.h"
#include "line_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef struct
{
    int16_t d; // TODO Originally uint
    float theta; // Radian
}
LINE;

typedef struct
{
    uint16_t d;
    uint16_t theta;
}
Arc;

typedef enum
{
    Nothing = 0,
    DoubleLine,
    TripleLine,
    // ...

    RoadSignalNum
}
RoadSignal;

typedef enum
{
    Left,
    Right
}
ArcDir;

typedef struct
{
	float lines[MAXLINES];
	int   cnt;
}
LSO_FLOAT;

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void lineInit();
float lineGetSingle();
LINE_SENSOR_OUT lineGetRawFront();
LSO_FLOAT lineGetRawFrontFloat();
LINE_SENSOR_OUT lineGetRawRear();
Arc lineGetArc(uint16_t r_mm, ArcDir dir);
RoadSignal lineGetRoadSignal();

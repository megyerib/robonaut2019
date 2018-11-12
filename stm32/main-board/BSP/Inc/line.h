#include "stm32f4xx_hal.h"

typedef struct
{
	uint16_t d;
	uint16_t theta;
}
Line;

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

void lineInit();
Line lineGet();
Arc lineGetArc(uint16_t r, ArcDir dir);
RoadSignal lineGetRoadSignal();

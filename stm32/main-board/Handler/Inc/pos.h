#include "stm32f4xx_hal.h"

typedef struct
{
	uint16_t x;
	uint16_t y;
	uint16_t theta;
}
Position;

void positionInit();
Position positionGet();
void positionSet(Position pos);

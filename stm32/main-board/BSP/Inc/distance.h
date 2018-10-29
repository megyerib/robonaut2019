#include "stm32f4xx_hal.h"

typedef enum
{
	Front = 0,
	Right,
	Left,

	DirectionNum
}
Direction;

void InitDistance();
uint16_t GetDistance(Direction dir);

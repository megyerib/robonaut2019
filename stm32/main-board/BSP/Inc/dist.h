#include "stm32f4xx_hal.h"

typedef enum // T�vols�gszenzor ir�nyok
{
	Front = 0,
	Right,
	Left,

	DirectionNum
}
Direction;

void distanceInit();
uint16_t distanceGet(Direction dir);

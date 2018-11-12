#include "stm32f4xx_hal.h"

typedef struct
{
	int16_t a_x;
	int16_t a_y;
	int16_t a_z;
	int16_t w_x;
	int16_t w_y;
	int16_t w_z;
}
Inertia;

void InitInertia();
Inertia GetInertia();

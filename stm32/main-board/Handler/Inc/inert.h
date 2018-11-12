#include "stm32f4xx_hal.h"

typedef struct
{
	uint16_t a_x;
	uint16_t a_y;
	uint16_t a_z;
	uint16_t omega_x;
	uint16_t omega_y;
	uint16_t omega_z;
}
Accel;

void inertialInit();
Accel inertialGetAccel();

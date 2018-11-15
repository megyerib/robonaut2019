#include "stm32f4xx_hal.h"

typedef struct
{
	uint16_t a_x;
	uint16_t a_y;
	uint16_t a_z;
}
Accel;

typedef struct
{
	uint16_t omega_x;
	uint16_t omega_y;
	uint16_t omega_z;
}
AngVel;

void inertialInit();
Accel inertialGetAccel();
AngVel inertialGetAngVel();

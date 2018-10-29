#include "stm32f4xx_hal.h"

typedef enum
{
	s0 = 0,
	s1,
	s2,
	s3,
	s4,
	s5,
	sNothing
}
RadioSignal;

void InitRadio();
RadioSignal GetRadioSignal();

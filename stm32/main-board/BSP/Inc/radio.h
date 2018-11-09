#include "stm32f4xx_hal.h"

typedef enum
{
	rs0 = 0,
	rs1,
	rs2,
	rs3,
	rs4,
	rs5,
	rsNothing,
	rsError,

	rsNum
}
RadioSignal;

void InitRadio();
RadioSignal GetRadioSignal();
void radioUartCallback();

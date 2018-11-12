#pragma once
#include "stm32f0xx_hal.h"

#define IR_DIST_MM    9
#define MID_IR_POS_MM 5
#define MAXLINES      3

typedef struct
{
	uint16_t lines[MAXLINES];
	uint8_t cnt;
	uint8_t cross;
}
LINE;

LINE getLine(uint32_t* measData);

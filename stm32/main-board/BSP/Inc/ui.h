#include "stm32f4xx_hal.h"

typedef enum
{
	off = 0,
	on,
	blink_fast,
	blink_slow,

	LedStateNum
}
LedState;

void uiInit();
void uiSetLedState(uint8_t led, LedState state);
GPIO_PinState uiGetButtonState();
uint8_t uiGetRotarySwitchVal();

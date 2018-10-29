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

void InitUi();
void SetLedState(uint8_t led, LedState state);
GPIO_PinState GetButtonState();
uint8_t GetRotarySwitchVal();

#include "feedback.h"

#include "../../BSP/Inc/ldriver.h"

static uint8_t mmToLedPos(uint8_t mm);

void ledFeedback(LINE* line)
{
	uint32_t ledval = 0, i;

	for (i = 0; i < line->cnt; i++)
	{
		ledval |= mmToLedPos(line->lines[i]);
		ledval |= mmToLedPos(line->lines[i] + 1);
	}

	writeLed(ledval);
}

static uint8_t mmToLedPos(uint8_t mm)
{
	return (uint8_t)((mm - MID_IR_POS_MM) / IR_DIST_MM + 16);
}


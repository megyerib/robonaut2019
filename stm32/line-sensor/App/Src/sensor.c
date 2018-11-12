// BSP
#include "eval.h"
#include "feedback.h"
#include "../../BSP/Inc/ldriver.h"
#include "../../BSP/Inc/measure.h"
#include "../../BSP/Inc/mux.h"

void startSensor()
{
	uint32_t measVals[32];
	LINE line;

	// Init
	initLDriver();
	initMux();

	enableIr();
	enableLed();

	// Loop
	while (1)
	{
		measure(measVals);
		line = getLine(measVals);
		ledFeedback(&line);
	}
}

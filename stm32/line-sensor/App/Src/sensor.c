// BSP
#include "ldriver.h"
#include "mux.h"
// APP
#include "measure.h"
#include "eval.h"
#include "feedback.h"

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

#include "measure.h"
#include "adc.h"
#include "mux.h"
#include "ldriver.h" // Az infr�hoz kell

// Static prototypes -----------------------------------------------------------

static void measureSet(uint8_t set, uint32_t* dst);

// Global ----------------------------------------------------------------------

void measure(uint32_t* dst)
{
	// K�s�bb lehet sz�rakozni azzal, hogy minden 3. vagy 2. infra led vil�g�t egyszerre.
	uint32_t irval = 0x88888888, i;

	for (i = 0; i < 4; i++)
	{
		writeIr(irval);
		HAL_Delay(1); // Be�ll�si id�. Ezt lehet majd egy timerrel cs�kkenteni.

		measureSet(i, dst);
		measureSet(i + 4, dst);

		irval >>= 1;
	}
}

// Static ----------------------------------------------------------------------

// Beolvassa az �sszes ADC �rt�k�t egy adott MUX be�ll�t�s (set) mellett.
static void measureSet(uint8_t set, uint32_t* dst)
{
	int i;

	setMux(set);

	HAL_ADC_Start(&hadc);

	for (i = 0; i < 4; i++)
	{
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		dst[8*i+(7-set)] = HAL_ADC_GetValue(&hadc);
	}

	HAL_ADC_Stop(&hadc);
}

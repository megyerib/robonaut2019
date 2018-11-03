#include "measure.h"
#include "adc.h"
#include "mux.h"

uint32_t values[32];

static void AdcSwap(uint32_t* val);

void measure()
{
	int i_adc, i_mux;

	for (i_mux = 7; i_mux >= 0; i_mux--)
	{
		SetMux(i_mux, 1);
		HAL_Delay(1);

		HAL_ADC_Start(&hadc);

		for (i_adc = 0; i_adc < 4; i_adc++)
		{
			HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
			values[8*(3-i_adc)+(7-i_mux)] = HAL_ADC_GetValue(&hadc);
		}

		HAL_ADC_Stop(&hadc);
	}

	AdcSwap(values);
}

static void AdcSwap(uint32_t* val)
{
	uint32_t tmp, i;

	for (i = 0; i < 8; i++)
	{
		tmp = val[i];
		val[i] = val[24+i];
		val[24+i] = tmp;

		tmp = val[8+i];
		val[8+i] = val[16+i];
		val[16+i] = tmp;
	}
}

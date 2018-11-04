#include "measure.h"
#include "adc.h"
#include "mux.h"
#include "ldriver.h"
#include "comm.h"

uint32_t values[32];
uint32_t magicDerivative[32];

static void AdcSwap(uint32_t* val);
static uint32_t Eval(uint32_t* val);
static uint32_t max4_pos(int32_t arg1, int32_t arg2, int32_t arg3, int32_t arg4);
static void magicDiff(uint32_t* src, uint32_t* dst);

void measure()
{
	int i_adc, i_mux;
	uint32_t irval = 0x01010101;

	InitLDriver();
	EnableIr();
	EnableLed();

	for (i_mux = 7; i_mux >= 0; i_mux--)
	{
		WriteIr(irval);
		SetMux(i_mux, 1);
		HAL_Delay(1);

		HAL_ADC_Start(&hadc);

		for (i_adc = 0; i_adc < 4; i_adc++)
		{
			HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
			values[8*(3-i_adc)+(7-i_mux)] = HAL_ADC_GetValue(&hadc);
		}

		HAL_ADC_Stop(&hadc);

		irval <<= 1;
	}

	AdcSwap(values);

	magicDiff(values, magicDerivative);

	WriteLed(Eval(magicDerivative));

	//SendFullMeasurment(values);
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

// A 4 érték közül a legnagyobbal tér vissza, ha negatív, akkor 0-val.
static uint32_t max4_pos(int32_t arg1, int32_t arg2, int32_t arg3, int32_t arg4)
{
	int32_t max = 0;

	max = arg1 > max ? arg1 : max;
	max = arg2 > max ? arg2 : max;
	max = arg3 > max ? arg3 : max;
	max = arg4 > max ? arg4 : max;

	return (uint32_t) max;
}

static void magicDiff(uint32_t* src, uint32_t* dst)
{
	int i;

	dst[0] = max4_pos(
		0,
		0,
		src[0] - src[1],
		src[0] - src[2]
	);

	dst[1] = max4_pos(
		0,
		src[1] - src[1],
		src[1] - src[2],
		src[1] - src[3]
	);

	for (i = 2; i < 30; i++)
	{
		dst[i] = max4_pos(
			src[i] - src[i-2],
			src[i] - src[i-1],
			src[i] - src[i+1],
			src[i] - src[i+2]
		);
	}

	dst[30] = max4_pos(
		src[30] - src[28],
		src[30] - src[29],
		src[30] - src[31],
		0
	);

	dst[31] = max4_pos(
		src[31] - src[29],
		src[31] - src[30],
		0,
		0
	);
}

#define THRESHOLD 500

static uint32_t Eval(uint32_t* val)
{
	uint32_t ledval, i, avg;
	ledval = 0;

	if (val[1] + THRESHOLD < val[0])
	{
		ledval |= 0x3;
	}

	for (i = 1; i < 31; i++)
	{
		if ((val[i-1] + THRESHOLD < val[i] && val[i+1] < val[i]) ||
			(val[i+1] + THRESHOLD < val[i] && val[i-1] < val[i]))
		{
			avg  = (val[i-1] * (i-1)) + (val[i] * (i)) + (val[i+1] * (i+1));
			avg /= val[i-1] + val[i] + val[i+1];

			ledval |= (1 << avg) | (1 << (avg + 1));
		}
	}

	if (val[30] + THRESHOLD < val[31])
		ledval |= (1 << 31) | (1 << 30);

	return ledval;
}

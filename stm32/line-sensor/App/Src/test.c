#include "ldriver.h"



void LedTest()
{
	uint32_t val = 0x0000000B;
	uint32_t dir = 1;

	InitLDriver();
	EnableLed();

	while(1)
	{
		if (dir == 1)
		{
			val <<= 1;

			if (val & 0x80000000)
				dir = 0;
		}
		else
		{
			val >>= 1;

			if (val & 0x00000001)
				dir = 1;
		}

		WriteLed(val);

		HAL_Delay(30);
	}
}

void IrTest()
{
	uint32_t val = 0x0000000B;
	uint32_t dir = 1;

	InitLDriver();
	EnableIr();

	while(1)
	{
		if (dir == 1)
		{
			val <<= 1;

			if (val & 0x80000000)
				dir = 0;
		}
		else
		{
			val >>= 1;

			if (val & 0x00000001)
				dir = 1;
		}

		WriteIr(val);

		HAL_Delay(30);
	}
}

void LedTest_Sweep()
{
	static uint32_t LedSweepLeds = 1;

	InitLDriver();
	EnableLed();

	while (1)
	{
		WriteLed(LedSweepLeds);

		LedSweepLeds <<= 1;
		if (LedSweepLeds == 0)
			LedSweepLeds = 1;

		HAL_Delay(30);
	}
}

void IrTest_Sweep()
{
	static uint32_t IrSweepLeds = 1;

	InitLDriver();
	EnableIr();

	while (1)
	{
		WriteIr(IrSweepLeds);

		IrSweepLeds <<= 1;
		if (IrSweepLeds == 0)
			IrSweepLeds = 1;

		HAL_Delay(30);
	}
}

void LedTest_KnightRider()
{
	uint32_t LED_KnightRiderLed = 0x0000FFFF;
	int LED_KnightRiderDir = 1;

	InitLDriver();
	EnableLed();

	while(1)
	{
		if (LED_KnightRiderDir == 1)
		{
			LED_KnightRiderLed <<= 1;

			if (LED_KnightRiderLed & 0x80000000)
				LED_KnightRiderDir = 0;
		}
		else
		{
			LED_KnightRiderLed >>= 1;

			if (LED_KnightRiderLed & 0x00000001)
				LED_KnightRiderDir = 1;
		}

		WriteLed(LED_KnightRiderLed);

		HAL_Delay(30);
	}
}

void IrTest_KnightRider()
{
	uint32_t IR_KnightRiderLed = 0x000000FF;
	int IR_KnightRiderDir = 1;

	InitLDriver();
	EnableIr();

	while(1)
	{
		if (IR_KnightRiderDir == 1)
		{
			IR_KnightRiderLed <<= 1;

			if (IR_KnightRiderLed & 0x80000000)
				IR_KnightRiderDir = 0;
		}
		else
		{
			IR_KnightRiderLed >>= 1;

			if (IR_KnightRiderLed & 0x00000001)
				IR_KnightRiderDir = 1;
		}

		WriteIr(IR_KnightRiderLed);

		HAL_Delay(30);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_pwm.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_pwm.h"
#include "tim.h"

#include "bsp_leds.h"
// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

float DutyCycleHalfBridge1;
float DutyCycleHalfBridge2;

int32_t CompareValueHigh2;		//Transistor 3	(because PCB)
int32_t CompareValueLow2;		//Transistor 4	(because PCB)
int32_t CompareValueHigh1;		//Transistor 1	(because PCB)
int32_t CompareValueLow1;		//Transistor 2	(because PCB)

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void BSP_PWMStart()
{
	HAL_TIM_Base_Start(&htim1);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PERIOD+1);	//to prevent any impulse on the output by starting
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PERIOD+1);	//to prevent any impulse on the output by starting
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PERIOD+1);	//to prevent any impulse on the output by starting
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PERIOD+1);	//to prevent any impulse on the output by starting

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	polarity = POSITIVE;
}

void BSP_CreateDutyCycle(float DutyCycle, float* DutyCycleHalfBridge1, float* DutyCycleHalfBridge2)
{
	float dutyCycle = DutyCycle;

	if (dutyCycle > DUTY_CYLE_MAX)
		{
			dutyCycle = DUTY_CYLE_MAX;
		}

	//polarity = NEGATIVE;

	if (polarity == POSITIVE)
	{
		*DutyCycleHalfBridge2 = (dutyCycle + 1) / 2;
		*DutyCycleHalfBridge1 = 1 - *DutyCycleHalfBridge2;
	}
	if (polarity == NEGATIVE)
	{
		*DutyCycleHalfBridge1 = (dutyCycle + 1) / 2;
		*DutyCycleHalfBridge2 = 1 - *DutyCycleHalfBridge1;
	}
}

void BSP_CreateCompareValue(float* DutyCycleHalfBridge, int32_t* CompareValueHigh, int32_t* CompareValueLow)
{
	float dutyCycle = *DutyCycleHalfBridge;

	if (dutyCycle > DUTY_CYCLE_HALF_BRIDGE_MAX)
	{
		dutyCycle = DUTY_CYCLE_HALF_BRIDGE_MAX;
	}

	if (dutyCycle < DUTY_CYCLE_HALF_BRIDGE_MIN)
	{
		dutyCycle = DUTY_CYCLE_HALF_BRIDGE_MIN;
	}

	*CompareValueHigh	= dutyCycle * PERIOD + DEADTIME_HIGH;
	*CompareValueLow	= dutyCycle	* PERIOD - DEADTIME_LOW;
}

void BSP_PWMSetCompareRegisters	(
								int32_t* CompareValueHigh2,		//Transistor 3	(because PCB)
								int32_t* CompareValueLow2,		//Transistor 4	(because PCB)
								int32_t* CompareValueHigh1,		//Transistor 1	(because PCB)
								int32_t* CompareValueLow1		//Transistor 2	(because PCB)
							)
{
	// 1. h�d�g
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);		           //Transistor 3 Fels� tilt
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, *CompareValueLow2);   //Transistor 4 Als� �t�r
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *CompareValueHigh2);  //Transistor 3 Fels� �t�r

	// 2. h�d�g
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);		           //Transistor 1 Fels� tilt
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, *CompareValueLow1);   //Transistor 2 Als� �t�r
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, *CompareValueHigh1);  //Transistor 1 Fels� �t�r
}

void BSP_SetDutyCycle(float* DutyCycle)
{
	BSP_CreateDutyCycle(*DutyCycle, &DutyCycleHalfBridge1, &DutyCycleHalfBridge2);
	BSP_CreateCompareValue(&DutyCycleHalfBridge1, &CompareValueHigh1, &CompareValueLow1);
	BSP_CreateCompareValue(&DutyCycleHalfBridge2, &CompareValueHigh2, &CompareValueLow2);
	BSP_PWMSetCompareRegisters	(
								&CompareValueHigh2,
								&CompareValueLow2,
								&CompareValueHigh1,
								&CompareValueLow1
						);
	BSP_SetLEDHeartbeatBlinkingDutyCyle(&CompareValueHigh1);
	BSP_SetLEDOrangeBlinkingDutyCyle(&CompareValueLow1);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END -----------------------------------------------------------------------------------------------------------------

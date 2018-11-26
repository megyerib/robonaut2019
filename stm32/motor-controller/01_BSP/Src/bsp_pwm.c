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

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void BSP_PWMStart()
{
	HAL_TIM_Base_Start(&htim1);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PERIOD/2);	//to prevent any impulse on the output by starting
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PERIOD/2);	//to prevent any impulse on the output by starting
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PERIOD/2);	//to prevent any impulse on the output by starting
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PERIOD/2);	//to prevent any impulse on the output by starting

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void BSP_CreateDutyCycle(float DutyCycle, float* DutyCycleHalfBridge1, float* DutyCycleHalfBridge2)
{
	*DutyCycleHalfBridge1 = (DutyCycle + 1) / 2;
	*DutyCycleHalfBridge2 = 1 - *DutyCycleHalfBridge1;
}

void BSP_CreateCompareValue(float* DutyCycleHalfBridge, int32_t* CompareValueHigh, int32_t* CompareValueLow)
{
	if( *DutyCycleHalfBridge < DUTY_CYCLE_HALF_BRIDGE_MAX)		//If the duty cycle of the half-bridge is "normal"
	{
		*CompareValueHigh	= *DutyCycleHalfBridge 		* PERIOD	+ DEADTIME;
		*CompareValueLow	= (1 - *DutyCycleHalfBridge)	* PERIOD	- DEADTIME;
	}
	else	//If the duty cycle of the half - bridge would be to big
	{
		*CompareValueHigh	= DUTY_CYCLE_HALF_BRIDGE_MAX 		* PERIOD	+ DEADTIME;
		*CompareValueLow	= (1 - DUTY_CYCLE_HALF_BRIDGE_MAX)	* PERIOD	- DEADTIME;
	}
}

void BSP_PWMSetCompareRegisters	(
								int32_t* CompareValueHigh2,		//Transistor 3	(because PCB)
								int32_t* CompareValueLow2,		//Transistor 4	(because PCB)
								int32_t* CompareValueHigh1,		//Transistor 1	(because PCB)
								int32_t* CompareValueLow1		//Transistor 2	(because PCB)
							)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *CompareValueHigh2);		//Transistor 3
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, *CompareValueLow2);		//Transistor 4
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, *CompareValueHigh1);		//Transistor 1
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, *CompareValueLow1);		//Transistor 2

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
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
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END -----------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_measurements.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_measurements.h"
#include "adc.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

// Local (static) function definitions ---------------------------------------------------------------------------------

void BSP_GetMeasurementValues	(
									int32_t* MotorCurrentValue,
									int32_t* ServoCurrentValue,
									int32_t* SystemCurrentValue,
									int32_t* SecBatVoltageValue,
									int32_t* MainBatVoltageValue
								)
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	*MotorCurrentValue		= HAL_ADC_GetValue(&hadc);
	*ServoCurrentValue 		= HAL_ADC_GetValue(&hadc);
	*SystemCurrentValue 	= HAL_ADC_GetValue(&hadc);
	*SecBatVoltageValue		= HAL_ADC_GetValue(&hadc);
	*MainBatVoltageValue 	= HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
}


void BSP_GetMeasurements		(
									float* MotorCurrent,
									float* ServoCurrent,
									float* SystemCurrent,
									float* SecBatVoltage,
									float* MainBatVoltage
								)
{
	BSP_GetMeasurementValues(&MotorCurrentValue, &ServoCurrentValue, &SystemCurrentValue, &SecBatVoltageValue, &MainBatVoltageValue);

	*MotorCurrent	=	MotorCurrentValue		*	CONST_MOTOR_CURRENT;
	*ServoCurrent	=	ServoCurrentValue		*	CONST_SERVO_CURRENT;
	*SystemCurrent 	=	SystemCurrentValue		*	CONST_SYSTEM_CURRENT;
	*SecBatVoltage	=	SecBatVoltageValue		*	CONST_SEC_VOLTAGE;
	*MainBatVoltage =	MainBatVoltageValue		*	CONST_MAIN_VOLTAGE;
}

// END -----------------------------------------------------------------------------------------------------------------

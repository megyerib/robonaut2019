////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		sharp.c
//!  \brief
//!  \details	This module handles the sharp distance measurements.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "sharp.h"
#include "bsp_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define 	M_1					1.0/33.0		//!< [(1/cm)/V]
#define 	B_1					-0.0242424		//!< [1/cm]
#define 	M_2					4.0/225.0		//!< [(1/cm)/V]
#define 	B_2					-0.000444444	//!< [1/cm]
#define		VERTEX				1.9 			//!< [V]

#define		WAIT_SEMAPHORE		2				//!< Ticks [ms]

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! Common resource that stores the actual measured distance.
static cMEASUREMENT_DIST sharpMeasurement;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void sharpSetMeasurement(const cMEASUREMENT_DIST meas);

static uint16_t sharpCharacteristic (const uint32_t adcValue);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc);

// Global function definitions -----------------------------------------------------------------------------------------

void sharpInit ()
{
	sharpMeasurement.Sequence = 0;

	// Initial value.
	sharpTriggerAdc();
}

void sharpTriggerAdc ()
{
	HAL_ADC_Start_IT(&BSP_SHARP_HADC);
}

cMEASUREMENT_DIST sharpGetMeasurement ()
{
	cMEASUREMENT_DIST retVal;

	if ( xSemaphoreTake(semSharp, WAIT_SEMAPHORE) == pdTRUE )
	{
		retVal = sharpMeasurement;
		xSemaphoreGive(semSharp);
	}

	return retVal;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//! @brief  Sets the calculated distance of the SHARP sensor safely
//! @param  Distance to be stored.
static void sharpSetMeasurement(const cMEASUREMENT_DIST meas)
{
	BaseType_t xTaskWokenByReceive = pdFALSE;

	if (xSemaphoreTakeFromISR(semSharp, &xTaskWokenByReceive) == pdTRUE)
	{
		sharpMeasurement = meas;
		xSemaphoreGiveFromISR(semSharp, &xTaskWokenByReceive);
	}

	if (xTaskWokenByReceive != pdFALSE)
	{
		// pdTRUE
		/* We should switch context so the ISR returns to a different task.
		   NOTE:  How this is done depends on the port you are using.  Check
		   the documentation and examples for your port. */
		taskYIELD();
	}
}

//! @brief 	Implements the inverse characteristics of the sensor and returns a
//!			distance from an adc value.
//! @param	ADC converted value from a voltage.
//! @retval	Calculated distance.
static uint16_t sharpCharacteristic (const uint32_t adcValue)
{
	float measured_voltage = 0.0;
	float calculated_distance = 0.0;

	// Converts the adc value to a voltage
	measured_voltage  = adcValue * BSP_3V3 / BSP_SHARP_ADC_RESOLUTION;

	// Approximation of the characteristic of the sensor with polygon chain.
	if(measured_voltage > VERTEX)
	{
		calculated_distance = 1.0 / (measured_voltage * M_1 + B_1);
	}
	else
	{
		calculated_distance = 1.0 / (measured_voltage * M_2 + B_2);
	}

	return (uint16_t)calculated_distance;
}

//! @brief	This function is called after the adc conversion and IT handling.
//!			Here the result of the conversion is ready to be used/saved.
//! 			Equation used: d = 1/y, where y = x * m + b
//! @param	Pointer to the handler of the ADC.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
	if (hadc->Instance == ADC1)
	{
		uint32_t adcValue;
		cMEASUREMENT_DIST meas;

		// Get the ADC result.
		adcValue = HAL_ADC_GetValue(&BSP_SHARP_HADC);

		// Calculate the distance and increment the sequence number.
		meas.Distance = sharpCharacteristic(adcValue);
		meas.Sequence++;

		sharpSetMeasurement(meas);
	}
}

// --------------------------------------------------------------------------//

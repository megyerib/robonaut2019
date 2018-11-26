/*
 * sds_SharpDistanceSensor.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

// ------------------------------- Includes -------------------------------- //

#include "../../2_Handler/Inc/sds_SharpDistanceSensor.h"

#include "../../3_BSP/Inc/bsp_common.h"
#include "../../3_BSP/Inc/bsp_sharp.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define 	M_1			1.0/33.0		// [(1/cm)/V]
#define 	B_1			-0.0242424		// [1/cm]
#define 	M_2			4.0/225.0		// [(1/cm)/V]
#define 	B_2			-0.000444444	// [1/cm]
#define		VERTEX		1.9 			// [V]

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/**
 * @brief 	Implements the inverse characteristics of the sensor and returns a
 * 			distance from an adc value.
 * @param	ADC converted value from a voltage.
 * @retval	Calculated distance.
 */
uint16_t sds_CalculateDistance (uint32_t adcValue);

/**
 * @brief	This function is called after the adc conversion and IT handling.
 * 			Here the result of the conversion is ready to be used/saved.
 * 			Equation used: d = 1/y, where y = x * m + b
 * @param	Pointer to the handler of the ADC.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc);

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

// Common resource that stores the actual measured distance.
static uint16_t sds_distance = 0;

// --------------------------------------------------------------------------//

// ------------------------------- Functions --------------------------------//

const uint16_t sds_GetDistance ()
{
	uint16_t distance;

	xSemaphoreTake(semSharp, portMAX_DELAY);
	distance = sds_distance;
	xSemaphoreGive(semSharp);

	return distance;
}

void sds_SetDistance(const uint16_t distance)
{
	xSemaphoreTake(semSharp, portMAX_DELAY);
	sds_distance = distance;
	xSemaphoreGive(semSharp);
}

void sds_ADC_Conversion ()
{
	HAL_ADC_Start_IT(&BSP_SHARP_HADC);
}

uint16_t sds_CalculateDistance (uint32_t adcValue)
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
	if (hadc->Instance == ADC1)
	{
		BaseType_t *taskWoken = pdFALSE;
		uint32_t adcValue = HAL_ADC_GetValue(&BSP_SHARP_HADC);
		uint16_t distance = sds_CalculateDistance(adcValue);

		xSemaphoreTakeFromISR(semSharp, taskWoken);
		sds_distance = distance;
		xSemaphoreGiveFromISR(semSharp, taskWoken);
	}
}

// --------------------------------------------------------------------------//

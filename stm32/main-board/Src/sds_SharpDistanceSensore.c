/*
 * sds_SharpDistanceSensore.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

// --------------------- Includes ----------------------- //

#include "sds_SharpDistanceSensore.h"
#include "bsp.h"

// -------------------------------------------------------//

// ---------------------- Defines ------------------------//

#define 	ADC_12_BIT 			4096;
#define 	MAX_PIN_VOLTAGE		3.3;

// -------------------------------------------------------//

// --------------------- Declarations --------------------//

void sds_CalculateDistance ();

// -------------------------------------------------------//

// ---------------------- Variables ----------------------//

static uint16_t sds_adcValue = 0;
static uint16_t sds_distance = 0;

// -------------------------------------------------------//

// ---------------------- Functions ----------------------//

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


void sds_CalculateDistance ()
{
	float voltage = 0.0;
	float distance = 0.0;

	voltage  = sds_adcValue * 3.3 / ADC_12_BIT;

	if(voltage > 1.9)
	{
		distance = 1.0 / ( voltage * 1.0/33.0 - 0.0242424 );
	}
	else
	{
		distance = 1.0 / ( voltage * 4.0/225.0 - 0.000444444 );
	}

	sds_SetDistance(distance);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
	if (hadc->Instance == ADC1)
	{
		//ADCHandler();
	}
}

// -------------------------------------------------------//

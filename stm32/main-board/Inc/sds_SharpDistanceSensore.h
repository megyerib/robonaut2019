/*
 * sds_SharpDIstanceSensore.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#ifndef SDS_SHARPDISTANCESENSORE_H_
#define SDS_SHARPDISTANCESENSORE_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "bsp.h"

SemaphoreHandle_t semSharp;

const uint16_t sds_GetDistance ();
void sds_SetDistance(const uint16_t distance);
void sds_ADC_Conversion ();
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc);

#endif /* SDS_SHARPDISTANCESENSORE_H_ */

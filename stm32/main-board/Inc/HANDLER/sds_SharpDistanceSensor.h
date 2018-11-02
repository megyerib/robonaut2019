/*
 * sds_SharpDIstanceSensor.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#ifndef SDS_SHARPDISTANCESENSOR_H_
#define SDS_SHARPDISTANCESENSOR_H_

// ------------------------------- Includes -------------------------------- //

#include "FreeRTOS.h"
#include "semphr.h"
#include "BSP/bsp.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

/*
 * @brief	Semaphore to the module's sds_distance private variable.
 */
SemaphoreHandle_t semSharp;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/**
  * @brief  Gets the calculated distance of the SHARP sensor safely.
  *
  * @retval Distance.
  */
const uint16_t sds_GetDistance ();

/**
  * @brief  Sets the calculated distance of the SHARP sensor safely
  *
  * @param  Distance to be stored.
  */
void sds_SetDistance(const uint16_t distance);

/**
  * @brief  Starts the ADC conversion and will generate an IT when it is ready.
  */
void sds_ADC_Conversion ();

// --------------------------------------------------------------------------//

#endif /* SDS_SHARPDISTANCESENSOR_H_ */

/*
 * bsp.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#ifndef BSP_H_
#define BSP_H_

// ------------------------------- Includes -------------------------------- //

#include "adc.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define 	DEFAULT_STACK_SIZE 				128
#define 	TASK_SHARP_PRIO						tskIDLE_PRIORITY+1
#define 	TASK2_PRIO						tskIDLE_PRIORITY+2

#define 	BSP_3V3							3.3

#define  	BSP_SHARP_HADC					hadc1
#define 	BSP_SHARP_ADC_CH 				4
#define 	BSP_SHARP_ADC_RESOLUTION 	    4096

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern ADC_HandleTypeDef hadc1;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/**
  * @brief  Initializes the ADC associated with the SHARP sensor.
  */
void BSP_Sharp_ADC_Init();

// --------------------------------------------------------------------------//


#endif /* BSP_H_ */

/*
 * bsp.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#ifndef BSP_H_
#define BSP_H_

// ------------------------------- Includes -------------------------------- //

#include "bsp_servo.h"
#include "adc.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define 	DEFAULT_STACK_SIZE 				128
#define 	TASK_SHARP_PRIO					tskIDLE_PRIORITY+3
#define 	TASK_SRV_PRIO					tskIDLE_PRIORITY+2
#define 	TASK_NAVI_PRIO					tskIDLE_PRIORITY+1

#define 	BSP_3V3							3.3

#define  	BSP_SHARP_HADC					hadc1
#define 	BSP_SHARP_ADC_CH 				4
#define 	BSP_SHARP_ADC_RESOLUTION 	    4096

#define 	TRUE							1
#define		FALSE							0

#define 	PI								3.14159265359

#define		BSP_DELAY_40_MS					40		// Depends on the Tick
#define		BSP_DELAY_20_MS					20
#define		BSP_DELAY_16_MS					16

// --------------------------------------------------------------------------//

// --------------------------------- Enum -----------------------------------//

typedef enum BSP_STATUS
{
	BSP_OK = 0,
	BSP_FAIL
} BSP_STATUS;

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

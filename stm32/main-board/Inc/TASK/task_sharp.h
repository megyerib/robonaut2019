/*
 * task_sharp.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#ifndef TASK_SHARP_H_
#define TASK_SHARP_H_

// ------------------------------- Includes -------------------------------- //

#include "FreeRTOS.h"
#include "event_groups.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern EventGroupHandle_t event_sharp;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/**
  * @brief  Initializes Task_Sharp task. It creates semaphore for the sds
  * 		module and calls the BSP_Sharp_ADC_Init() function.
  */
void TaskInit_Sharp(void* p);

/**
 * @brief	Task function that periodically updates the the measured distance
 * 			value of the sds module.
 */
void Task_Sharp(void* p);

// --------------------------------------------------------------------------//

#endif /* TASK_SHARP_H_ */

/*
 * bsp.h
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#ifndef BSP_H_
#define BSP_H_

#include "adc.h"

extern ADC_HandleTypeDef hadc1;

#define 	BSP_3V3							3.3

#define  	BSP_SHARP_HADC					hadc1
#define 	BSP_SHARP_ADC_CH 				4
#define 	BSP_SHARP_ADC_RESOLUTION 	    4096

void BSP_Sharp_ADC_Init();


#endif /* BSP_H_ */

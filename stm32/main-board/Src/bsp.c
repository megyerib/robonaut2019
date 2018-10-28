/*
 * bsp.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "bsp.h"


void BSP_Sharp_ADC_Init()
{
	MX_ADC1_Init();
}

/*
 * task_sharp.c
 *
 *  Created on: 2018. okt. 27.
 *      Author: Joci
 */

#include "bsp.h"
#include "task_sharp.h"
#include "sds_SharpDistanceSensore.h"

void TaskInit_Sharp(void * p)
{
	semSharp = xSemaphoreCreateBinary();
	if(semSharp != NULL)
	{
		xSemaphoreGive(semSharp);
		BSP_Sharp_ADC_Init();
	}
}

void Task_Sharp(void * p)
{

}

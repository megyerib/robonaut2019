#include "stm32f0xx_hal.h"
#include "bsp_pwm.h"
#include "bsp_emergency_stop.h"
#include "tim.h"

void BSP_emergency_stop_init()
{
	HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	float StopDutyCycle = 0.0;

	if(htim->Instance == TIM6)	//Motor leállító parancsot itt adjuk meg, ha lejár a timer.
	{
		BSP_SetDutyCycle(&StopDutyCycle);
	}
}

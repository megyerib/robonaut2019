#include "stm32f0xx_hal.h"
#include "bsp_pwm.h"
#include "bsp_emergency_stop.h"
#include "tim.h"

#define ER_TIMER_HANDLE    &htim6
#define ER_TIMER_INSTANCE  TIM6

void BSP_emergency_stop_init()
{
	HAL_TIM_Base_Start_IT(ER_TIMER_HANDLE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	float StopDutyCycle = 0.0;

	if(htim->Instance == ER_TIMER_INSTANCE)	//Motor leállító parancsot itt adjuk meg, ha lejár a timer.
	{
		//BSP_SetDutyCycle(&StopDutyCycle);
	}
}

void BSP_emergency_stop_reset_timer()
{
	__HAL_TIM_SET_COUNTER(ER_TIMER_HANDLE, 0);
}

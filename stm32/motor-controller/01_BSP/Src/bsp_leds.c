////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_leds.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f0xx_hal.h"
#include "bsp_leds.h"
#include "gpio.h"
#include "tim.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

// Local (static) function definitions ---------------------------------------------------------------------------------

void BSP_SetLEDBurstOFF5V(void)
{
	 HAL_GPIO_WritePin(BurstOFF5V_GPIO_Port, BurstOFF5V_Pin, GPIO_PIN_SET);
}

void BSP_SetLEDBurtsOFF6V(void)
{
	HAL_GPIO_WritePin(BurstOFF6V_GPIO_Port, BurstOFF6V_Pin, GPIO_PIN_SET);
}

void BSP_SetLEDHeartbeat(void)
{
	HAL_TIM_Base_Start(&htim3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PERIOD_LED_HEARTBEAT);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void BSP_SetLEDHeartbeatBlinking(void)
{
	HAL_TIM_Base_Start(&htim3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PERIOD_LED_HEARTBEAT/2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void BSP_SetLEDHeartbeatBlinkingDutyCyle(float* DutyCyle)
{
	HAL_TIM_Base_Start(&htim3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, *DutyCyle);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void BSP_SetLEDOrange(void)
{
	HAL_TIM_Base_Start(&htim3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PERIOD_LED_HEARTBEAT);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void BSP_SetLEDOrangeBlinking(void)
{
	HAL_TIM_Base_Start(&htim3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PERIOD_LED_HEARTBEAT/2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void BSP_SetLEDFault(void)
{
	HAL_GPIO_WritePin(LEDProblem_GPIO_Port, LEDProblem_Pin, GPIO_PIN_SET);
}

/*
void BSP_ResetLEDBurstOFF5V(void)		//This LED must always set on.
{
	HAL_GPIO_WritePin(BurstOFF5V_GPIO_Port, BurstOFF5V_Pin, GPIO_PIN_RESET);
}

void BSP_ResetEDBurtsOFF6V(void)		This LED must always set on.
{
	HAL_GPIO_WritePin(BurstOFF6V_GPIO_Port, BurstOFF6V_Pin, GPIO_PIN_RESET);
}
*/

void BSP_ResetLEDHeartbeat(void)
{
	HAL_TIM_Base_Start(&htim3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void BSP_ResetLEDOrange(void)
{
	HAL_TIM_Base_Start(&htim3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void BSP_ResetLEDFault(void)
{
	HAL_GPIO_WritePin(LEDProblem_GPIO_Port, LEDProblem_Pin, GPIO_PIN_RESET);
}

void BSP_LEDStart(void)
{
	BSP_SetLEDBurstOFF5V();
	BSP_SetLEDBurtsOFF6V();
}

// END -----------------------------------------------------------------------------------------------------------------

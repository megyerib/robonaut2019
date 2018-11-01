/*
 * bsp_servo.c
 *
 *  Created on: 2018. nov. 1.
 *      Author: Joci
 */


// ------------------------------- Includes -------------------------------- //

#include "BSP/bsp_servo.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

SRV_INIT_STATUS bsp_Validate_Servo_Motor(SRV_MOTOR* servo);

const SRV_TIM_STATUS bsp_Check_Servo_PWM(SRV_MOTOR* servo);

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

SRV_MOTOR FutabaS3003;

// --------------------------------------------------------------------------//

// ------------------------------ Functions ---------------------------------//


const SRV_INIT_STATUS bsp_Servo_Init(void)
{
	SRV_MOTOR servo;
	SRV_TIM_STATUS status;
	SRV_INIT_STATUS ret_val = SRV_INIT_OK;

	// Choose from available servo models with BSP_ACTUAL_SERVO define.
	ret_val = bsp_Validate_Servo_Motor(&servo);

	if(ret_val == SRV_INIT_OK)
	{
		// Valid servo model is chosen.

		status = bsp_Check_Servo_PWM(&servo);
		if(status == SRV_TIM_STAT_OK)
		{
			// PWM settings are in order.

			// Init OK

		}
		else
		{
			// Error: Servo timer (PWM) configuration indicates an error.
			ret_val = SRV_INITFAIL_SRV_PWM;
		}
	}

	return ret_val; // Maybe deinit so the servo will not be killed.
}

void bsp_Servo_Rotate(uint16_t theta)
{

}

SRV_INIT_STATUS bsp_Validate_Servo_Motor(SRV_MOTOR* servo)
{
	SRV_INIT_STATUS ret_val = SRV_INIT_OK;

	switch(BSP_ACTUAL_SERVO) {

		case SRV_FUTABAS3003  :
			servo->PWM_freq = 50;
			servo->PWM_cntr_freq = 84000;	// 62500;
			servo->PWM_prescaler = 1680;	// 1343;
			servo->PWM_period = 999;		// 1249;

			servo->Right_End = 28;			// 0°
			servo->Deg_30 = 42;
			servo->Deg_90 = 70;
			servo->Deg_150 = 98;			// 180°
			servo->Left_End = 112;
			servo->Delta_Deg = (servo->Deg_90 - servo->Right_End) / 90;
			break;

	  // case constant-expression  :
	  //   statement(s);
	  //    break; /* optional */

		default :
			ret_val = SRV_INIT_FAIL_SRV_MODELL;
	}

	return ret_val;
}

const SRV_TIM_STATUS bsp_Check_Servo_PWM(SRV_MOTOR* servo)
{
	int ret_val = SRV_TIM_STAT_OK;

	// Get the frequency of TIM2 (APB2). Equals with 2xPCLK1 accord. to CubeMX.
	uint32_t PCLK1 = HAL_RCC_GetSysClockFreq();
	uint32_t APB1 = PCLK1 * 2;

	uint16_t cntr_freq;
	uint16_t freq;

	if(BSP_SRV_HTIM2.Instance != BSP_SRV_TIM_INSATNCE)
		ret_val |= SRV_TIM_STAT_ERR_INSTANCE;

	if(BSP_SRV_HTIM2.Channel != BSP_SRV_TIM_CHANNEL)
		ret_val |= SRV_TIM_STAT_ERR_CHANNEL;

	if(APB1 != BSP_SRV_TIM_APB1_FREQ)
		ret_val |= SRV_TIM_STAT_ERR_APB1_FREQ;

	cntr_freq = APB1 / (BSP_SRV_HTIM2.Init.Prescaler + 1);
	if( cntr_freq != servo->PWM_cntr_freq)
		ret_val |= SRV_TIM_STAT_ERR_CNTR_FREQ;

	freq = cntr_freq / (BSP_SRV_HTIM2.Init.Period + 1);
	if(freq != servo->PWM_freq)
		ret_val |= SRV_TIM_STAT_ERR_FREQ;

	if(BSP_SRV_HTIM2.Init.Prescaler != servo->PWM_prescaler)
		ret_val |= SRV_TIM_STAT_ERR_PRESCALER;

	if(BSP_SRV_HTIM2.Init.Period != servo->PWM_period)
		ret_val |= SRV_TIM_STAT_ERR_PERIOD;

	return ret_val;
}

// --------------------------------------------------------------------------//

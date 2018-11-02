/*
 * bsp_servo.c
 *
 *  Created on: 2018. nov. 1.
 *      Author: Joci
 */


// ------------------------------- Includes -------------------------------- //

#include "BSP/bsp_servo.h"
#include "stm32f4xx_hal_tim.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define		BSP_SRV_HTIM2			 htim2
#define		BSP_SRV_TIM_INSATNCE	 TIM2
#define		BSP_SRV_TIM_CHANNEL		 TIM_CHANNEL_1

#define 	BSP_SRV_TIM_APB1_FREQ    84000000		   // Hz

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/*
 * @brief	Checks if the PWM settings of the servo module variable are correct
 * 			according to the HW and the validated SRV_MOTOR.
 *
 * @retval	Status of the testing.
 */
const BSP_Servo_TIM_Stat bsp_Servo_Check_PWM();

/*
 * @brief	Initializes the servo timer according to the setting of the hsrv.
 */
void bsp_Servo_TIM_Init();

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern TIM_HandleTypeDef htim2;

BSP_SrvHandleTypeDef hsrv;

// --------------------------------------------------------------------------//

// ------------------------------ Functions ---------------------------------//


const BSP_SrvInitStat bsp_Servo_Init_PWM(void)
{
	BSP_SrvInitStat status = SRV_INIT_OK;
	BSP_Servo_TIM_Stat tim_stat = SRV_TIM_STAT_OK;

	// Init the servo timer
	bsp_Servo_TIM_Init();

	// Check if the settings are correct
	tim_stat = bsp_Servo_Check_PWM();

	if(tim_stat == SRV_TIM_STAT_OK)
	{
		// PWM settings are in order.

		// Servo PWM is on the pin
		HAL_TIM_PWM_Start(&BSP_SRV_HTIM2, BSP_SRV_TIM_CHANNEL);

		// Set the servo in the middle position
		bsp_Servo_Set_Compare(hsrv.Deg_90);
	}
	else
	{
		// Error: Servo timer (PWM) configuration indicates an error.
		status = SRV_INIT_FAIL_SRV_PWM;

		// Disable CLK
		bsp_Servo_Disable_TIM();
	}

	return status;
}

void bsp_Servo_Disable_TIM(void)
{
	HAL_TIM_PWM_MspDeInit(&BSP_SRV_HTIM2);
}

void bsp_Servo_Enable_TIM(void)
{
	HAL_TIM_PWM_MspInit(&BSP_SRV_HTIM2);
}

void bsp_Servo_Set_Compare(const uint32_t value)
{
	uint32_t valid_value = value;

	// Makes sure that the value is in the valid interval
	if(value < hsrv.Right_End)
	{
		valid_value = hsrv.Right_End;
	}
	else if(value > hsrv.Left_End)
	{
		valid_value = hsrv.Left_End;
	}

	__HAL_TIM_SET_COMPARE(&BSP_SRV_HTIM2, BSP_SRV_TIM_CHANNEL, valid_value);
}

const uint32_t bsp_Servo_Get_Compare()
{
	return __HAL_TIM_GET_COMPARE(&BSP_SRV_HTIM2, BSP_SRV_TIM_CHANNEL);
}

const BSP_Servo_TIM_Stat bsp_Servo_Check_PWM()
{
	int ret_val = SRV_TIM_STAT_OK;

	// Get the frequency of TIM2 (APB2). Equals with 2xPCLK1 accord. to CubeMX.
	volatile uint32_t PCLK1 = HAL_RCC_GetPCLK1Freq();
	volatile uint32_t APB1 = PCLK1 * 2;

	volatile uint16_t cntr_freq;
	volatile uint16_t freq;

	// Check the Instance of the timer.
	if(BSP_SRV_HTIM2.Instance != BSP_SRV_TIM_INSATNCE)
		ret_val |= SRV_TIM_STAT_ERR_INSTANCE;

	// Check the Channel of the timer.
	if(BSP_SRV_HTIM2.Channel != BSP_SRV_TIM_CHANNEL)
		ret_val |= SRV_TIM_STAT_ERR_CHANNEL;

	// Check the Base Frequency of the timer.
	if(APB1 != BSP_SRV_TIM_APB1_FREQ)
		ret_val |= SRV_TIM_STAT_ERR_APB1_FREQ;

	// Check the Counter Frequency of the timer.
	cntr_freq = APB1 / (hsrv.PWM_prescaler + 1);
	if( cntr_freq != hsrv.PWM_cntr_freq)
		ret_val |= SRV_TIM_STAT_ERR_CNTR_FREQ;

	// Check the Frequency of the timer.
	freq = cntr_freq / (hsrv.PWM_period + 1);
	if(freq != hsrv.PWM_freq)
		ret_val |= SRV_TIM_STAT_ERR_FREQ;

	// Check the Prescaler of the timer.
	if(BSP_SRV_HTIM2.Init.Prescaler != hsrv.PWM_prescaler)
		ret_val |= SRV_TIM_STAT_ERR_PRESCALER;

	// Check the Period of the timer.
	if(BSP_SRV_HTIM2.Init.Period != hsrv.PWM_period)
		ret_val |= SRV_TIM_STAT_ERR_PERIOD;

	return ret_val;
}

void bsp_Servo_TIM_Init()
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	// Configure up the servo timer
	BSP_SRV_HTIM2.Instance = BSP_SRV_TIM_INSATNCE;
	BSP_SRV_HTIM2.Init.Prescaler = hsrv.PWM_prescaler;
	BSP_SRV_HTIM2.Init.CounterMode = TIM_COUNTERMODE_UP;
	BSP_SRV_HTIM2.Init.Period = hsrv.PWM_period;
	BSP_SRV_HTIM2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&BSP_SRV_HTIM2) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&BSP_SRV_HTIM2, &sMasterConfig) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&BSP_SRV_HTIM2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

// --------------------------------------------------------------------------//

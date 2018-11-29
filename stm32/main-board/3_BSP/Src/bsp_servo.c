////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_servo.c
//!  \brief     Controls the servo.
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ------------------------------- Includes -------------------------------- //

#include "bsp_servo.h"

#include "stm32f4xx_hal_tim.h"
#include "tim.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define		BSP_SRV_HTIM2			 htim2
#define		BSP_SRV_TIM_INSATNCE	 TIM2
#define		BSP_SRV_TIM_CHANNEL		 TIM_CHANNEL_1

#define 	BSP_SRV_TIM_APB1_FREQ    84000000		   // Hz

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//


static eBSP_Servo_TIM_Stat bspServoCheckPWM();

static void bspServoTimerInit();

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern TIM_HandleTypeDef htim2;

cBSP_SrvHandleTypeDef hsrv;

// --------------------------------------------------------------------------//

// ------------------------------ Functions ---------------------------------//


const eBSP_SrvInitStat bspServoInit(void)
{
	eBSP_SrvInitStat status = SRV_INIT_OK;
	eBSP_Servo_TIM_Stat tim_stat = SRV_TIM_STAT_OK;

	// Init the servo timer
	bspServoTimerInit();

	// Check if the settings are correct
	tim_stat = bspServoCheckPWM();

	if(tim_stat == SRV_TIM_STAT_OK)
	{
		// PWM settings are in order.

		// Servo PWM is on the pin
		HAL_TIM_PWM_Start(&BSP_SRV_HTIM2, BSP_SRV_TIM_CHANNEL);

		// Set the servo in the middle position
		bspServoSetCompare(hsrv.Deg_90);
	}
	else
	{
		// Error: Servo timer (PWM) configuration indicates an error.
		status = SRV_INIT_FAIL_SRV_PWM;

		// Disable CLK
		bspServoTimerDisable();
	}

	return status;
}

void bspServoTimerDisable(void)
{
	HAL_TIM_PWM_MspDeInit(&BSP_SRV_HTIM2);
}

void bspServoTimerEnable(void)
{
	HAL_TIM_PWM_MspInit(&BSP_SRV_HTIM2);
}

void bspServoSetCompare(const uint32_t value)
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

const uint32_t bspServoGetCompare()
{
	return __HAL_TIM_GET_COMPARE(&BSP_SRV_HTIM2, BSP_SRV_TIM_CHANNEL);
}

/*
 * @brief	Checks if the PWM settings of the servo module variable are correct
 * 			according to the HW and the validated SRV_MOTOR.
 *
 * @retval	Status of the testing.
 */
static eBSP_Servo_TIM_Stat bspServoCheckPWM()
{
	int ret_val = SRV_TIM_STAT_OK;

	// Get the frequency of TIM2 (APB2). Equals with 2xPCLK1 accord. to CubeMX.
	uint32_t PCLK1 = HAL_RCC_GetPCLK1Freq();
	uint32_t APB1 = PCLK1 * 2;

	uint16_t cntr_freq;
	uint16_t freq;

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


//! @brief	Initializes the servo timer according to the setting of the hsrv.
static void bspServoTimerInit()
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

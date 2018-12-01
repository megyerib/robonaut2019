/*
 * bsp_servo.h
 *
 *  Created on: 2018. nov. 1.
 *      Author: Joci
 */

#ifndef BSP_BSP_SERVO_H_
#define BSP_BSP_SERVO_H_

// ------------------------------- Includes -------------------------------- //

#include "tim.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define		BSP_SRV_HTIM2			 htim2
#define		BSP_SRV_TIM_INSATNCE	 TIM2
#define		BSP_SRV_TIM_CHANNEL		 TIM_CHANNEL_2

#define 	BSP_SRV_TIM_APB1_FREQ    84000000		   // Hz

#define 	BSP_ACTUAL_SERVO		 SRV_FUTABAS3003   // Choose from SRV_MODEL

// --------------------------------------------------------------------------//

// --------------------------------- Enums ----------------------------------//

typedef enum SRV_MODEL
{
	SRV_FUTABAS3003 				= 0
} SRV_MODEL;

typedef enum SRV_TIM_STATUS
{
	SRV_TIM_STAT_OK 				= 0,
	SRV_TIM_STAT_ERR_APB1_FREQ		= 1,
	SRV_TIM_STAT_ERR_INSTANCE		= 2,
	SRV_TIM_STAT_ERR_CHANNEL		= 4,
	SRV_TIM_STAT_ERR_CNTR_FREQ		= 8,
	SRV_TIM_STAT_ERR_FREQ			= 16,
	SRV_TIM_STAT_ERR_PRESCALER		= 32,
	SRV_TIM_STAT_ERR_PERIOD			= 64
} SRV_TIM_STATUS;

typedef enum SRV_INIT_STATUS
{
	SRV_INIT_OK 					= 0,
	SRV_INIT_FAIL_SRV_MODELL,
	SRV_INITFAIL_SRV_PWM,
} SRV_INIT_STATUS;

// --------------------------------------------------------------------------//

// -------------------------------- Structs ---------------------------------//

/*
 * Informations and configuration according to the given servo motor datasheet.
 *
 * PWM timer config example:
 *
 * 		Prescaler = ( APB1_clk    / TIM_counter_clk ) - 1
 * 			 1343 = ( 84 MHz	  / 62500  		    ) - 1
 *
 * 	  	   Period = ( TIM_counter_clk / f  ) - 1
 * 	    	 1249 = ( 62500			  / 50 ) - 1
 */
typedef struct SRV_MOTOR
{
	// PWM properties
	uint16_t PWM_freq;			// Analog: ~(50-60)Hz. Digital: ~(250-333)Hz.
	uint32_t PWM_cntr_freq;		// TIM2: 32 bit
	uint16_t PWM_prescaler;		// 16 bit
	uint16_t PWM_period;		// 16 bit

	// Rotational properties
	uint16_t Right_End;			// element of [0°,90°[ interval
	uint16_t Deg_30;
	uint16_t Deg_90;			// 1,5 ms ~ 90°
	uint16_t Deg_150;
	uint16_t Left_End;			// element of ]90°,180°] interval
	double Delta_Deg;			// 1° = delta_deg increment
} SRV_MOTOR;

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern TIM_HandleTypeDef htim2;

extern SRV_MOTOR FutabaS3003;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

const SRV_INIT_STATUS bsp_Servo_Init(void);

void bsp_Servo_Rotate(uint16_t theta);

// --------------------------------------------------------------------------//

#endif /* BSP_BSP_SERVO_H_ */


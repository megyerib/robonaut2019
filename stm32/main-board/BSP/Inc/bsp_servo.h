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
#include "bsp_servo.h"
#include "adc.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define 	DEFAULT_STACK_SIZE 				128
#define 	TASK_SHARP_PRIO					tskIDLE_PRIORITY+3
#define 	TASK_SRV_PRIO					tskIDLE_PRIORITY+2
#define 	TASK_NAVI_PRIO					tskIDLE_PRIORITY+1

#define 	BSP_3V3							3.3

#define  	BSP_SHARP_HADC					hadc1
#define 	BSP_SHARP_ADC_CH 				4
#define 	BSP_SHARP_ADC_RESOLUTION 	    4096

#define 	TRUE							1
#define		FALSE							0

#define 	PI								3.14159265359

#define		BSP_DELAY_40_MS					40		// Depends on the Tick
#define		BSP_DELAY_20_MS					20
#define		BSP_DELAY_16_MS					16


// --------------------------------------------------------------------------//

// --------------------------------- Enums ----------------------------------//

/*
 * @brief	Timer test status and error codes. The error can be the sum of the
 * 			individual codes.
 */
typedef enum BSP_Servo_TIM_Stat
{
	SRV_TIM_STAT_OK 				= 0,
	SRV_TIM_STAT_ERR_APB1_FREQ		= 1,
	SRV_TIM_STAT_ERR_INSTANCE		= 2,
	SRV_TIM_STAT_ERR_CHANNEL		= 4,
	SRV_TIM_STAT_ERR_CNTR_FREQ		= 8,
	SRV_TIM_STAT_ERR_FREQ			= 16,
	SRV_TIM_STAT_ERR_PRESCALER		= 32,
	SRV_TIM_STAT_ERR_PERIOD			= 64
} BSP_Servo_TIM_Stat;


/*
 * @brief	Servo module init status. Init must check the selected type of the
 * 			servo and the timer configuration.
 */
typedef enum BSP_SrvInitStat
{
	SRV_INIT_OK 					= 0,
	SRV_INIT_FAIL_SRV_MODELL,
	SRV_INIT_FAIL_SRV_PWM,
} BSP_SrvInitStat;

typedef enum BSP_STATUS
{
	BSP_OK = 0,
	BSP_FAIL
} BSP_STATUS;

// --------------------------------------------------------------------------//

// -------------------------------- Structs ---------------------------------//

/*
 * @brief	Informations and configuration according to the given servo motor
 * 			datasheet or oscilloscope measurement.
 *
 *
 * @example	PWM timer config example:
 *
 * 				Prescaler = ( APB1_clk    / TIM_counter_clk ) - 1
 * 					 1343 = ( 84 MHz	  / 62500  		    ) - 1
 *
 * 	  	   			Period = ( TIM_counter_clk / f  ) - 1
 * 	    			  1249 = ( 62500		   / 50 ) - 1
 */
typedef struct BSP_SrvHandleTypeDef
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

	// Characteristics: theta = position * m + b
	double Gradient;			// m
	double Y_intercept;			// b
} BSP_SrvHandleTypeDef;

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern BSP_SrvHandleTypeDef hsrv;
extern ADC_HandleTypeDef hadc1;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/*
 * @brief	Initializes the servo, checks the PWM and the servo configuration.
 * 			On successful init the PWM can be used, on unsuccessful init TIM
 * 			clk is disabled so no harm can be done.
 *
 * @retval	Init was successful or not: SRV_INIT_STATUS
 */
const BSP_SrvInitStat bsp_Servo_Init_PWM(void);

/*
 * @brief	Disables the servo timer clock.
 */
void bsp_Servo_Disable_TIM(void);

/*
 * @brief	Enables the servo timer.
 */
void bsp_Servo_Enable_TIM(void);

/*
 * @brief	Sets the timer compare to a given value and changes the duty cycle
 * 			of the PWM. Servo will rotate to this position.
 *
 * @param	_pos_ : The desired compare value.
 */
void bsp_Servo_Set_Compare(const uint32_t pos);

/*
 * @brief	Gets the servo timer compare value.
 *
 * @retval	The compare value.
 */
const uint32_t bsp_Servo_Get_Compare();

// --------------------------------------------------------------------------//

#endif /* BSP_BSP_SERVO_H_ */


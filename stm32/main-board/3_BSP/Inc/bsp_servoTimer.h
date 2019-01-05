////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_servoTimer.h
//!  \brief		This module handles the servo timer.
//!  \details   Initializes the timer according to the chosen servo (analog/digital). It holds limits to the pwm
//! 			signal so the servo can not be harmed.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

//! @brief	Timer test status and error codes. The error can be the sum of the individual codes.
typedef enum
{
	SRV_TIM_STAT_OK 				= 0,
	SRV_TIM_STAT_ERR_APB1_FREQ		= 1,
	SRV_TIM_STAT_ERR_INSTANCE		= 2,
	SRV_TIM_STAT_ERR_CHANNEL		= 4,
	SRV_TIM_STAT_ERR_CNTR_FREQ		= 8,
	SRV_TIM_STAT_ERR_FREQ			= 16,
	SRV_TIM_STAT_ERR_PRESCALER		= 32,
	SRV_TIM_STAT_ERR_PERIOD			= 64
} eBSP_Servo_TIM_Stat;

//! @brief	Servo module init status. Init must check the selected type of the servo and the timer configuration.
typedef enum
{
	SRV_INIT_OK 					= 0,
	SRV_INIT_FAIL_SRV_MODELL,
	SRV_INIT_FAIL_SRV_PWM,
} eBSP_SrvTimInitStat;

//! @brief	Informations and configuration according to the given servo motor
//!			datasheet or oscilloscope measurement.
//!
//!
//! @example	PWM timer config example:
//!
//! 				Prescaler = ( APB1_clk    / TIM_counter_clk ) - 1
//! 					 1343 = ( 84 MHz	  / 62500  		    ) - 1
//!
//! 	  	   			Period = ( TIM_counter_clk / f  ) - 1
//! 	    			  1249 = ( 62500		   / 50 ) - 1
typedef struct
{
	//! PWM properties
	uint16_t PWM_freq;			//! Analog: ~(50-60)Hz. Digital: ~(250-333)Hz.
	uint32_t PWM_cntr_freq;		//! TIM2: 32 bit
	uint16_t PWM_prescaler;		//! 16 bit
	uint16_t PWM_period;		//! 16 bit

	//! Rotational properties
	uint16_t Left_End;			//! element of [0deg;90deg] interval. IMPORTANT: Needed for compare value saturation!
	uint16_t Deg_30;
	uint16_t Deg_90;			//! 1,5 ms ~ 90 deg
	uint16_t Deg_150;
	uint16_t Right_End;			//! element of [90deg;180deg] interval. IMPORTANT: Needed for compare value saturation!

	int16_t CV_compensation;   	//! Compare value compensation (most be set after every settings).

	//! Characteristics: theta = compare value * m + b
	double Gradient;			//! m
	double Y_intercept;			//! b
} cBSP_SrvHandleTypeDef;

//! Servo control with PWM duty cycle. T = 50ms or 4ms
//!
//!     <-------------->
//!     ____        T    ____
//!    |    |           |         0 deg
//! ___|    |___________|
//!     <-->
//!      1ms
//!
//!     <-------------->
//!     ______      T    ____
//!    |      |         |         90 deg
//! ___|      |_________|
//!     <---->
//!      1.5ms
//!
//!     <-------------->
//!     ________    T    ____
//!    |        |       |        180 deg
//! ___|        |_______|
//!     <------>
//!        2ms

// Variables -----------------------------------------------------------------------------------------------------------

extern cBSP_SrvHandleTypeDef hsrv;

// Function prototypes -------------------------------------------------------------------------------------------------

//! @brief	Initializes the servo, checks the PWM and the servo configuration. On successful init the PWM can be used,
//! 		on unsuccessful init TIM clk is disabled so no harm can be done.
//! @retval	Init was successful or not: SRV_INIT_STATUS
const eBSP_SrvTimInitStat bspServoTimInit(void);

//! @brief	Disables the servo timer clock.
void bspServoTimDisable(void);

//! @brief	Enables the servo timer.
void bspServoTimEnable(void);

//! @brief	Sets the timer compare to a given value and changes the duty cycle of the PWM. Servo will rotate to
//! 		this position.
//! @param	_pos_ : The desired compare value.
void bspServoTimSetCompare(const uint32_t compare);

//! @brief	Gets the servo timer compare value.
//! @retval	The compare value.
const uint32_t bspServoTimGetCompare();

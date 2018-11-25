////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_measurements.h
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f0xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define	CONST_MOTOR_CURRENT			1
#define CONST_SERVO_CURRENT			1
#define CONST_SYSTEM_CURRENT		1
#define CONST_SEC_VOLTAGE			2.967201e-3+0.25
#define CONST_MAIN_VOLTAGE			2.967201e-3+0.25

// Typedefs ------------------------------------------------------------------------------------------------------------

// Variables -----------------------------------------------------------------------------------------------------------

int32_t MotorCurrentValue;
int32_t ServoCurrentValue;
int32_t SystemCurrentValue;
int32_t SecBatVoltageValue;
int32_t MainBatVoltageValue;

float MotorCurrent;
float ServoCurrent;
float SystemCurrent;
float SecBatVoltage;
float MainBatVoltage;

// Function prototypes -------------------------------------------------------------------------------------------------

void BSP_GetMeasurementValues	(
									int32_t* MotorCurrentValue,
									int32_t* ServoCurrentValue,
									int32_t* SystemCurrentValue,
									int32_t* SecBatVoltageValue,
									int32_t* MainBatVoltageValue
								);

void BSP_GetMeasurements	(
								float* MotorCurrent,
								float* ServoCurrent,
								float* SystemCurrent,
								float* SecBatVoltage,
								float* MainBatVoltage
							);

// END -----------------------------------------------------------------------------------------------------------------

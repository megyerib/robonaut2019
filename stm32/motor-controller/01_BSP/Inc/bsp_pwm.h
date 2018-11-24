////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_pwm.h
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

#include "stm32f0xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define PERIOD							500
#define DEADTIME 						8
#define DUTY_CYCLE_HALF_BRIDGE_MAX		0.95

// Typedefs ------------------------------------------------------------------------------------------------------------

// Variables -----------------------------------------------------------------------------------------------------------

float DutyCycleHalfBridge1;
float DutyCycleHalfBridge2;

int32_t CompareValueHigh2;		//Transistor 3	(because PCB)
int32_t CompareValueLow2;		//Transistor 4	(because PCB)
int32_t CompareValueHigh1;		//Transistor 1	(because PCB)
int32_t CompareValueLow1;

// Function prototypes -------------------------------------------------------------------------------------------------

void BSP_PWMStart();
void BSP_CreateDutyCycle(float DutyCycle, float* DutyCycleHalfBridge1, float* DutyCycleHalfBridge2);
void BSP_CreateCompareValue(float* DutyCycleHalfBridge, int32_t* CompareValueHigh, int32_t* CompareValueLow);
void BSP_BSP_PWMSetCompareRegisters	(
										int32_t* CompareValueHigh2,		//Transistor 3	(because PCB)
										int32_t* CompareValueLow2,		//Transistor 4	(because PCB)
										int32_t* CompareValueHigh1,		//Transistor 1	(because PCB)
										int32_t* CompareValueLow1		//Transistor 2	(because PCB)
									);

void BSP_SetDutyCycle(float* DutyCycle);

// END -----------------------------------------------------------------------------------------------------------------

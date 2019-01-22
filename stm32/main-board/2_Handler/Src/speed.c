////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      speed.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "speed.h"
#include "tim.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define MUL    (1.0f)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static uint16_t prev_cntrval = 0;
static uint16_t cur_cntrval = 0;
static uint16_t cntrDiff = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void speedInit()
{
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int16_t speedGet()
{
	float speed = cntrDiff * MUL;

	return speed;
}

void speedCallback()
{
	__disable_irq();
	prev_cntrval = cur_cntrval;
	cur_cntrval  = TIM3->CNT;

	cntrDiff = cur_cntrval - prev_cntrval;
	__enable_irq();
}

// Local (static) function definitions ---------------------------------------------------------------------------------

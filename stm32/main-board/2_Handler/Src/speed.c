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

#define MUL_DIST     (1.0f/140702.0f)
#define MUL_SPEED    (MUL_DIST * 1000.0f)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static uint32_t prev_cntrval = 0;
static uint32_t cur_cntrval = 0;
static uint32_t cntrDiff = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void speedInit()
{
	TIM5->CNT = 0; // Reset timer 5

	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

float speedGet()
{
	return cntrDiff * MUL_SPEED;
}

int32_t speedGetCounter()
{
	return cur_cntrval;
}

float speedGetDistance()
{
	return cur_cntrval * MUL_DIST;
}

void speedCallback()
{
	uint32_t tim_val;

	__disable_irq();
	tim_val  = TIM5->CNT;
	__enable_irq();

	prev_cntrval = cur_cntrval;
	cur_cntrval = tim_val;

	cntrDiff = cur_cntrval - prev_cntrval;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

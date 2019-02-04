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
#include "bsp_bluetooth.h"
#include "trace.h"

// Defines -------------------------------------------------------------------------------------------------------------

//#define INC_PER_M    (140702.0f) /* Balazs */
#define INC_PER_M    (140331.0f)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static uint32_t prev_cntrval = 0;
static uint32_t cur_cntrval = 0;
static  int32_t cntrDiff = 0;

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
	// (incr/ms)*1000 = incr/s
	// (incr/s)/(incr/m) = m/s

	float speed = cntrDiff * 1000.0 / INC_PER_M;

	return speed;
}

int32_t speedGetCounter()
{
	return TIM5->CNT;
}

float speedGetDistance()
{
	// incr/(incr/m) = m
	return TIM5->CNT / INC_PER_M;
}

void speedCallback()
{
	static int32_t ms_timer_val;
	static int32_t ms_timer_val_prev;

	uint32_t tim_val;
	int32_t meas_diff;
	int32_t diff;

	tim_val      = TIM5->CNT;
	ms_timer_val = TIM4->CNT;

	prev_cntrval = cur_cntrval;
	cur_cntrval = tim_val;

	// Compensation
	meas_diff = 1000 + ms_timer_val - ms_timer_val_prev;
	ms_timer_val_prev = ms_timer_val;

	diff = cur_cntrval - prev_cntrval;
	diff *= 1000;
	diff /= meas_diff;

	cntrDiff = diff;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

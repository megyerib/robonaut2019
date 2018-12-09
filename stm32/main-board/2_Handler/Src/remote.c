////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      remote.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "tim.h"

// Defines -------------------------------------------------------------------------------------------------------------

  #define PERIOD 16320 /* us */
  #define MIDDLE  1495 /* us */
//#define MIN      995 /* us */
//#define MAX     2080 /* us */

  #define MARGIN   200 /* us */

  #define HIGH_THR (MIDDLE + MARGIN)
  #define LOW_THR  (PERIOD - MIDDLE - MARGIN)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static int input_capture = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void remoteInit()
{
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
}

int remoteGetState()
{
	if (input_capture > HIGH_THR && input_capture < LOW_THR)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// Interrupt callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		input_capture = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
		__HAL_TIM_SetCounter(&htim1, 0);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END -----------------------------------------------------------------------------------------------------------------

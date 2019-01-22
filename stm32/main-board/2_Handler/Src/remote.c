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

  #define MIDDLE  2945 /* us */
  #define MAX     4060 /* us */

  #define MARGIN   250 /* us */

  #define ON_THR     (MIDDLE + MARGIN)
  #define UPPER_THR  (MAX + MARGIN)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

static int remote_state  = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void remoteInit()
{
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
}

int remoteGetState()
{
	return remote_state;
}

// Interrupt callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		int input_capture = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
		__HAL_TIM_SetCounter(&htim1, 0);


		/* // Some code for calibration (from debugger)
		static int j;

		j++;

		if (j == 5)
		{
			j = 0; // Set breakpoint here
		}*/

		if (input_capture < UPPER_THR && input_capture > ON_THR)
		{
			remote_state = 1;
		}
		else if (input_capture < ON_THR)
		{
			remote_state = 0;
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

// END -----------------------------------------------------------------------------------------------------------------

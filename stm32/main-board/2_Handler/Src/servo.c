////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		servo.c
//!  \brief
//!  \details	This module controls the rotation of the servo.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "servo.h"
#include "bsp_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define 	SERVO_ACTUAL_TYPE	 	SRV_SRT_CH6012   // Choose from SCH_ServoModel

// Typedefs ------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! @brief	Available types of servos.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
	SRV_FUTABAS3003 		= 0,
	SRV_SRT_CH6012
} eServoModel;

// Local (static) & extern variables -----------------------------------------------------------------------------------

extern cBSP_SrvHandleTypeDef hsrv;

// Local (static) function prototypes ----------------------------------------------------------------------------------


//! @brief	Determines if the SCH_ACTUAL_SERVO value is a valid SCH_ServoModel
//! 		enum variable and if it is, then it loads a valid servo config into
//! 		the hsrv module variable.
//! @retval	Signals if the initialization of the servo was successful because
//! 		the valid config was found and set
eBSP_SrvInitStat servoConfig();

// Global function definitions -----------------------------------------------------------------------------------------

eBSP_SrvInitStat servoInit(void)
{
	eBSP_SrvInitStat ret_val = SRV_INIT_OK;

	ret_val = servoConfig();

	if(ret_val == SRV_INIT_OK)
	{
		// Valid servo model is chosen.

		// Configure the TIM PWM, if error occurs TIM clk is disabled
		bspServoInit();
	}
	else
	{
		// Error: No valid servo motor was selected.

		// Disable CLK
		bspServoTimerDisable();
	}

	return ret_val;
}

double servoGetAngle()
{
	uint32_t compare;
	double angle;

	// Actual position.
	compare = bspServoGetCompare();

	return angle = compare * hsrv.Gradient + hsrv.Y_intercept;
}

void servoSetAngle(const double theta)
{
	uint32_t compare;
	double theta2 = theta + PI/2;

	// Calculate the position from the angle
	compare = (uint32_t)(((theta2 - hsrv.Y_intercept) / hsrv.Gradient) + hsrv.CV_compensation);

	bspServoSetCompare(compare);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

eBSP_SrvInitStat servoConfig()
{
	eBSP_SrvInitStat ret_val = SRV_INIT_OK;

	switch(SERVO_ACTUAL_TYPE)
	{
		case SRV_FUTABAS3003  :
			hsrv.PWM_freq = 50;						// 20 ms
			hsrv.PWM_cntr_freq = 62500;
			hsrv.PWM_prescaler = 1343;
			hsrv.PWM_period = 1249;

			hsrv.Right_End = 68;
			hsrv.Deg_30 = 74;
			hsrv.Deg_90 = 91;
			hsrv.Deg_150 = 113;
			hsrv.Left_End = 114;

			hsrv.CV_compensation = -1;

			// 3°/inc = PI/60 rad/inc
			hsrv.Gradient = PI/180 * (90-30)/(hsrv.Deg_90-hsrv.Deg_30);
			// -192° = -PI*16/15 rad
			hsrv.Y_intercept = PI/2 - hsrv.Deg_90 * hsrv.Gradient;
			break;

	   case SRV_SRT_CH6012:
		   // Calculated with known equations (bsp_servo.h)
		   hsrv.PWM_freq = 250;						// 4 ms
		   hsrv.PWM_cntr_freq = 62500;
		   hsrv.PWM_prescaler = 1343;
		   hsrv.PWM_period = 249;
		   // TODO Measure servo propeties.
		   hsrv.Left_End = 75;
		   hsrv.Deg_30 = 81;
		   hsrv.Deg_90 = 95;
		   hsrv.Deg_150 = 106;
		   hsrv.Right_End = 113;
		   // Characteristics
		   // m = pi/180 * (180 - 90)/(x_180 - x_90)
		   hsrv.Gradient = PI/180 * (90-30)/(hsrv.Deg_90-hsrv.Deg_30);
		   // y = m*x + b => b = y_90 - x_90 * m
		   hsrv.Y_intercept = PI/2 - hsrv.Deg_90 * hsrv.Gradient;
		   // Compensating the car installation error.
		   hsrv.CV_compensation = 0;

		   break;

		default :
			ret_val = SRV_INIT_FAIL_SRV_MODELL;
	}

	return ret_val;
}

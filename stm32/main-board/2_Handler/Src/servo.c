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
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

extern cBSP_SrvHandleTypeDef hsrv;

static eServoModel actualServo;

static uint8_t alreadyInited = 0;

// Local (static) function prototypes ----------------------------------------------------------------------------------

//! @brief	Determines if the SERVO_ACTUAL_TYPE value is a valid eServoModel enum variable and if it is, then it loads
//! 		a valid servo config into the hsrv module variable.
//! @retval	Signals if the initialization of the servo was successful because the valid config was found and set.
eBSP_SrvTimInitStat servoConfig();

// Global function definitions -----------------------------------------------------------------------------------------

eBSP_SrvTimInitStat servoInit(eServoModel myServoModel)
{
	eBSP_SrvTimInitStat ret_val = SRV_INIT_OK;

	if (alreadyInited == 0)
	{
		actualServo = myServoModel;

		ret_val = servoConfig();

		if(ret_val == SRV_INIT_OK)
		{
			// Valid servo model was chosen.

			// Configure the TIM PWM, if error occurs TIM clk is disabled.
			bspServoTimInit();
		}
		else
		{
			// Error: No valid servo motor was selected.

			// Disable CLK
			bspServoTimDisable();
		}

		alreadyInited = 1;
	}
	else
	{
		ret_val = SRV_INIT_FAIL_SRV_ALREADY_RUNNING;
	}

	return ret_val;
}

double servoGetAngle()
{
	uint32_t compare;
	double angle;

	// Actual position.
	compare = bspServoTimGetCompare();

	// angle = m*compare + b + compensation
	return angle = compare * hsrv.Gradient + hsrv.Y_intercept + hsrv.CV_compensation;
}

//! Servo Anlge:
//!
//!           120°  90°  60°
//!              \  |  /
//!               \ | /
//!     180°_______\|/________0°
//!   Left end               Right end
//!
void servoSetAngle(const double theta)
{
	uint32_t compare;
	//TODO double theta2 = theta + PI/2;

	// Calculate the position from the angle:  compare = (angle - b - compensation) / m
	compare = (uint32_t)( ((theta - hsrv.Y_intercept - hsrv.CV_compensation) / hsrv.Gradient) );

	bspServoTimSetCompare(compare);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

eBSP_SrvTimInitStat servoConfig()
{
	eBSP_SrvTimInitStat ret_val = SRV_INIT_OK;

	switch(actualServo)
	{
		case SRV_MAVERICK_MS22:
		{
			hsrv.PWM_freq 			= 50;							// [Hz] ~ (20ms)

			hsrv.PWM_cntr_freq		= 62500;						// [Hz]
			hsrv.PWM_prescaler 		= 1343;
			hsrv.PWM_period    		= 1249;

			hsrv.Left_End  			= 68;							// TODO
			hsrv.Deg_30    			= 74;							// TODO
			hsrv.Deg_90    			= 70;							// TODO
			hsrv.Deg_150   			= 113;							// TODO
			hsrv.Right_End 			= 112;							// TODO

			hsrv.Gradient = PI/180 * (90-30)/(hsrv.Deg_90-hsrv.Deg_30); 		// 3°/inc = PI/60 rad/inc
			hsrv.Y_intercept = PI/2 - hsrv.Deg_90 * hsrv.Gradient;				// -192° = -PI*16/15 rad

			hsrv.CV_compensation 	= -1;
			break;
	   }
	   case SRV_SRT_CH6012:
	   {
		   // Chosen frequency of the servo.
		   hsrv.PWM_freq 			= 250;						// [Hz] ~ (4 ms)

		   // Calculated with known equations (bsp_servo.h)
		   hsrv.PWM_cntr_freq 		= 62500;					// [Hz]
		   hsrv.PWM_prescaler 		= 1343;
		   hsrv.PWM_period 			= 249;

		   // Measured  servo properties.
		   hsrv.Left_End 			= 53;						// Defined by car 	(30° 0.846ms 53)
		   hsrv.Deg_30 				= 76;						// 1.214ms	60° 	(60° 1.214ms 76)
		   hsrv.Deg_90 				= 95;						// 1.52ms	90°		(90° 1.52ms  95)
		   hsrv.Deg_150 			= 114;						// 1.82ms	120°	(120° 1.82ms 114)
		   hsrv.Right_End 			= 136;						// Defined by car	(150° 2.17ms 136)

		   // Characteristics:  y = m*x + b
		   //
		   //      pi     y_150 - y_90
		   // m = ---- * --------------
		   //     180     x_150 - x_90
		   //
		   // b = y_90 - x_90 * m
		   //
		   hsrv.Gradient = PI/180 * (120 - 90) / (hsrv.Deg_150 - hsrv.Deg_90);	// [rad/compare increment]
		   hsrv.Y_intercept = PI/2 - hsrv.Deg_90 * hsrv.Gradient;				// [rad]

		   // Compensating the car installation error (offset).
		   hsrv.CV_compensation = 0;

		   break;
	   }
		case SRV_FUTABAS3003:
		{
/////////////////////////////////////////////// FUTBABA ////////////////////////////////////////////////////////////////
			hsrv.PWM_freq 			= 50;							// [Hz] ~ (20ms)
			hsrv.PWM_cntr_freq		= 50000;						// [Hz] from marLab homework
			hsrv.PWM_prescaler 		= 1679;							// from marLab homework
			hsrv.PWM_period    		= 999;							// from marLab homework

			hsrv.Left_End  			= 28;							// from marLab homework 0°
			hsrv.Deg_30    			= 42;							// measured
			hsrv.Deg_90    			= 70;							// from marLab homework
			hsrv.Deg_150   			= 97;							// measured
			hsrv.Right_End 			= 112;							// from marLab homework 180°

			hsrv.Gradient = PI/180 * (90-0)/(hsrv.Deg_90-hsrv.Left_End);
			hsrv.Y_intercept = PI/2 - hsrv.Deg_90 * hsrv.Gradient;

			hsrv.CV_compensation 	= 0;
/////////////////////////////////////////////// FUTBABA ////////////////////////////////////////////////////////////////
			break;
	   }
		default :
			ret_val = SRV_INIT_FAIL_SRV_MODELL;
	}

	return ret_val;
}

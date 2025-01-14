////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		servo.c
//!  \brief
//!  \details	This module controls the rotation of the servo.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_servo.h"
#include "bsp_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define 	SERVO_ACTUAL_TYPE	 	SRV_SRT_CH6012   // Choose from eServoModel

// Typedefs ------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//! @brief	Available types of servos.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
	SRV_FUTABAS3003 		= 0,		// Joci's servo.		Analog
	SRV_MAVERICK_MS22,					// Car' basic servo.	Analog
	SRV_SRT_CH6012						// From model shop.		Digital
} eServoModel;

// Local (static) & extern variables -----------------------------------------------------------------------------------

extern cBSP_SrvHandleTypeDef hsrv;

// Local (static) function prototypes ----------------------------------------------------------------------------------

//! @brief	Determines if the SERVO_ACTUAL_TYPE value is a valid eServoModel enum variable and if it is, then it loads
//! 		a valid servo config into the hsrv module variable.
//! @retval	Signals if the initialization of the servo was successful because the valid config was found and set.
eBSP_SrvTimInitStat servoConfig();

// Global function definitions -----------------------------------------------------------------------------------------

eBSP_SrvTimInitStat servoInit(void)
{
	eBSP_SrvTimInitStat ret_val = SRV_INIT_OK;

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

	return ret_val;
}

double servoGetAngle()
{
	uint32_t compare;
	double angle;

	// Actual position.
	compare = bspServoTimGetCompare();

//TODO
	// angle = m*compare + b + compensation
	return angle = compare * hsrv.Gradient + hsrv.Y_intercept;
}

//! Servo Anlge:
//!
//!           60�  90�  120�
//!              \  |  /
//!               \ | /
//!       0�_______\|/________180�
//!   Left end               Right end
//!
void servoSetAngle(const double theta)
{
	uint32_t compare;
	double theta2 = theta + PI/2;

	// Calculate the position from the angle	TODO compensation corrupts the max angle of the servo in one direction
	compare = (uint32_t)(((theta2 - hsrv.Y_intercept) / hsrv.Gradient) + hsrv.CV_compensation);

	bspServoTimSetCompare(compare);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

eBSP_SrvTimInitStat servoConfig()
{
	eBSP_SrvTimInitStat ret_val = SRV_INIT_OK;

	switch(SERVO_ACTUAL_TYPE)
	{
		case SRV_MAVERICK_MS22:
		{
			hsrv.PWM_freq 			= 50;							// [Hz] ~ (20ms)

			hsrv.PWM_cntr_freq		= 62500;						// [Hz]
			hsrv.PWM_prescaler 		= 1343;
			hsrv.PWM_period    		= 1249;

			hsrv.Left_End  			= 68;							// TODO
			hsrv.Deg_30    			= 74;							// TODO
			hsrv.Deg_90    			= 91;							// TODO
			hsrv.Deg_150   			= 113;							// TODO
			hsrv.Right_End 			= 114;							// TODO

			hsrv.Gradient = PI/180 * (90-30)/(hsrv.Deg_90-hsrv.Deg_30); 		// 3�/inc = PI/60 rad/inc
			hsrv.Y_intercept = PI/2 - hsrv.Deg_90 * hsrv.Gradient;				// -192� = -PI*16/15 rad

			hsrv.CV_compensation 	= -1;
			break;
	   }
	   case SRV_SRT_CH6012:
	   {
		   // Chosen frequency of the servo.
		   hsrv.PWM_freq 			= 250;						// [Hz] ~ (4 ms)

		   // Calculated with known equations (bsp_servo.h)
		   hsrv.PWM_cntr_freq 		= 62500;					// [Hz]
		   hsrv.PWM_prescaler 		= 1440 - 1;
		   hsrv.PWM_period 			=  250 - 1;

		   // Measured  servo properties.
		   hsrv.Left_End 			= 62;						// Defined by car 	(30� 0.846ms 53)
		   hsrv.Deg_30 				= 76;						// 1.214ms	60� 	(60� 1.214ms 76)
		   hsrv.Deg_90 				= 90;						// 1.52ms	90�		(90� 1.52ms  95)
		   hsrv.Deg_150 			= 114;						// 1.82ms	120�	(120� 1.82ms 114)
		   hsrv.Right_End 			= 118;						// Defined by car	(150� 2.17ms 136)

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
		   hsrv.CV_compensation = -4;

		   break;
	   }
		default :
			ret_val = SRV_INIT_FAIL_SRV_MODELL;
	}

	return ret_val;
}

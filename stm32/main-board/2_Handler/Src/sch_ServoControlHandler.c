/*
 * sch_ServoControlHandler.c
 *
 *  Created on: 2018. nov. 1.
 *      Author: Joci
 */

// ------------------------------- Includes -------------------------------- //

#include "../../2_Handler/Inc/sch_ServoControlHandler.h"

#include "../../3_BSP/Inc/bsp_common.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define 	SCH_ACTUAL_SERVO	 SRV_FUTABAS3003   // Choose from SCH_ServoModel

// --------------------------------------------------------------------------//

// --------------------------------- Enums ----------------------------------//

/*
 * @brief	Available types of servos.
 */
typedef enum SCH_ServoModel
{
	SRV_FUTABAS3003 		= 0
} SCH_ServoModel;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/*
 * @brief	Determines if the SCH_ACTUAL_SERVO value is a valid SCH_ServoModel
 * 			enum variable and if it is, then it loads a valid servo config into
 * 			the hsrv module variable.

 *
 * @retval	Signals if the initialization of the servo was successful because
 * 			the valid config was found and set
 */
const eBSP_SrvInitStat sch_Configure_Servo();

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

extern cBSP_SrvHandleTypeDef hsrv;

// --------------------------------------------------------------------------//

// ------------------------------ Functions ---------------------------------//

const eBSP_SrvInitStat sch_Servo_Init(void)
{
	eBSP_SrvInitStat ret_val = SRV_INIT_OK;

	ret_val = sch_Configure_Servo();

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

const double sch_Get_Servo_Angle()
{
	uint32_t compare;
	double angle;

	//Actual position
	compare = bspServoGetCompare();

	return angle = compare * hsrv.Gradient + hsrv.Y_intercept;
}

void sch_Set_Servo_Angle(const double theta)
{
	uint32_t compare;

	// Calculate the position from the angle
	compare = (uint32_t)((theta - hsrv.Y_intercept) / hsrv.Gradient);

	/* REDUNDANT
	// Make certain that the theta is in the valid interval.
	if(pos < hsrv.Right_End)
	{
		pos = hsrv.Right_End;
	}
	else if(pos > hsrv.Left_End)
	{
		pos = hsrv.Left_End;
	}
	*/

	bspServoSetCompare(compare);
}

const eBSP_SrvInitStat sch_Configure_Servo()
{
	eBSP_SrvInitStat ret_val = SRV_INIT_OK;

	switch(SCH_ACTUAL_SERVO)
	{
		case SRV_FUTABAS3003  :
			hsrv.PWM_freq = 50;
			hsrv.PWM_cntr_freq = 62500;
			hsrv.PWM_prescaler = 1343;
			hsrv.PWM_period = 1249;

			hsrv.Right_End = 70;
			hsrv.Deg_30 = 74;
			hsrv.Deg_90 = 94;
			hsrv.Deg_150 = 113;
			hsrv.Left_End = 115;

			// 3°/inc = PI/60 rad/inc
			hsrv.Gradient = PI/180 * (90-30)/(hsrv.Deg_90-hsrv.Deg_30);
			// -192° = -PI*16/15 rad
			hsrv.Y_intercept = PI/2 - hsrv.Deg_90 * hsrv.Gradient;
			break;

	  // case constant-expression  :
	  //   statement(s);
	  //    break; /* optional */

		default :
			ret_val = SRV_INIT_FAIL_SRV_MODELL;
	}

	return ret_val;
}

// --------------------------------------------------------------------------//

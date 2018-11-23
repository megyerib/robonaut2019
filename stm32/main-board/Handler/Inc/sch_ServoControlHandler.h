/*
 * sch_ServoControlHandler.h
 *
 *  Created on: 2018. nov. 1.
 *      Author: Joci
 */

#ifndef HANDLER_SCH_SERVOCONTROLHANDLER_H_
#define HANDLER_SCH_SERVOCONTROLHANDLER_H_

// ------------------------------- Includes -------------------------------- //

#include <bsp_servo.h>

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/*
 * @brief	Initializes the module and the servo.
 *
 * @retval	Result of the initialization (OK, FAIL).
 */
const eBSP_SrvInitStat sch_Servo_Init(void);

/*
 * @brief	Gets the servo position in radian.
 *
 * @retval	Servo angle.
 */
const double sch_Get_Servo_Angle(void);

/*
 * @brief	Rotates the servo to a given angle.
 *
 * @param	_theta_ : The desired servo angle.
 */
void sch_Set_Servo_Angle(const double theta);

// --------------------------------------------------------------------------//

#endif /* HANDLER_SCH_SERVOCONTROLHANDLER_H_ */


/*
 * sch_ServoControlHandler.h
 *
 *  Created on: 2018. nov. 1.
 *      Author: Joci
 */

#ifndef HANDLER_SCH_SERVOCONTROLHANDLER_H_
#define HANDLER_SCH_SERVOCONTROLHANDLER_H_

// ------------------------------- Includes -------------------------------- //

#include "BSP/bsp_servo.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/*
 *
 */
const BSP_SrvInitStat sch_Servo_Init(void);

/*
 *
 */
const double sch_Get_Servo_Angle(void);

/*
 *
 */
void sch_Set_Servo_Angle(const double theta);

// --------------------------------------------------------------------------//

#endif /* HANDLER_SCH_SERVOCONTROLHANDLER_H_ */


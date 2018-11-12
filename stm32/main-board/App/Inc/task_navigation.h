/*
 * task_navigation.h
 *
 *  Created on: 2018. nov. 11.
 *      Author: Joci
 */

#ifndef TASK_TASK_NAVIGATION_H_
#define TASK_TASK_NAVIGATION_H_

// ------------------------------- Includes -------------------------------- //

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

/*
 * @brief	Initializes the Task_Servo task.
 */
void TaskInit_Navigation(void);

/*
 * @brief	Communicates with the Task_Sharp task and moves the  servo.
 */
void Task_Navigation(void* p);

// --------------------------------------------------------------------------//

#endif /* TASK_TASK_NAVIGATION_H_ */


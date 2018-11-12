/*
 * app_common.h
 *
 *  Created on: 2018. nov. 12.
 *      Author: Joci
 */

#ifndef INC_APP_COMMON_H_
#define INC_APP_COMMON_H_

// ------------------------------- Includes -------------------------------- //

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

#define 	DEFAULT_STACK_SIZE 				128
#define 	TASK_SHARP_PRIO					tskIDLE_PRIORITY+3
#define 	TASK_SRV_PRIO					tskIDLE_PRIORITY+2
#define 	TASK_NAVI_PRIO					tskIDLE_PRIORITY+1

#define		TASK_DELAY_40_MS				40		// Depends on the Tick
#define		TASK_DELAY_20_MS				20
#define		TASK_DELAY_16_MS				16

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

// --------------------------------------------------------------------------//

#endif /* INC_APP_COMMON_H_ */


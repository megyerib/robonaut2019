/*
 * drn_DeadReckoningNavigation.h
 *
 *  Created on: 2018. nov. 9.
 *      Author: Joci
 */

#ifndef HANDLER_DRN_DEADRECKONINGNAVIGATION_H_
#define HANDLER_DRN_DEADRECKONINGNAVIGATION_H_

// ------------------------------- Includes -------------------------------- //

#include "FreeRTOS.h"
#include "semphr.h"

// --------------------------------------------------------------------------//

// -------------------------------- Defines ---------------------------------//

typedef struct
{
	double x;
	double y;
} VelocityVector;

typedef struct
{
	double omega;
	double time;
} AngularVelocity;

typedef struct
{
	double n;
	double e;
} NED_Parameters;

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

SemaphoreHandle_t semDrNavi;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

void drn_Init (void);

const NED_Parameters drn_GetNedCoordinates (void);

void drn_SetNedCoordinates (const NED_Parameters coords);

//												[m/s]							[rad/s]					[ms]
const NED_Parameters drn_ReckonNavigation (const VelocityVector v, const AngularVelocity w, const uint32_t dt);

const double drn_NumInteg_Trapezoidal (const double a, const double b, const double fa, const double fb);


// --------------------------------------------------------------------------//

#endif /* HANDLER_DRN_DEADRECKONINGNAVIGATION_H_ */


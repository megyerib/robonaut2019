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
} cVelocityVector;

//TODO _Joci_ needs update
typedef struct
{
	double omega;
	double time;
} cAngularVelocity;

typedef struct
{
	double n;
	double e;
} cNedParameters;

// --------------------------------------------------------------------------//

// ------------------------------- Variables --------------------------------//

SemaphoreHandle_t semDrNavi;

// --------------------------------------------------------------------------//

// ------------------------------ Declarations ------------------------------//

void drnInit (void);

cNedParameters drnGetNedCoordinates (void);

void drnSetNedCoordinates (const cNedParameters coords);

//												[m/s]							[rad/s]					[ms]
cNedParameters drnReckonNavigation (const cVelocityVector v, const cAngularVelocity w, const uint32_t dt);

double drnNumIntegTrapezoidal (const double a, const double b, const double fa, const double fb);


// --------------------------------------------------------------------------//

#endif /* HANDLER_DRN_DEADRECKONINGNAVIGATION_H_ */


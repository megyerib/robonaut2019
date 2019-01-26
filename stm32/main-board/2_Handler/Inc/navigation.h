////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file		navigation.h
//!  \brief
//!  \details  	This module is responsible for the navigation of the car. This module implements the simple Dead
//! 			Reckoning algorithm.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "semphr.h"
#include "inert.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	eNAVI_INERT         = 0,
	eNAVI_INERT_GRAVY_CORR,
} eNAVI_INERT_MODE;

typedef enum
{
	eNAVI_ENC            = 0,
	eNAVI_ENC_GRAVY_CORR
} eNAVI_ENC_MODE;

typedef struct
{
	float u;
	float v;
} cVEC_ACCEL;

typedef struct
{
	float u;
	float v;
} cVEC_VEL;

typedef struct
{
	float n;
	float e;
} cNED_COORD;

typedef struct
{
	cNED_COORD p;
	float phi;
} cNAVI_STATE;

// Variables -----------------------------------------------------------------------------------------------------------

SemaphoreHandle_t semDrNavi;

// Function prototypes -------------------------------------------------------------------------------------------------

//*********************************************************************************************************************
//! Initializes the navigation module.
//!
//! Resets all of the module variables.
//!
//! @return -
//*********************************************************************************************************************
void naviDRInit (void);

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
cNAVI_STATE naviGetNaviDataEnc (const float v, const ANGVEL w, const uint32_t dt, const eNAVI_ENC_MODE mode);

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
cNAVI_STATE naviGetNaviDataInrt (const cVEC_ACCEL a, const ANGVEL w, const uint32_t dt, const eNAVI_INERT_MODE mode);

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
float naviConvertDpsToSI (const float ang_dps);

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
float naviConvertGToSI (const float accel_g);

/* TODO Do we need this?
cNED_COORD naviDRGetNedCoordinates (void);

void naviDRSetNedCoordinates (const cNED_COORD coords);
*/

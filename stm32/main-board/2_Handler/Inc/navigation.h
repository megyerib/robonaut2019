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
	double u;
	double v;
} cVEC_ACCEL;

typedef struct
{
	double u;
	double v;
} cVEC_VEL;

typedef struct
{
	double n;
	double e;
} cNED_COORD;

typedef struct
{
	cNED_COORD p;
	double psi;
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
cNAVI_STATE naviGetNaviDataEnc (const double v, const ANGVELd w, const uint32_t dt, const eNAVI_ENC_MODE mode);

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
cNAVI_STATE naviGetNaviDataInrt (const cVEC_ACCEL a, const ANGVELd w, const uint32_t dt, const eNAVI_INERT_MODE mode);

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
double naviConvertDpsToSI (const double ang_dps);

//**********************************************************************************************************************
//!
//!
//**********************************************************************************************************************
double naviConvertGToSI (const double accel_g);

double naviNormaliseOrientation (const double psi);

/* TODO Do we need this?
cNED_COORD naviDRGetNedCoordinates (void);

void naviDRSetNedCoordinates (const cNED_COORD coords);
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      controller.h
//!  \brief    
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include <stdint.h>

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------

//! @brief	Discrete transfer function first order in normal form.
//!
//!               B[z]    b0 + b1*z^-1
//!       H[z]  = ---- = --------------
//!               A[z]    1  + a1*z^-1
//!
typedef struct
{
	double   b0;
	double   b1;
	//double   a0;		//!< Equals with 1 in normal form
	double   a1;
	uint32_t ts;		//!< Sampling time

	double	 bn_past;	//!< b n-1 previous value
	double   an_past;	//!< a n-1 previous value
} cFirstOrderTF;

// Variables -----------------------------------------------------------------------------------------------------------
// Function prototypes -------------------------------------------------------------------------------------------------

double controllerTransferFunction (cFirstOrderTF* tf, double an);

void controllerPdConfigure (cFirstOrderTF* const tf, const double Kp, const double Td, const double T);

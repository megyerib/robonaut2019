////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      scm_SpeedControlModule.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "hndlCommon.h"
#include "scm_SpeedControlModule.h"

// Defines -------------------------------------------------------------------------------------------------------------

//TODO DEBUG: Demo process
#define 	SCM_PROCESS_K			1
#define 	SCM_PROCESS_B0		    0
#define     SCM_PROCESS_B1		    0.008
#define     SCM_PROCESS_A0		    1
#define     SCM_PROCESS_A1		    -0.99
#define 	SCM_PROCESS_ST			0.016
// END_DEBUG

#define 	SCM_PI_K			    5
#define 	SCM_PI_B0			    1
#define     SCM_PI_B1			    -0.99
#define     SCM_PI_A0			    1
#define     SCM_PI_A1			    -1
#define 	SCM_PI_ST				0.016

#define     SCM_TF_ORDER		   1

// Typedefs ------------------------------------------------------------------------------------------------------------

//! @brief	Discrete transfer function first order in normal form.
typedef struct
{
	double   b0;
	double   b1;
	double   a0;		//!< Equals with 1 in normal form
	double   a1;
	uint32_t ts;		//!< Sampling time

	double	 bn_past;	//!< b n-1 previous value
	double   an_past;	//!< a n-1 previous value
} cFirstOrderTF;

// Local (static) & extern variables -----------------------------------------------------------------------------------

cFirstOrderTF piController;
cFirstOrderTF process;

double e_prev[SCM_TF_ORDER+1];
double u_prev[SCM_TF_ORDER+1];
double y_prev[SCM_TF_ORDER+1];

// Local (static) function prototypes ----------------------------------------------------------------------------------

static double scmTransferFunction (cFirstOrderTF* tf, double an);

// Global function definitions -----------------------------------------------------------------------------------------

void scmInitControllerPI (void)
{
	process.b0      = SCM_PROCESS_B0 * SCM_PROCESS_K;
	process.b1      = SCM_PROCESS_B1 * SCM_PROCESS_K;
	process.a0      = SCM_PROCESS_A0;
	process.a1      = SCM_PROCESS_A1;
	process.bn_past = 0;
	process.an_past = 0;

	piController.b0      = SCM_PI_B0 * SCM_PI_K;
	piController.b1      = SCM_PI_B1 * SCM_PI_K;
	piController.a0      = SCM_PI_A0;
	piController.a1      = SCM_PI_A1;
	piController.bn_past = 0;
	piController.an_past = 0;

    memset(e_prev, 0, sizeof(e_prev));
    memset(u_prev, 0, sizeof(u_prev));
    memset(y_prev, 0, sizeof(y_prev));
}

double scmControlLoop (double rn)
{
	double en;
	double un;
    double yn;

	en = rn - y_prev[0];
	un = scmTransferFunction(&piController, e_prev[0]);
	yn = scmTransferFunction(&process, 	    u_prev[0]);

	e_prev[0] = en;
	u_prev[0] = un;
	y_prev[0] = yn;

	return yn;
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//! @param tf
//! @param an
//! @return
//!
//! @example
static double scmTransferFunction (cFirstOrderTF* tf, double an)
{
	double bn;

	bn = tf->b0 * an + tf->b1 * tf->an_past - tf->a1 * tf->bn_past;

	tf->bn_past = bn;
	tf->an_past = an;

	return bn;
}

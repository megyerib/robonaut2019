////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      mux.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "../../2_BSP/Inc/mux.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define MUX_ENABLED  0
#define MUX_DISABLED 1

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

void initMux()
{
    // Enabling MUX
    HAL_GPIO_WritePin(MUX_E_GPIO_Port, MUX_E_Pin, MUX_ENABLED);
}

// Propagation delay < 60 ns
// T_clk ~ 20 ns; a function call lasts longer than that
void setMux(uint8_t input)
{
    HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, (input >> 2) & 1);
    HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, (input >> 1) & 1);
    HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, (input >> 0) & 1);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

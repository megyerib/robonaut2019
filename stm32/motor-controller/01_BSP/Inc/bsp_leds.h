////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_leds.h
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
// Includes ------------------------------------------------------------------------------------------------------------

// Defines -------------------------------------------------------------------------------------------------------------

#define PERIOD_LED_HEARTBEAT	2400

// Typedefs ------------------------------------------------------------------------------------------------------------

// Variables -----------------------------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------------------

void BSP_SetLEDBurstOFF5V(void);
void BSP_SetLEDBurtsOFF6V(void);
void BSP_SetLEDHeartbeat(void);
void BSP_SetLEDHeartbeatBlinking(void);
void BSP_SetLEDHeartbeatBlinkingDutyCyle(float* DutyCyle);
void BSP_SetLEDOrange(void);
void BSP_SetLEDFault(void);

//void BSP_ResetLEDBurstOFF5V(void);	This LEDs must always set on.
//void BSP_ResetEDBurtsOFF6V(void);
void BSP_ResetLEDHeartbeat(void);
void BSP_ResetLEDOrange(void);
void BSP_ResetLEDFault(void);

void BSP_LEDStart(void);

// END -----------------------------------------------------------------------------------------------------------------

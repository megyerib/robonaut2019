////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bsp_i2c.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "bsp_i2c.h"

#include "i2c.h"
#include "stm32f4xx_hal.h"

// Defines -------------------------------------------------------------------------------------------------------------

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

// Global function definitions -----------------------------------------------------------------------------------------

// Local (static) function definitions ---------------------------------------------------------------------------------

void i2cInit()
{
	//HAL_I2C_MspInit(&BSP_HI2C_DISTANCE);

	//HAL_I2C_Master_Transmit(&HI2C_DISTANCE);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == INERTIAL_I2C)
	{
		i2cInertialSensorMemRxCallback();
	}
	/*else if (hi2c == BSP_HI2C_DISTANCE)
	{
		i2cDistanceMemRxCallbacl();
	}*/
}

__weak void i2cInertialSensorMemRxCallback()
{
	// Weak function defined in the inertial sensor handler.
}

__weak void i2cDistanceMemRxCallbacl()
{
	// Weak function defined in the time of flight sensor handler.
}

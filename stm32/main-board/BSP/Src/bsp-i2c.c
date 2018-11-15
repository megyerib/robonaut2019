#include "bsp-i2c.h"
#include "stm32f4xx_hal.h"

#define INERTIAL_I2C (&hi2c1) /* Sole device on this bus */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == INERTIAL_I2C)
	{
		i2cInertialSensorMemRxCallback();
	}
}

__weak void i2cInertialSensorMemRxCallback()
{
	// Weak function defined in the inertial sensor handler.
}


#include <mux.h>

#define MUX_ENABLED  0
#define MUX_DISABLED 1

void InitMux()
{
	HAL_GPIO_WritePin(MUX_E_GPIO_Port, MUX_E_Pin, MUX_DISABLED); // Low active

	HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, GPIO_PIN_RESET);
}

void SetMux(uint8_t input, uint8_t en)
{
	HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, (input >> 2) & 1);
	HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, (input >> 1) & 1);
	HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, (input >> 0) & 1);

	HAL_GPIO_WritePin(MUX_E_GPIO_Port, MUX_E_Pin, (en ? MUX_ENABLED : MUX_DISABLED));
}

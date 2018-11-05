#include <mux.h>

#define MUX_ENABLED  0
#define MUX_DISABLED 1

// Multiplexer enged�lyez�se
void initMux()
{
	// Nem l�tom �rtelm�t letiltani
	HAL_GPIO_WritePin(MUX_E_GPIO_Port, MUX_E_Pin, MUX_ENABLED);
}

// Propagation delay < 60 ns
// Kb. 20 ns egy �rajel-ciklus, semmi �rtelme k�sleltetni.
void setMux(uint8_t input)
{
	HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, (input >> 2) & 1);
	HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, (input >> 1) & 1);
	HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, (input >> 0) & 1);
}

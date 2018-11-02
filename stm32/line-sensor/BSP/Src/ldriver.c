#include "ldriver.h"
#include "spi.h"

#define OE_ENABLED  0
#define OE_DISABLED 1
#define LE_ENABLED  1
#define LE_DISABLED 0

void InitLDriver()
{
	HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, OE_DISABLED);
	HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, LE_DISABLED);

	HAL_GPIO_WritePin(IR_OE_GPIO_Port,  IR_OE_Pin,  OE_DISABLED);
	HAL_GPIO_WritePin(IR_LE_GPIO_Port,  IR_LE_Pin,  LE_DISABLED);
}

void EnableIr()
{
	HAL_GPIO_WritePin(IR_OE_GPIO_Port,  IR_OE_Pin,  OE_ENABLED);
}

void DisableIr()
{
	HAL_GPIO_WritePin(IR_OE_GPIO_Port,  IR_OE_Pin,  OE_DISABLED);
}

void EnableLed()
{
	HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, OE_ENABLED);
}

void DisableLed()
{
	HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, OE_DISABLED);
}

void WriteLed(uint32_t ledval)
{
	// Csere az elhelyezkedés miatt
	uint16_t tmp;
	uint16_t* buf = (uint16_t*) &ledval;

	tmp = buf[0];
	buf[0] = buf[1];
	buf[1] = tmp;

	/* Ez a proci nem támogatja :(
	asm("rbit %1,%0" : "=r" (ledval) : "r" (ledval));
	asm("rev %1,%0" : "=r" (ledval) : "r" (ledval));*/

	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	HAL_SPI_Init(&hspi1);

	HAL_SPI_Transmit(&hspi1, (uint8_t*) &ledval, 4, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &ledval, 4, HAL_MAX_DELAY); // Dummy

	hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
	HAL_SPI_Init(&hspi1);

	HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, LE_ENABLED);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, LE_DISABLED);
}

void WriteIr(uint32_t irval)
{
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &irval, 4, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(IR_LE_GPIO_Port, IR_LE_Pin, LE_ENABLED);
	HAL_Delay(1);
	HAL_GPIO_WritePin(IR_LE_GPIO_Port, IR_LE_Pin, LE_DISABLED);
}

void WriteLedIr(uint32_t ledval, uint32_t irval)
{
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &ledval, 4, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &irval,  4, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, LE_ENABLED);
	HAL_GPIO_WritePin(IR_LE_GPIO_Port, IR_LE_Pin, LE_ENABLED);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, LE_DISABLED);
	HAL_GPIO_WritePin(IR_LE_GPIO_Port, IR_LE_Pin, LE_DISABLED);
}

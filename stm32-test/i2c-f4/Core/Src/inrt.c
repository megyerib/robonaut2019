#include "inrt.h"
#include "LSM6DS3.h"
#include "i2c.h"
#include <string.h>

#define ACCCEL_CONST 1671 // Read value -> m/s^2

static void Write8bitReg(uint8_t addr, uint8_t data);
static void Write16bitReg(uint8_t addr, uint16_t data);
static void Read8bitReg(uint8_t addr, void* dst);
static void Read16bitReg(uint8_t addr, void* dst);

// Global ----------------------------------------------------------------------

void InitInertia()
{
	HAL_GPIO_WritePin(INRT_SD0_GPIO_Port, INRT_SD0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_RESET); // CS: high-active

	Write8bitReg(CTRL9_XL, 0x38); // Acc X, Y, Z axes enabled
	Write8bitReg(CTRL1_XL, 0x60); // Acc = 416Hz (High-Performance mode)
}

Inertia GetInertia()
{
	Inertia ret;
	int16_t raw[3];
	volatile int32_t full[3];
	int i;

	memset(&ret, 0, sizeof(Inertia)); // Stackre kerül a változó, mindig valami hülyeség lesz benne. Csak debug miatt.

	Read16bitReg(OUTX_L_XL, &raw[0]);
	Read16bitReg(OUTY_L_XL, &raw[1]);
	Read16bitReg(OUTZ_L_XL, &raw[2]);

	for (i = 0; i < 3; i++)
		full[i] = raw[i] * 1000 / ACCCEL_CONST;

	// Transzformáció az autó inerciarendszerébe
	ret.a_x = (int16_t) full[1];
	ret.a_y = (int16_t) full[0] * -1;
	ret.a_z = (int16_t) full[2] * -1;

	return ret;
}

// Static ----------------------------------------------------------------------

static void Write8bitReg(uint8_t addr, uint8_t data)
{
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_SET);
	HAL_I2C_Mem_Write(&hi2c1, LSM6DS3_ADDR0, addr, 1, &data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_RESET);
}

static void Write16bitReg(uint8_t addr, uint16_t data)
{
	Write8bitReg(addr,     *((uint8_t*)(&data))    );
	Write8bitReg(addr + 1, *((uint8_t*)(&data) + 1));
}

static void Read8bitReg(uint8_t addr, void* dst)
{
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_SET);
	HAL_I2C_Mem_Read(&hi2c1, LSM6DS3_ADDR0, addr, 1, (uint8_t*) dst, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(INRT_CS_GPIO_Port, INRT_CS_Pin, GPIO_PIN_RESET);
}

static void Read16bitReg(uint8_t addr, void* dst)
{
	Read8bitReg(addr,     (uint8_t*) dst    );
	Read8bitReg(addr + 1, (uint8_t*) dst + 1);
}

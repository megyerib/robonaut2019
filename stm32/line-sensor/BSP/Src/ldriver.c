////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      ${file_name}
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "ldriver.h"
#include "spi.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define OUTPUT_ENABLED  0
#define OUTPUT_DISABLED 1
#define LATCH_ENABLED   1
#define LATCH_DISABLED  0

#define IR_ENABLE()   HAL_GPIO_WritePin(IR_OE_GPIO_Port,  IR_OE_Pin,  OUTPUT_ENABLED)
#define IR_DISABLE()  HAL_GPIO_WritePin(IR_OE_GPIO_Port,  IR_OE_Pin,  OUTPUT_DISABLED)
#define LED_ENABLE()  HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, OUTPUT_ENABLED)
#define LED_DISABLE() HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, OUTPUT_DISABLED)

#define IR_LATCH_ENABLE()   HAL_GPIO_WritePin(IR_LE_GPIO_Port,  IR_LE_Pin,  LATCH_ENABLED)
#define IR_LATCH_DISABLE()  HAL_GPIO_WritePin(IR_LE_GPIO_Port,  IR_LE_Pin,  LATCH_DISABLED)
#define LED_LATCH_ENABLE()  HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, LATCH_ENABLED)
#define LED_LATCH_DISABLE() HAL_GPIO_WritePin(LED_LE_GPIO_Port, LED_LE_Pin, LATCH_DISABLED)

// Typedefs ------------------------------------------------------------------------------------------------------------

// Local (static) & extern variables -----------------------------------------------------------------------------------

// Local (static) function prototypes ----------------------------------------------------------------------------------

static uint32_t transformLedReg(uint32_t ledval);
static uint32_t transformIrReg(uint32_t irval);

// Global function definitions -----------------------------------------------------------------------------------------

void initLDriver()
{
    IR_DISABLE();
    LED_DISABLE();

    // A biztonság kedvéért, ha a az egyikbe írunk, ne latchelõdjön a másik.
    IR_LATCH_DISABLE();
    LED_LATCH_DISABLE();
}

void enableIr()
{
    IR_ENABLE();
}

void DisableIr()
{
    IR_DISABLE();
}

void enableLed()
{
    LED_ENABLE();
}

void disableLed()
{
    LED_DISABLE();
}

void writeLed(uint32_t ledval)
{
    ledval = transformLedReg(ledval);

    // Ha átírnám nem blokkolóra, állandó helyrõl kellene küldeni!
    HAL_SPI_Transmit(&hspi1, (uint8_t*) &ledval, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*) &ledval, 4, HAL_MAX_DELAY); // Dummy

    // 6 ns kell neki, több, mint 20-at kap.
    LED_LATCH_ENABLE();
    LED_LATCH_DISABLE();
}

void writeIr(uint32_t irval)
{
    irval = transformIrReg(irval);

    // Ha átírnám nem blokkolóra, állandó helyrõl kellene küldeni!
    HAL_SPI_Transmit(&hspi1, (uint8_t*) &irval, 4, HAL_MAX_DELAY);

    // 6 ns kell neki, több, mint 20-at kap.
    IR_LATCH_ENABLE();
    IR_LATCH_DISABLE();
}

void writeLedIr(uint32_t ledval, uint32_t irval)
{
    ledval = transformLedReg(ledval);
    irval = transformIrReg(irval);

    // Ha átírnám nem blokkolóra, állandó helyrõl kellene küldeni!
    HAL_SPI_Transmit(&hspi1, (uint8_t*) &ledval, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*) &irval,  4, HAL_MAX_DELAY);

    LED_LATCH_ENABLE();
    LED_LATCH_DISABLE();

    IR_LATCH_ENABLE();
    IR_LATCH_DISABLE();
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static uint32_t transformIrReg(uint32_t irval)
{
    uint16_t tmp;
    uint16_t* words = (uint16_t*) &irval;

    // Szósorrend-váltás
    tmp = words[0];
    words[0] = words[1];
    words[1] = tmp;

    return irval;
}

static uint32_t transformLedReg(uint32_t ledval)
{
    // Igény szerint kirakható, ha kell máshol is.
    const uint8_t fbo_lut[] =
    {0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
     0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
     0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
     0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
     0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
     0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
     0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
     0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
     0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
     0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
     0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
     0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
     0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
     0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
     0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
     0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF};

    int i;
    uint16_t tmp;
    uint16_t* words = (uint16_t*) &ledval;
    uint8_t* bytes  = (uint8_t*)  &ledval;

    // Bitsorrend-váltás
    for (i = 0; i < 4; i++)
        bytes[i] = fbo_lut[bytes[i]];

    /* Ez a proci nem támogatja :(
    asm("rbit %1,%0" : "=r" (ledval) : "r" (ledval));
    asm("rev %1,%0" : "=r" (ledval) : "r" (ledval));*/

    // Szósorrend-váltás
    tmp = words[0];
    words[0] = words[1];
    words[1] = tmp;

    return ledval;
}

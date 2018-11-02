#include "stm32f0xx_hal.h"

void InitLDriver();

void EnableIr();
void DisableIr();
void EnableLed();
void DisableLed();

void WriteLed(uint32_t ledval);
void WriteIr(uint32_t irval);
void WriteLedIr(uint32_t ledval, uint32_t irval);

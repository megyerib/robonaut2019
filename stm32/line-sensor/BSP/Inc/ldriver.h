#pragma once
#include "stm32f0xx_hal.h"

void initLDriver();

void enableIr();
void disableIr();
void enableLed();
void disableLed();

void writeLed(uint32_t ledval);
void writeIr(uint32_t irval);
void writeLedIr(uint32_t ledval, uint32_t irval);

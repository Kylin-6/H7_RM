#pragma once
#include <stdint.h>

struct ADC_HandleTypeDef
{
    void *Instance;
};

extern void *const ADC1;
extern void *const ADC2;
extern void *const ADC3;

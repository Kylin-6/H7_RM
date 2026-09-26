#pragma once
#include "adc.h"

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum { ADC_CALIB_OFFSET, ADC_SINGLE_ENDED };

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *, int, int);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *, uint32_t *, uint32_t);

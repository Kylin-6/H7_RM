#include "bsp_adc.h"
#include <stdio.h>

static int adc_instances[3];
void *const ADC1 = &adc_instances[0];
void *const ADC2 = &adc_instances[1];
void *const ADC3 = &adc_instances[2];

static HAL_StatusTypeDef calibration_result = HAL_OK;
static HAL_StatusTypeDef dma_result = HAL_OK;
static unsigned calibration_calls, dma_calls;
static uint32_t dma_length;
static uint32_t *dma_buffer;

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *, int, int)
{
    ++calibration_calls;
    return calibration_result;
}

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *, uint32_t *buffer, uint32_t length)
{
    ++dma_calls;
    dma_buffer = buffer;
    dma_length = length;
    return dma_result;
}

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); return 1; } } while (0)

int main()
{
    ADC_HandleTypeDef adc{ADC1};
    CHECK(!ADC_Init(nullptr, 1));
    CHECK(!ADC_Init(&adc, 0));
    CHECK(!ADC_Init(&adc, ADC_BUFFER_SIZE + 1));
    CHECK(calibration_calls == 0 && dma_calls == 0);

    calibration_result = HAL_ERROR;
    CHECK(!ADC_Init(&adc, 1));
    CHECK(calibration_calls == 1 && dma_calls == 0);

    calibration_result = HAL_OK;
    dma_result = HAL_ERROR;
    CHECK(!ADC_Init(&adc, 1));
    CHECK(calibration_calls == 2 && dma_calls == 1);

    dma_result = HAL_OK;
    CHECK(ADC_Init(&adc, ADC_BUFFER_SIZE));
    CHECK(dma_length == ADC_BUFFER_SIZE);
    CHECK(dma_buffer == (uint32_t *) &ADC1_Manage_Object.ADC_Data);

    adc.Instance = ADC2;
    CHECK(ADC_Init(&adc, 1));
    CHECK(dma_buffer == (uint32_t *) &ADC2_Manage_Object.ADC_Data);

    adc.Instance = ADC3;
    CHECK(ADC_Init(&adc, 1));
    CHECK(dma_buffer == (uint32_t *) &ADC3_Manage_Object.ADC_Data);

    adc.Instance = &calibration_calls;
    CHECK(!ADC_Init(&adc, 1));
    CHECK(dma_calls == 4);
    puts("PASS ADC calibration, DMA status, and buffer bounds");
    return 0;
}

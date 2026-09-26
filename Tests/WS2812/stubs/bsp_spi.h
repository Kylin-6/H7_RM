#pragma once
#include <stdint.h>

struct SPI_HandleTypeDef {};
struct GPIO_TypeDef {};
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
enum { HAL_OK = 0, HAL_ERROR = 1 };

struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
};

extern Struct_SPI_Manage_Object SPI6_Manage_Object;
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t,
                          GPIO_PinState, const uint8_t *, uint16_t);

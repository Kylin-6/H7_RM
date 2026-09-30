#pragma once
#include <stdint.h>
#include "stm32h7xx.h"
#define UART_WORDLENGTH_9B 9U
#define UART_STOPBITS_2 2U
#define UART_PARITY_EVEN 2U
#define UART_MODE_RX 1U
typedef struct { unsigned unused; } DMA_HandleTypeDef;
typedef struct {
    struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode; } Init;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;
extern UART_HandleTypeDef huart5;
#ifdef __cplusplus
extern "C" {
#endif
uint32_t HAL_GetTick(void);
#ifdef __cplusplus
}
#endif

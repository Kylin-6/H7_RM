#pragma once
#include "usart.h"
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef void (*UART_Callback)(uint8_t *, uint16_t);
#ifdef __cplusplus
extern "C" {
#endif
void UART_Init(UART_HandleTypeDef *, UART_Callback);
HAL_StatusTypeDef UART_Transmit_Data(UART_HandleTypeDef *, uint8_t *, uint16_t);
#ifdef __cplusplus
}
#endif

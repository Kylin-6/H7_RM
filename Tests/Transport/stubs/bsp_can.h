#pragma once
#include "fdcan.h"
#include <stdint.h>
typedef void (*CAN_RxCallback_t)(FDCAN_HandleTypeDef *, uint32_t, uint8_t *, uint32_t, void *);
bool BSP_CAN_RegisterCallback(uint32_t, FDCAN_HandleTypeDef *, CAN_RxCallback_t, void *);
struct Struct_CAN_Tx_Msg {
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *);

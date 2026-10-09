#ifndef CLIENT_UI_CONFIG_H
#define CLIENT_UI_CONFIG_H

#include "usart.h"

// 未确认接线前不绑定。只能选择独占、已配置 115200 8N1 与 RX/TX DMA 的
// 裁判系统电源管理模块 User 串口，禁止复用 UART7 图传或其他设备串口。
inline UART_HandleTypeDef *BoardConfig_ClientUIUart()
{
    return nullptr;
}

#endif

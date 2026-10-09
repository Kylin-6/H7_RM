#ifndef CLIENT_UI_CONFIG_H
#define CLIENT_UI_CONFIG_H

#include "usart.h"

// 老步兵底盘裁判 User 串口：USART10，PE2 RX / PE3 TX，115200 8N1。
// 与 RM_3 down 一致；云台板不绑定此端口。只用于 ID 接收和 UI 发送。
inline UART_HandleTypeDef *BoardConfig_ClientUIUart()
{
    return &huart10;
}

#endif

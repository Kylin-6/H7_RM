#ifndef CLIENT_UI_LINK_H
#define CLIENT_UI_LINK_H
#include "client_ui_protocol.h"

bool ClientUILink_Init(bool receive);
void ClientUILink_Capture(uint8_t state, uint32_t timestamp_ms);
bool ClientUILink_Read(uint8_t &state, uint32_t &timestamp_ms);
bool ClientUILink_Send(uint8_t state);
#endif

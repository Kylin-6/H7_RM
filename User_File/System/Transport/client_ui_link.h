#ifndef CLIENT_UI_LINK_H
#define CLIENT_UI_LINK_H
#include "client_ui_protocol.h"

bool ClientUILink_Init(bool receive);
void ClientUILink_Capture(const Struct_Client_UI_Chassis &state, uint32_t timestamp_ms);
bool ClientUILink_Read(Struct_Client_UI_Chassis &state, uint32_t &timestamp_ms);
bool ClientUILink_Send(const Struct_Client_UI_Chassis &state);
#endif

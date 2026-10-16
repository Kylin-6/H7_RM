#ifndef CLIENT_UI_LINK_H
#define CLIENT_UI_LINK_H
#include "client_ui_protocol.h"
#include "client_ui_diagnostic.h"

bool ClientUILink_Init(bool receive);
void ClientUILink_Capture(const Struct_Client_UI_Gimbal &state, uint32_t timestamp_ms);
bool ClientUILink_Read(Struct_Client_UI_Gimbal &state, uint32_t &timestamp_ms);
bool ClientUILink_Send(const Struct_Client_UI_Gimbal &state);
void ClientUILink_CaptureDiagnostic(const Struct_Client_UI_Diagnostic& state, uint32_t timestamp_ms);
bool ClientUILink_ReadDiagnostic(Struct_Client_UI_Diagnostic& state, uint32_t& timestamp_ms);
bool ClientUILink_SendDiagnostic(const Struct_Client_UI_Diagnostic& state);
#endif

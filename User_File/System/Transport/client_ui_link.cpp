#include "client_ui_link.h"
#include "board_config.h"
#include "bsp_can.h"
#include "stm32h7xx_hal.h"

namespace
{
FDCAN_HandleTypeDef *bus;
Struct_Client_UI_Gimbal snapshot;
uint32_t received_ms;
bool valid;
uint8_t sequence;
Struct_Client_UI_Diagnostic diagnostic;
uint32_t diagnostic_ms;
bool diagnostic_valid;
uint8_t diagnostic_sequence;

void Receive(FDCAN_HandleTypeDef *rx_bus, uint32_t id, uint8_t *data, uint32_t length, void *)
{
    Struct_Client_UI_Gimbal next{};
    if (rx_bus != bus || id != CLIENT_UI_CAN_ID || !ClientUIProtocol_Decode(data, length, next))
        return;
    const uint32_t now = HAL_GetTick();
    const uint8_t delta = static_cast<uint8_t>(data[1] - sequence);
    if (valid && now - received_ms <= CLIENT_UI_MAX_AGE_MS && (delta == 0U || delta >= 128U))
        return;
    sequence = data[1];
    snapshot = next;
    received_ms = now;
    valid = true;
}

void ReceiveDiagnostic(FDCAN_HandleTypeDef* rx_bus, uint32_t id, uint8_t* data, uint32_t length, void*)
{
    Struct_Client_UI_Diagnostic next{};
    if (rx_bus != bus || id != CLIENT_UI_DIAGNOSTIC_ID || !ClientUIDiagnostic_Decode(data, length, next))
        return;
    const uint32_t now = HAL_GetTick();
    const uint8_t delta = static_cast<uint8_t>(data[1] - diagnostic_sequence);
    if (diagnostic_valid && now - diagnostic_ms <= CLIENT_UI_MAX_AGE_MS && (delta == 0U || delta >= 128U))
        return;
    diagnostic_sequence = data[1];
    diagnostic = next;
    diagnostic_ms = now;
    diagnostic_valid = true;
}
}

bool ClientUILink_Init(bool receive)
{
    bus = BoardConfig_Get().remote_forward_bus;
    return bus != nullptr && (!receive ||
                              (BSP_CAN_RegisterCallback(CLIENT_UI_CAN_ID, bus, Receive, nullptr) &&
                               BSP_CAN_RegisterCallback(CLIENT_UI_DIAGNOSTIC_ID, bus, ReceiveDiagnostic, nullptr)));
}

void ClientUILink_Capture(const Struct_Client_UI_Gimbal &state, uint32_t timestamp_ms)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    snapshot = state;
    received_ms = timestamp_ms;
    valid = true;
    __DMB();
    __set_PRIMASK(mask);
}

bool ClientUILink_Read(Struct_Client_UI_Gimbal &state, uint32_t &timestamp_ms)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool available = valid;
    state = snapshot;
    timestamp_ms = received_ms;
    __DMB();
    __set_PRIMASK(mask);
    return available;
}

bool ClientUILink_Send(const Struct_Client_UI_Gimbal &state)
{
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = bus;
    message.id = CLIENT_UI_CAN_ID;
    message.len = 8U;
    if (!ClientUIProtocol_Encode(state, sequence, message.data))
        return false;
    if (!CAN_Tx_Perform(&message))
        return false;
    ++sequence;
    return true;
}

void ClientUILink_CaptureDiagnostic(const Struct_Client_UI_Diagnostic& state, uint32_t timestamp_ms)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    diagnostic = state;
    diagnostic_ms = timestamp_ms;
    diagnostic_valid = true;
    __DMB();
    __set_PRIMASK(mask);
}

bool ClientUILink_ReadDiagnostic(Struct_Client_UI_Diagnostic& state, uint32_t& timestamp_ms)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool available = diagnostic_valid;
    state = diagnostic;
    timestamp_ms = diagnostic_ms;
    __DMB();
    __set_PRIMASK(mask);
    return available;
}

bool ClientUILink_SendDiagnostic(const Struct_Client_UI_Diagnostic& state)
{
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = bus;
    message.id = CLIENT_UI_DIAGNOSTIC_ID;
    message.len = 8U;
    ClientUIDiagnostic_Encode(state, diagnostic_sequence, message.data);
    if (!CAN_Tx_Perform(&message))
        return false;
    ++diagnostic_sequence;
    return true;
}

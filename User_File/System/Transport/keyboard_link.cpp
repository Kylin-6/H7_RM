#include "keyboard_link.h"

#include "board_config.h"
#include "bsp_can.h"
#include "daemon.h"
#include "stm32h7xx_hal.h"

namespace
{
FDCAN_HandleTypeDef* bus;
bool initialized;
bool received;
uint8_t tx_sequence;
Struct_Keyboard_Frame snapshot;
Class_ControlSequence sequence;
Daemon link_daemon{KEYBOARD_CONTROL_MAX_AGE_MS};

void Receive(FDCAN_HandleTypeDef* rx_bus, uint32_t id, uint8_t* data, uint32_t length, void*)
{
    Struct_Keyboard_Frame frame{};
    if (rx_bus != bus || id != KEYBOARD_CONTROL_ID || !KeyboardProtocol_Decode(data, length, frame))
        return;
    const uint32_t now = HAL_GetTick();
    link_daemon.Feed();
    if (!sequence.Accept(frame.sequence, now))
        return;
    frame.received_ms = now;
    snapshot = frame;
    received = true;
}
} // namespace

bool KeyboardLink_Init(bool receive)
{
    if (initialized)
        return true;
    bus = BoardConfig_Get().remote_forward_bus;
    if (bus == nullptr)
        return false;
    if (receive && (!DaemonManager::Register(link_daemon) ||
                    !BSP_CAN_RegisterCallback(KEYBOARD_CONTROL_ID, bus, Receive, nullptr)))
        return false;
    initialized = true;
    return true;
}

bool KeyboardLink_Send(Struct_Keyboard_Frame frame)
{
    if (!initialized)
        return false;
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = bus;
    message.id = KEYBOARD_CONTROL_ID;
    message.len = 8U;
    frame.sequence = ++tx_sequence;
    KeyboardProtocol_Encode(frame, message.data);
    return CAN_Tx_Perform(&message);
}

bool KeyboardLink_Read(Struct_Keyboard_Frame& frame)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool valid = initialized && received;
    if (valid)
        frame = snapshot;
    __DMB();
    __set_PRIMASK(mask);
    return valid;
}

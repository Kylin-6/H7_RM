#include "ClientUI.h"
#include "client_ui_protocol.h"
#include "client_ui_link.h"
#include "message_center.h"
#include "stm32h7xx_hal.h"

#if H7_CLIENT_UI_GIMBAL
#include "client_ui_config.h"
extern "C" {
#include "referee_UI_26.h"
#include "crc_ref.h"
}
#include <cmath>
#include <cstdio>
#include <cstring>
#else
#include "input_state.h"
#include "source_arbitration.h"
#include "keyboard_link.h"
#include "keyboard_logic.h"
#include "sbus.h"
#endif

namespace
{
bool initialized;
uint32_t last_send_ms;

#if H7_CLIENT_UI_GIMBAL
uint8_t friction; // 0 未知，1 停止，2 转动
uint32_t friction_ms;
bool uart_bound;
uint8_t slot;
uint16_t client_robot_id;
uint32_t rebuild_ms;
bool adding = true;

// 对应裁判系统 0x0301；只使用本 UI 的第 9 层，不删除其他应用图层。
bool SendPacket(uint16_t robot_id, uint16_t content_id, const void *content, uint16_t length)
{
    static uint8_t tx_sequence;
    uint8_t packet[120]{};
    const uint16_t payload_length = 6U + length;
    packet[0] = 0xA5U;
    packet[1] = static_cast<uint8_t>(payload_length);
    packet[2] = static_cast<uint8_t>(payload_length >> 8U);
    packet[3] = tx_sequence;
    Append_CRC8_Check_Sum(packet, 5U);
    packet[5] = 0x01U;
    packet[6] = 0x03U;
    packet[7] = static_cast<uint8_t>(content_id);
    packet[8] = static_cast<uint8_t>(content_id >> 8U);
    packet[9] = static_cast<uint8_t>(robot_id);
    packet[10] = static_cast<uint8_t>(robot_id >> 8U);
    const uint16_t client_id = 0x100U + robot_id;
    packet[11] = static_cast<uint8_t>(client_id);
    packet[12] = static_cast<uint8_t>(client_id >> 8U);
    std::memcpy(packet + 13U, content, length);
    Append_CRC16_Check_Sum(packet, payload_length + 9U);
    if (!RefereeTrySend(packet, payload_length + 9U))
        return false;
    ++tx_sequence;
    return true;
}

bool Draw(uint16_t robot_id, Struct_Client_UI_Chassis state, bool fresh, uint8_t friction_state)
{
    static_assert(sizeof(interaction_figure_t) == 15U, "UI graphic wire size");
    static_assert(sizeof(ext_client_custom_character_t) == 45U, "UI text wire size");
    const uint32_t operation = adding ? 1U : 2U;
    if (slot == 0U)
    {
        const uint8_t deletion[2] = {1U, 9U};
        return SendPacket(robot_id, 0x0100U, deletion, sizeof(deletion));
    }
    if (slot == 1U)
    {
        interaction_figure_t lines[2]{};
        char horizontal[3] = {'U', 'X', 'H'}, vertical[3] = {'U', 'X', 'V'};
        UILineDraw(&lines[0], horizontal, operation, 9U, 2U, 2U, 930U, 540U, 990U, 540U);
        UILineDraw(&lines[1], vertical, operation, 9U, 2U, 2U, 960U, 510U, 960U, 570U);
        return SendPacket(robot_id, 0x0102U, lines, sizeof(lines));
    }
    ext_client_custom_character_t text{};
    auto &graph = text.graphic_data_struct;
    graph.figure_name[0] = 'U';
    graph.figure_name[1] = 'S';
    graph.figure_name[2] = slot;
    graph.operate_type = operation;
    graph.figure_type = 7U;
    graph.layer = 9U;
    graph.color = 8U;
    graph.width = 2U;
    graph.start_x = 1400U;
    graph.start_y = 800U - (slot - 2U) * 45U;
    graph.details_a = 20U;
    const char *label;
    const char *value;
    if (slot == 2U)
    {
        label = "MODE";
        value = !fresh ? "UNKNOWN" : state.mode == 2U ? "KEYBOARD" : state.mode == 1U ? "REMOTE" : "STOP";
    }
    else if (slot == 3U)
    {
        label = "SPEED";
        value = !fresh || state.gear == 0U ? "UNKNOWN" : state.gear == 1U ? "1 / 9" : state.gear == 2U ? "2 / 18" : "3 / 30";
    }
    else if (slot == 4U)
    {
        label = "SPIN";
        value = !fresh ? "UNKNOWN" : state.spin == 1U ? "POSITIVE" : state.spin == 2U ? "NEGATIVE" : state.spin == 3U ? "FOLLOW" : "STOP";
    }
    else
    {
        label = "FRIC";
        value = friction_state == 0U ? "UNKNOWN" : friction_state == 2U ? "ROTATING" : "STOP";
    }
    std::snprintf(reinterpret_cast<char *>(text.data), sizeof(text.data), "%s: %-10s", label, value);
    graph.details_b = std::strlen(reinterpret_cast<const char *>(text.data));
    return SendPacket(robot_id, 0x0110U, &text, sizeof(text));
}
#endif
} // namespace

bool ClientUI_Init()
{
    if (!ClientUILink_Init(H7_CLIENT_UI_GIMBAL != 0))
        return false;
#if H7_CLIENT_UI_GIMBAL
    auto *uart = BoardConfig_ClientUIUart();
    uart_bound = uart != nullptr && RefereeInit(uart) != nullptr;
    if (uart != nullptr && !uart_bound)
        return false;
#endif
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    initialized = true;
    __DMB();
    __set_PRIMASK(mask);
    return true;
}

void ClientUI_Capture()
{
#if H7_CLIENT_UI_GIMBAL
    ShootFeedback feedback{};
    const bool valid = MessageCenter::Shoot_Feedback_Topic.ReadFresh(feedback, 100000U) && feedback.online;
    const uint8_t next = !valid ? 0U :
        (std::fabs(feedback.friction_left_speed_rad_s) > 1.0f || std::fabs(feedback.friction_right_speed_rad_s) > 1.0f) ? 2U : 1U;
    const uint32_t now = HAL_GetTick();
#else
    const auto input = InputState_Read();
    const auto decision = SourceArbitration_Resolve(input);
    Struct_Client_UI_Chassis next{};
    if (decision.armed)
    {
        next.mode = input.selected == InputSource::Keyboard ? 2U : 1U;
        if (next.mode == 2U)
        {
            Struct_Keyboard_Frame frame{};
            if (KeyboardLink_Read(frame))
                next.gear = (frame.keyboard & KeyboardInput::Shift) != 0U ? 2U : 1U;
        }
        else
        {
            Struct_SBUS_Frame frame{};
            if (SBUS_ReadLatest(&frame))
                next.gear = frame.channels[6] < -392 ? 1U : frame.channels[6] > 392 ? 3U : 2U;
        }
        const auto &command = decision.chassis;
        next.spin = command.mode == ChassisMode::FOLLOW_GIMBAL_YAW ? 3U :
                    command.angular_velocity_rad_s > 0.001f ? 1U :
                    command.angular_velocity_rad_s < -0.001f ? 2U : 0U;
    }
    const uint32_t now = HAL_GetTick();
#endif
#if H7_CLIENT_UI_GIMBAL
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    friction = next;
    friction_ms = now;
    __DMB();
    __set_PRIMASK(mask);
#else
    ClientUILink_Capture(next, now);
#endif
}

void ClientUI_Update()
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool ready = initialized;
#if H7_CLIENT_UI_GIMBAL
    const uint8_t friction_state = friction;
    const uint32_t captured_friction_ms = friction_ms;
#endif
    __DMB();
    __set_PRIMASK(mask);
    Struct_Client_UI_Chassis state{};
    uint32_t captured_ms = 0U;
    const bool valid = ClientUILink_Read(state, captured_ms);
    const uint32_t now = HAL_GetTick();
    if (!ready)
        return;
#if H7_CLIENT_UI_GIMBAL
    uint16_t robot_id;
    if (!uart_bound || !RefereeReadUIRobotId(&robot_id))
    {
        client_robot_id = 0U;
        return;
    }
    if (robot_id != client_robot_id || now - rebuild_ms >= 5000U)
    {
        client_robot_id = robot_id;
        slot = 0U;
        adding = true;
        rebuild_ms = now;
    }
    if (now - last_send_ms < 100U)
        return;
    if (Draw(robot_id, state, valid && now - captured_ms <= CLIENT_UI_MAX_AGE_MS,
             now - captured_friction_ms <= CLIENT_UI_MAX_AGE_MS ? friction_state : 0U))
    {
        last_send_ms = now;
        if (++slot == 6U)
        {
            slot = 1U;
            adding = false;
        }
    }
#else
    if (now - last_send_ms < 50U)
        return;
    if (ClientUILink_Send(valid && now - captured_ms <= CLIENT_UI_MAX_AGE_MS ? state : Struct_Client_UI_Chassis{}))
        last_send_ms = now;
#endif
}

#include "board_transport.h"
#include "bsp_can.h"
#include "message_center.h"
#include "transport_config.h"
#include "transport_protocol.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <cstring>
#include "../Daemon/registry_full.h"

#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); std::exit(1); } } while (0)
FDCAN_HandleTypeDef hfdcan3{3};
uint32_t test_primask;
static uint64_t now_us;
static CAN_RxCallback_t callback;
static void *callback_context;
static bool callback_ok=true;
namespace MessageCenter { TestTopic<ChassisCmd> Chassis_Command_Topic; }
namespace MessageCenter { TestTopic<ChassisFeedback> Chassis_Feedback_Topic; }
static Struct_CAN_Tx_Msg sent{};
static unsigned sent_count;
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *message) { sent = *message; ++sent_count; return true; }
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void) { return now_us; }
bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef *bus, CAN_RxCallback_t cb, void *context)
{ CHECK(id == 0x141 && bus == &hfdcan3); if (!callback_ok) return false; callback = cb; callback_context = context; return true; }
static void Receive(uint8_t seq, uint64_t rx_us, bool valid = true)
{
    ChassisCmd cmd{};
    cmd.mode = ChassisMode::NO_FOLLOW;
    uint8_t bytes[8];
    CHECK(TransportProtocol::EncodeChassisCmd(cmd, seq, bytes));
    if (!valid) bytes[1] = 0xFF;
    now_us = rx_us;
    callback(&hfdcan3, 0x141, bytes, 8, callback_context);
}
static void PollAt(uint64_t time_us) { now_us = time_us; BoardTransport_Poll(); }
int main(int argc,char **argv)
{
    if (argc>1) {
        if (!std::strcmp(argv[1],"registry")) CHECK(FillTestDaemonRegistry());
        else callback_ok=false;
        CHECK(!BoardTransport_Init() && !BoardTransport_IsOnline());
        if (callback) { Receive(1,1000); PollAt(1000); CHECK(MessageCenter::Chassis_Command_Topic.count==0); }
        CHECK(sent_count==0); return 0;
    }
    CHECK(BoardTransport_Init()); CHECK(!BoardTransport_IsOnline());
    CHECK(callback);
    Receive(250, 1000); PollAt(1000);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 1);
    CHECK(BoardTransport_IsOnline());
    Receive(250, 2000); PollAt(2000);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 1); // duplicate
    Receive(0, 3000); PollAt(3000);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 2); // wrap
    Receive(0, 104000, false); PollAt(104000);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 2); // invalid cannot reset
    Receive(0, 105000); PollAt(105000);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 3); // restart after timeout
    Receive(1, 106000); PollAt(207001);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 3); // stale pending
    Receive(0, 107000); PollAt(107000);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 3); // stale frame did not reset
    Receive(1, 208000); PollAt(208000);
    CHECK(MessageCenter::Chassis_Command_Topic.count == 4);
    CHECK(MessageCenter::Chassis_Command_Topic.at == 208000);
    ChassisCmd invalid_command{};
    uint8_t command_bytes[8];
    CHECK(TransportProtocol::EncodeChassisCmd(invalid_command, 9, command_bytes));
    command_bytes[1] = 0x1FU;
    uint8_t command_sequence;
    CHECK(!TransportProtocol::DecodeChassisCmd(command_bytes, 8, invalid_command, command_sequence));
    command_bytes[1] = 0x10U;
    CHECK(!TransportProtocol::DecodeChassisCmd(command_bytes, 7, invalid_command, command_sequence));
    invalid_command.velocity_x_m_s = std::numeric_limits<float>::quiet_NaN();
    CHECK(!TransportProtocol::EncodeChassisCmd(invalid_command, 9, command_bytes));
    ChassisFeedback feedback{};
    feedback.velocity_x_m_s = 1.25f;
    feedback.angular_velocity_rad_s = -0.5f;
    feedback.enabled = true;
    feedback.online = true;
    uint8_t fb_bytes[8];
    CHECK(TransportProtocol::EncodeChassisFeedback(feedback, 7, fb_bytes));
    ChassisFeedback decoded{};
    uint8_t fb_seq = 0;
    CHECK(TransportProtocol::DecodeChassisFeedback(fb_bytes, 8, decoded, fb_seq));
    CHECK(fb_seq == 7 && decoded.enabled && decoded.online && decoded.velocity_x_m_s == 1.25f);
    CHECK(decoded.angular_velocity_rad_s == -0.5f);
    fb_bytes[1] |= 0x04U;
    CHECK(!TransportProtocol::DecodeChassisFeedback(fb_bytes, 8, decoded, fb_seq));
    fb_bytes[1] &= ~0x04U;
    CHECK(!TransportProtocol::DecodeChassisFeedback(fb_bytes, 7, decoded, fb_seq));
    fb_bytes[1] = 0x21U;
    CHECK(!TransportProtocol::DecodeChassisFeedback(fb_bytes, 8, decoded, fb_seq));
    feedback.velocity_x_m_s = 40.0f;
    CHECK(!TransportProtocol::EncodeChassisFeedback(feedback, 8, fb_bytes));
    feedback.velocity_x_m_s = std::numeric_limits<float>::infinity();
    CHECK(!TransportProtocol::EncodeChassisFeedback(feedback, 8, fb_bytes));
    feedback.velocity_x_m_s = 1.25f;
    MessageCenter::Chassis_Feedback_Topic.PublishAt(feedback, 209000);
    PollAt(209000);
    CHECK(sent_count == 1 && sent.id == 0x222 && sent.len == 8);
    CHECK(TransportProtocol::DecodeChassisFeedback(sent.data, sent.len, decoded, fb_seq));
    CHECK(decoded.velocity_x_m_s == 1.25f);
    const auto count=MessageCenter::Chassis_Command_Topic.count;
    const auto at=MessageCenter::Chassis_Command_Topic.at;
    for (uint64_t ms=230; ms<=430; ms+=20) {
        Receive(1,ms*1000); PollAt(ms*1000);
        CHECK(BoardTransport_IsOnline() && MessageCenter::Chassis_Command_Topic.count==count);
    }
    CHECK(MessageCenter::Chassis_Command_Topic.at==at);
    ChassisCmd stale{}; CHECK(!MessageCenter::Chassis_Command_Topic.ReadFresh(stale,100000));
    Receive(1,500000,false); PollAt(500000);
    PollAt(531000); CHECK(!BoardTransport_IsOnline());
    CHECK(BoardTransport_OfflineDurationMs()==1);
    std::puts("PASS transport command sequence and age");
}

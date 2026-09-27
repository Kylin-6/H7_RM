#include "board_transport.h"
#include "bsp_can.h"
#include "message_center.h"
#include "transport_config.h"
#include "transport_protocol.h"
#include <cstdio>
#include <cstdlib>

#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #c); std::exit(1); } } while (0)
FDCAN_HandleTypeDef hfdcan2{2};
uint32_t test_primask;
static uint64_t now_us;
static CAN_RxCallback_t callback;
static void *callback_context;
namespace MessageCenter { TestTopic<ChassisFeedback> Chassis_Feedback_Topic; }
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void) { return now_us; }
bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef *bus, CAN_RxCallback_t cb, void *context)
{ CHECK(id == 0x222 && bus == &hfdcan2); callback = cb; callback_context = context; return true; }
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *) { return true; }
static void Receive(uint8_t seq, uint64_t rx_us, uint32_t id = 0x222, unsigned len = 8)
{
    ChassisFeedback feedback{};
    feedback.enabled = true;
    uint8_t bytes[8];
    CHECK(TransportProtocol::EncodeChassisFeedback(feedback, seq, bytes));
    now_us = rx_us;
    callback(&hfdcan2, id, bytes, len, callback_context);
}
static void PollAt(uint64_t us) { now_us = us; BoardTransport_Poll(); }
int main()
{
    BoardTransport_Init(); CHECK(callback);
    Receive(250, 1000, 0x223); PollAt(1000);
    Receive(250, 1000, 0x222, 7); PollAt(1000);
    CHECK(MessageCenter::Chassis_Feedback_Topic.count == 0);
    Receive(250, 1000); PollAt(1000);
    CHECK(MessageCenter::Chassis_Feedback_Topic.count == 1);
    Receive(250, 2000); PollAt(2000);
    CHECK(MessageCenter::Chassis_Feedback_Topic.count == 1);
    Receive(0, 3000); PollAt(3000);
    CHECK(MessageCenter::Chassis_Feedback_Topic.count == 2);
    Receive(1, 4000); PollAt(105000);
    CHECK(MessageCenter::Chassis_Feedback_Topic.count == 2); // stale pending
    Receive(0, 5000); PollAt(5000);
    CHECK(MessageCenter::Chassis_Feedback_Topic.count == 2);
    Receive(0, 104000); PollAt(104000);
    CHECK(MessageCenter::Chassis_Feedback_Topic.count == 3); // reset after timeout
    CHECK(MessageCenter::Chassis_Feedback_Topic.at == 104000);
    std::puts("PASS gimbal feedback receive and age");
}

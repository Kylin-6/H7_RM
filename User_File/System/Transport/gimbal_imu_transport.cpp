#include "gimbal_imu_transport.h"

#include "board_config.h"
#include "message_center.h"
#include "sys_timestamp.h"
#include "transport_protocol.h"

namespace
{
// 底盘位置闭环要求 10 ms 新鲜度；发送端也拒绝延迟积压的源样本。
constexpr uint64_t kControlSampleMaxAgeUs = 10000U;
bool initialized;
uint8_t divider;
uint8_t sequence;
bool has_sent;
uint32_t last_topic_sequence;
}

bool GimbalImuTransport_Init(void)
{
    initialized = BoardConfig_Get().remote_forward_bus != nullptr;
    return initialized;
}

void GimbalImuTransport_Update(void)
{
    if (!initialized || ++divider < 2U) { return; }
    divider = 0U;
    const auto sample = MessageCenter::INS_State_Topic.ReadWithMeta();
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    // 同一传感器样本只提交一次；无新样本时不能靠重复发送维持远端 freshness。
    if (!sample.valid || now_us < sample.timestamp_us ||
        now_us - sample.timestamp_us > kControlSampleMaxAgeUs ||
        (has_sent && sample.sequence == last_topic_sequence))
    {
        return;
    }
    Struct_CAN_Tx_Msg attitude{};
    Struct_CAN_Tx_Msg rate{};
    attitude.hfdcan = rate.hfdcan = BoardConfig_Get().remote_forward_bus;
    attitude.id = TransportProtocol::kGimbalImuAttitudeCanId;
    rate.id = TransportProtocol::kGimbalImuRateCanId;
    attitude.len = rate.len = TransportProtocol::kPayloadSize;
    if (!TransportProtocol::EncodeGimbalImu(sample.data, sequence, attitude.data, rate.data))
    {
        return;
    }
    // 每次尝试使用新序号，避免部分提交失败后混合两个采样时刻。
    ++sequence;
    // 两个槽独立提交，部分失败不算完成；下一周期重试，接收端只发布配对帧。
    const bool attitude_sent = CAN_Tx_Perform(&attitude);
    const bool rate_sent = CAN_Tx_Perform(&rate);
    if (attitude_sent && rate_sent)
    {
        last_topic_sequence = sample.sequence;
        has_sent = true;
    }
}

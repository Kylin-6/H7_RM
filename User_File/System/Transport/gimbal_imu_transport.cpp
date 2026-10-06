#include "gimbal_imu_transport.h"

#include "board_config.h"
#include "message_center.h"
#include "sys_timestamp.h"
#include "transport_protocol.h"

namespace
{
struct PendingFrame
{
    uint8_t bytes[8]{};
    uint64_t received_us = 0U;
    bool valid = false;
};
// RX ISR 写入，ControlTask 在保存/恢复 PRIMASK 的临界区内取得一致快照。
PendingFrame pending[2];
bool initialized;
bool has_sequence;
uint8_t last_sequence;
uint64_t last_valid_rx_us;

void Receive(FDCAN_HandleTypeDef *bus, uint32_t id, uint8_t *data,
             uint32_t size, void *)
{
    if (bus != BoardConfig_Get().remote_forward_bus || data == nullptr || size != 8U ||
        (id != TransportProtocol::kGimbalImuAttitudeCanId &&
         id != TransportProtocol::kGimbalImuRateCanId))
    {
        return;
    }
    const uint8_t index = id == TransportProtocol::kGimbalImuAttitudeCanId ? 0U : 1U;
    for (uint8_t byte = 0U; byte < 8U; ++byte)
    {
        pending[index].bytes[byte] = data[byte];
    }
    pending[index].received_us = SYS_Timestamp_Get_Microsecond();
    pending[index].valid = true;
}
}

bool GimbalImuTransport_Init(void)
{
    if (initialized) { return true; }
    auto *bus = BoardConfig_Get().remote_forward_bus;
    if (bus == nullptr) { return false; }
    // 初始化仅调用一次；任一注册失败时 Update 不消费部分注册的数据。
    initialized = BSP_CAN_RegisterCallback(TransportProtocol::kGimbalImuAttitudeCanId,
                                           bus, Receive, nullptr) &&
                  BSP_CAN_RegisterCallback(TransportProtocol::kGimbalImuRateCanId,
                                           bus, Receive, nullptr);
    return initialized;
}

void GimbalImuTransport_Update(void)
{
    if (!initialized) { return; }
    PendingFrame frames[2];
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    frames[0] = pending[0];
    frames[1] = pending[1];
    const bool paired = frames[0].valid && frames[1].valid &&
                        frames[0].bytes[0] == frames[1].bytes[0];
    if (paired)
    {
        pending[0].valid = false;
        pending[1].valid = false;
    }
    __DMB();
    __set_PRIMASK(primask);
    if (!paired) { return; }
    const uint64_t oldest_us = frames[0].received_us < frames[1].received_us ?
                                  frames[0].received_us : frames[1].received_us;
    const uint64_t newest_us = frames[0].received_us > frames[1].received_us ?
                                  frames[0].received_us : frames[1].received_us;
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    // 配对跨度最多 10 ms；新鲜度按较早帧，不能因第二帧迟到延长有效期。
    if (now_us < newest_us || now_us - oldest_us > GIMBAL_IMU_MAX_AGE_US ||
        newest_us - oldest_us > 10000U)
    {
        return;
    }
    INS_State state{};
    uint8_t sequence;
    if (!TransportProtocol::DecodeGimbalImu(frames[0].bytes, frames[1].bytes,
                                            state, sequence))
    {
        return;
    }
    if (has_sequence && oldest_us < last_valid_rx_us) { return; }
    if (has_sequence && oldest_us - last_valid_rx_us > GIMBAL_IMU_MAX_AGE_US)
    {
        has_sequence = false;
    }
    last_valid_rx_us = oldest_us;
    if (has_sequence && !TransportProtocol::SequenceNewer(sequence, last_sequence))
    {
        return;
    }
    MessageCenter::Gimbal_INS_State_Topic.PublishAt(state, oldest_us);
    last_sequence = sequence;
    has_sequence = true;
}

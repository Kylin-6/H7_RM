/**
 * @file sbus.h
 * @brief SBUS 遥控接收设备（老步兵配置）。
 * @details
 * 由 rm/demo 的 User/device/sbus 移植而来。原实现在设备层自行管理 UART5 的
 * DMA 缓冲与逐字节组帧；本版本改为复用框架 UART BSP 的 IDLE+DMA 双缓冲通道，
 * 因此只保留“帧对齐、解析、健康监测”这些协议职责，不再直接触碰 HAL 或 DMA。
 *
 * 时间基准使用 HAL_GetTick()，单位毫秒，天然支持 uint32 回绕。
 * @author  Kylin-6（原始实现）/ H7_BSP 移植
 */

#ifndef SBUS_H
#define SBUS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** SBUS 协议单帧长度，字节。 */
#define SBUS_FRAME_SIZE (25U)
/** SBUS 协议一帧承载的通道数。 */
#define SBUS_PROTOCOL_CHANNEL_COUNT (16U)
/** 本工程（老步兵）实际使用的通道数。 */
#define SBUS_CHANNEL_COUNT (10U)
/** 通道中位偏置：解析结果已减去该值，范围 [-1024, 1023]，中位 0。 */
#define SBUS_CHANNEL_OFFSET (1024)
/** 遥控摇杆满量程幅值，用于归一化与死区换算。 */
#define SBUS_CHANNEL_MAX (784.0F)
/** 超过该时间未收到任何完整帧即判定链路离线，毫秒。 */
#define SBUS_RX_TIMEOUT_MS (100U)
/** 健康帧新鲜度门限：超过该时间没有新的健康帧即不再算健康，毫秒。 */
#define SBUS_HEALTH_FRAME_TIMEOUT_MS (50U)

/** 接收诊断计数，供调试与故障定位使用。 */
typedef struct
{
    uint32_t rx_events;       ///< UART IDLE 事件次数
    uint32_t rx_bytes;        ///< 累计收到的字节数
    uint32_t valid_frames;    ///< 解析成功的完整帧数
    uint32_t invalid_frames;  ///< 无法对齐/校验失败的帧数
    uint32_t frame_lost_frames; ///< 携带 frame lost 标志的帧数
    uint32_t failsafe_frames;   ///< 携带 failsafe 标志的帧数
    uint16_t last_rx_size;      ///< 最近一次 IDLE 事件交付的字节数
} SBUS_Diagnostics;

/**
 * @brief 注册 UART5 接收回调并开始接收。
 * @note 必须在 System_Init() 之后（init_finished 置位后）调用，否则 BSP UART
 *       不会向回调分发数据。重复调用会重新启动接收。
 * @return true 表示已成功交给 UART BSP 启动接收。
 */
bool SBUS_Init(void);

/**
 * @brief 取最近一帧完整 SBUS 数据。
 * @param channels   输出通道数组，已减中位，范围 [-1024, 1023]。
 * @param frame_lost 输出 frame lost 标志，可为 NULL。
 * @param failsafe   输出 failsafe 标志，可为 NULL。
 * @return true 表示取得一帧；false 表示尚未收到过完整帧。
 */
bool SBUS_GetLatestFrame(int16_t channels[SBUS_CHANNEL_COUNT],
                         uint8_t* frame_lost,
                         uint8_t* failsafe);

/** @return true 表示最近 SBUS_RX_TIMEOUT_MS 内收到过完整帧。 */
bool SBUS_IsOnline(void);

/**
 * @brief 健康帧是否已超时。
 * @param timeout_ms 超时门限，毫秒。
 * @details 健康帧指 frame_lost 与 failsafe 均为 0 的帧。监测尚未开始时返回 false。
 */
bool SBUS_IsControlLostFor(uint32_t timeout_ms);

/**
 * @brief 健康帧是否已持续足够长时间且仍然新鲜。
 * @param duration_ms 需要持续的时间，毫秒。
 * @details 用于失联恢复去抖：既要连续健康 duration_ms，又要求最近
 *          SBUS_HEALTH_FRAME_TIMEOUT_MS 内仍有健康帧。
 */
bool SBUS_IsControlHealthyFor(uint32_t duration_ms);

/** @brief 拷贝一份接收诊断计数。 */
void SBUS_GetDiagnostics(SBUS_Diagnostics* diagnostics);

/**
 * @brief UART IDLE+DMA 接收回调，由框架 UART BSP 在中断上下文调用。
 * @warning 运行在中断中，不得阻塞；不要在内部调用可能睡眠的接口。
 */
void SBUS_RxCallback(uint8_t* buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_H */

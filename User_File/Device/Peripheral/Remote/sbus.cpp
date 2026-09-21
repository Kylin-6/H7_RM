/**
 * @file sbus.c
 * @brief SBUS 遥控接收设备实现（老步兵配置）。
 * @details
 * 移植自 rm/demo 的 User/device/sbus.c。原实现在设备层自行持有 DMA 缓冲、逐字节
 * 组帧并做 UART 硬复位；本版本把“字节搬运”交给框架 UART BSP（IDLE+DMA 双缓冲），
 * 设备层只负责帧对齐、协议解析、健康监测与诊断计数。
 */

#include "sbus.h"

#include "bsp_uart.h"
#include "main.h"
#include "usart.h"

/** SBUS 帧头与帧尾标志字节。 */
#define SBUS_FRAME_HEADER (0x0FU)
#define SBUS_FRAME_FOOTER (0x00U)
/** frame[23] 中的链路状态标志位。 */
#define SBUS_FLAG_FRAME_LOST (0x04U)
#define SBUS_FLAG_FAILSAFE (0x08U)

/** 一次 IDLE 事件最多向前回溯的字节数，用于在多次交付合并时对齐帧边界。 */
#define SBUS_ALIGN_SEARCH_BYTES (SBUS_FRAME_SIZE * 2U)

/** 中断与任务共享的最新一帧数据，整体由 PRIMASK 临界区保护。 */
typedef struct
{
    int16_t channels[SBUS_CHANNEL_COUNT];
    uint8_t frame_lost;
    uint8_t failsafe;
    uint32_t sequence;
    bool available;
} SBUS_Frame_t;

static volatile SBUS_Frame_t sbus_frame;
static volatile SBUS_Diagnostics sbus_diagnostics;
static volatile uint32_t sbus_last_frame_ms;
static volatile uint32_t sbus_last_healthy_ms;
static volatile uint32_t sbus_healthy_since_ms;
static volatile bool sbus_monitor_started;
static volatile bool sbus_healthy;

/**
 * @brief 进入极短临界区。
 * @details 保存并恢复 PRIMASK，而不是无条件开中断，便于在已屏蔽中断的上下文中安全嵌套。
 */
static uint32_t SBUS_CriticalEnter(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void SBUS_CriticalExit(uint32_t primask)
{
    __DMB();
    __set_PRIMASK(primask);
}

/**
 * @brief 解析一帧 25 字节 SBUS 数据。
 * @details 每通道 11 位、LSB 在前，自 frame[1] 起连续排布；解析结果减去中位 1024，
 *          得到范围 [-1024, 1023]、中位为 0 的摇杆值。
 */
static bool SBUS_ParseFrame(const uint8_t* frame,
                           int16_t channels[SBUS_CHANNEL_COUNT],
                           uint8_t* frame_lost,
                           uint8_t* failsafe)
{
    if (frame[0] != SBUS_FRAME_HEADER ||
        frame[SBUS_FRAME_SIZE - 1U] != SBUS_FRAME_FOOTER)
    {
        return false;
    }

    *frame_lost = (frame[23] & SBUS_FLAG_FRAME_LOST) != 0U ? 1U : 0U;
    *failsafe = (frame[23] & SBUS_FLAG_FAILSAFE) != 0U ? 1U : 0U;

    for (uint32_t channel = 0U; channel < SBUS_CHANNEL_COUNT; ++channel)
    {
        const uint32_t bit_offset = channel * 11U;
        const uint32_t byte_offset = bit_offset / 8U;
        const uint32_t shift = bit_offset % 8U;
        const uint32_t value = (uint32_t)frame[1U + byte_offset] |
                               ((uint32_t)frame[2U + byte_offset] << 8U) |
                               ((uint32_t)frame[3U + byte_offset] << 16U);

        channels[channel] =
            (int16_t)(((value >> shift) & 0x07FFU) - (uint32_t)SBUS_CHANNEL_OFFSET);
    }

    return true;
}

void SBUS_RxCallback(uint8_t* buffer, uint16_t length)
{
    if (buffer == NULL)
    {
        return;
    }

    sbus_diagnostics.rx_events++;
    sbus_diagnostics.rx_bytes += length;
    sbus_diagnostics.last_rx_size = length;

    if (length < SBUS_FRAME_SIZE)
    {
        return;
    }

    /* IDLE 交付可能包含多帧，也可能不是帧长的整数倍：从末尾向前对齐取最近一帧。 */
    const int32_t earliest_start = (int32_t)length - (int32_t)SBUS_ALIGN_SEARCH_BYTES;
    int32_t frame_start = -1;

    for (int32_t offset = (int32_t)length - (int32_t)SBUS_FRAME_SIZE;
         offset >= 0 && offset > earliest_start;
         --offset)
    {
        if (buffer[offset] == SBUS_FRAME_HEADER &&
            buffer[offset + SBUS_FRAME_SIZE - 1U] == SBUS_FRAME_FOOTER)
        {
            frame_start = offset;
            break;
        }
    }

    if (frame_start < 0)
    {
        sbus_diagnostics.invalid_frames++;
        return;
    }

    int16_t channels[SBUS_CHANNEL_COUNT];
    uint8_t frame_lost = 0U;
    uint8_t failsafe = 0U;

    if (!SBUS_ParseFrame(&buffer[frame_start], channels, &frame_lost, &failsafe))
    {
        sbus_diagnostics.invalid_frames++;
        return;
    }

    const uint32_t now = HAL_GetTick();
    const uint32_t primask = SBUS_CriticalEnter();

    for (uint32_t channel = 0U; channel < SBUS_CHANNEL_COUNT; ++channel)
    {
        sbus_frame.channels[channel] = channels[channel];
    }
    sbus_frame.frame_lost = frame_lost;
    sbus_frame.failsafe = failsafe;
    sbus_frame.sequence++;
    sbus_frame.available = true;
    sbus_last_frame_ms = now;
    sbus_diagnostics.valid_frames++;

    if (frame_lost == 0U && failsafe == 0U)
    {
        /* 健康帧：首次进入健康态时记录起始时刻，用于恢复去抖。 */
        if (!sbus_healthy)
        {
            sbus_healthy_since_ms = now;
        }
        sbus_healthy = true;
        sbus_last_healthy_ms = now;
        sbus_monitor_started = true;
    }
    else
    {
        sbus_healthy = false;
        if (frame_lost != 0U)
        {
            sbus_diagnostics.frame_lost_frames++;
        }
        if (failsafe != 0U)
        {
            sbus_diagnostics.failsafe_frames++;
        }
    }

    SBUS_CriticalExit(primask);
}

bool SBUS_Init(void)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t primask = SBUS_CriticalEnter();

    sbus_frame.available = false;
    sbus_frame.sequence = 0U;
    sbus_frame.frame_lost = 0U;
    sbus_frame.failsafe = 0U;
    sbus_last_frame_ms = now;
    sbus_last_healthy_ms = now;
    sbus_healthy_since_ms = now;
    sbus_monitor_started = false;
    sbus_healthy = false;

    SBUS_CriticalExit(primask);

    /* 复用框架 UART BSP 的 IDLE+DMA 通道；重复调用会重新启动接收。 */
    UART_Init(&huart5, SBUS_RxCallback);

    return huart5.hdmarx != NULL;
}

bool SBUS_GetLatestFrame(int16_t channels[SBUS_CHANNEL_COUNT],
                         uint8_t* frame_lost,
                         uint8_t* failsafe)
{
    const uint32_t primask = SBUS_CriticalEnter();

    const bool available = sbus_frame.available;
    if (available)
    {
        for (uint32_t channel = 0U; channel < SBUS_CHANNEL_COUNT; ++channel)
        {
            channels[channel] = sbus_frame.channels[channel];
        }
        if (frame_lost != NULL)
        {
            *frame_lost = sbus_frame.frame_lost;
        }
        if (failsafe != NULL)
        {
            *failsafe = sbus_frame.failsafe;
        }
    }

    SBUS_CriticalExit(primask);

    return available;
}

bool SBUS_IsOnline(void)
{
    const uint32_t primask = SBUS_CriticalEnter();
    const bool available = sbus_frame.available;
    const uint32_t last_frame_ms = sbus_last_frame_ms;
    SBUS_CriticalExit(primask);

    return available && ((HAL_GetTick() - last_frame_ms) < SBUS_RX_TIMEOUT_MS);
}

bool SBUS_IsControlLostFor(uint32_t timeout_ms)
{
    const uint32_t primask = SBUS_CriticalEnter();
    const bool monitor_started = sbus_monitor_started;
    const uint32_t last_healthy_ms = sbus_last_healthy_ms;
    SBUS_CriticalExit(primask);

    return monitor_started && ((HAL_GetTick() - last_healthy_ms) >= timeout_ms);
}

bool SBUS_IsControlHealthyFor(uint32_t duration_ms)
{
    const uint32_t primask = SBUS_CriticalEnter();
    const bool monitor_started = sbus_monitor_started;
    const bool healthy = sbus_healthy;
    const uint32_t last_healthy_ms = sbus_last_healthy_ms;
    const uint32_t healthy_since_ms = sbus_healthy_since_ms;
    SBUS_CriticalExit(primask);

    const uint32_t now = HAL_GetTick();

    return monitor_started && healthy &&
           ((now - last_healthy_ms) < SBUS_HEALTH_FRAME_TIMEOUT_MS) &&
           ((now - healthy_since_ms) >= duration_ms);
}

void SBUS_GetDiagnostics(SBUS_Diagnostics* diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }

    const uint32_t primask = SBUS_CriticalEnter();

    diagnostics->rx_events = sbus_diagnostics.rx_events;
    diagnostics->rx_bytes = sbus_diagnostics.rx_bytes;
    diagnostics->valid_frames = sbus_diagnostics.valid_frames;
    diagnostics->invalid_frames = sbus_diagnostics.invalid_frames;
    diagnostics->frame_lost_frames = sbus_diagnostics.frame_lost_frames;
    diagnostics->failsafe_frames = sbus_diagnostics.failsafe_frames;
    diagnostics->last_rx_size = sbus_diagnostics.last_rx_size;

    SBUS_CriticalExit(primask);
}

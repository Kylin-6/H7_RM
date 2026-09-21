/**
 * @file chassis_board.h
 * @brief 底盘板到云台板的板间链路接收端（老步兵云台板配置）。
 * @details
 * 老步兵是双板架构：底盘板负责底盘电机、SBUS 遥控接收与姿态解算，云台板负责云台
 * 与发射机构。底盘板把遥控关键通道整理成标准帧 0x065，经 FDCAN2 转发给云台板：
 *
 * - byte 0~1 火控开关（扳机）
 * - byte 2~3 拨弹盘速度（波轮）
 * - byte 4~5 Pitch 轴通道
 *
 * 三个通道均为大端 int16。底盘板侧对应的下发实现在
 * `Device/Peripheral/GimbalBoard/gimbal_board.*`（老步兵底盘板配置分支）。
 *
 * 本驱动只负责帧解码与超时判定，不参与控制；通道语义由 Application 解释。
 * @note 与云台板既有实现一致，当前只解析 0x065。底盘板还会下发 0x070（底盘 Yaw）
 *       与 0x075（机器人状态），云台板侧尚未使用，需要时再注册对应回调。
 * @author  Kylin-6（原始实现）/ H7_BSP 移植
 */
#ifndef CHASSIS_BOARD_H
#define CHASSIS_BOARD_H

#include "bsp_can.h"
#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

/** 板间链路帧 ID：遥控关键通道。 */
#define CHASSIS_BOARD_ID_REMOTE_CHANNELS (0x065U)
/** 通道数据有效期，单位 ms；超过后视为链路失效。 */
#define CHASSIS_BOARD_CHANNEL_TIMEOUT_MS (100U)

class Class_ChassisBoard
{
public:
    /**
     * @brief 绑定板间链路使用的 FDCAN 总线并注册 0x065 接收回调。
     * @param hfdcan 总线句柄，非空；通常是云台板的 FDCAN2。
     * @return true 表示句柄合法且回调注册成功。
     * @note 可重复调用，已初始化时直接返回 true，不重复占用回调槽。
     */
    bool Init(FDCAN_HandleTypeDef *hfdcan);

    /**
     * @brief 读取火控开关通道。
     * @param value 输出通道原始值；未收到数据或已超时时保持不变。
     * @return true 表示存在 100 ms 内的有效数据。
     */
    bool GetFire(int16_t *value);
    /** @brief 读取拨弹盘（波轮）速度通道，语义同 GetFire。 */
    bool GetDial(int16_t *value);
    /** @brief 读取 Pitch 轴通道，语义同 GetFire。 */
    bool GetPitch(int16_t *value);

    /**
     * @brief 判断板间链路是否在线。
     * @return true 表示最近 100 ms 内收到过 0x065。
     */
    bool IsOnline() const;

private:
    /** 接收中断入口；按 ID 分发到解析函数。 */
    static void RxCallback(FDCAN_HandleTypeDef *hfdcan,
                           uint32_t id,
                           uint8_t *data,
                           uint32_t len,
                           void *context);

    /** 解析 0x065 的三个大端 int16 通道并刷新时间戳。 */
    void OnRemoteChannels(const uint8_t *data, uint32_t len);

    /** 判断某个通道值的时效性，成功时写出数值。 */
    bool ReadChannel(const volatile int16_t &source, int16_t *value) const;

    FDCAN_HandleTypeDef *hfdcan = nullptr;
    bool initialized = false;

    volatile int16_t fire_channel = 0;
    volatile int16_t dial_channel = 0;
    volatile int16_t pitch_channel = 0;
    volatile uint32_t last_rx_ms = 0U;
    volatile bool received = false;
};

#endif /* CHASSIS_BOARD_H */

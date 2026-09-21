/**
 * @file gimbal_board.h
 * @brief 底盘板到云台板的板间 CAN 链路（老步兵配置）。
 * @details
 * 老步兵是双板架构：底盘板负责底盘电机、遥控接收与姿态解算，云台板负责云台与发射。
 * 两板之间用一组自定义标准帧通信，本文件承载移植自 rm/demo（User/bsp/bsp_CAN.c 的
 * CAN_BSP_SendGimbalPitch / SendGimbalYaw / SendRobotStatus）的三个下行帧：
 *
 * - 0x065 遥控关键通道（火控开关 / 发射速度 / Pitch 轴）
 * - 0x070 底盘 Yaw（DM 云台电机角度与地面系 Yaw）
 * - 0x075 机器人状态（枪口热量上限 / 冷却值 / 机器人 ID）
 *
 * 全部经框架 CAN BSP 的周期通道下发，同一 (总线, ID) 只保留最新值。
 * @author  Kylin-6（原始实现）/ H7_BSP 移植
 */

#ifndef GIMBAL_BOARD_H
#define GIMBAL_BOARD_H

#include "bsp_can.h"
#include "fdcan.h"

#include <stdint.h>

/** 下行帧 ID：遥控关键通道。 */
#define GIMBAL_BOARD_ID_REMOTE_CHANNELS (0x065U)
/** 下行帧 ID：底盘 Yaw。 */
#define GIMBAL_BOARD_ID_CHASSIS_YAW (0x070U)
/** 下行帧 ID：机器人状态。 */
#define GIMBAL_BOARD_ID_ROBOT_STATUS (0x075U)
/** 接收并转发给云台板的通道数，与 SBUS 使用通道数一致。 */
#define GIMBAL_BOARD_CHANNEL_COUNT (10U)

/* 0x065 帧各字段对应的 SBUS 通道索引（通道值已减中位 1024）。 */
/** 火控（发射）开关。 */
#define GIMBAL_BOARD_CHANNEL_FIRE_SWITCH (5U)
/** 发射速度。 */
#define GIMBAL_BOARD_CHANNEL_SHOOT_SPEED (8U)
/** 云台俯仰轴。 */
#define GIMBAL_BOARD_CHANNEL_PITCH (2U)

/** 火控开关极性：置 1 时发送前取反，用于遥控开关方向与云台板约定相反的情况。 */
#define GIMBAL_BOARD_FIRE_SWITCH_INVERT (1)

/** 0x070 帧的 Yaw 编码偏移，度。 */
#define GIMBAL_BOARD_YAW_OFFSET_DEG (180.0F)
/** 0x070 帧的 Yaw 编码倍率：(角度 - 偏移) * 倍率 存入 int16。 */
#define GIMBAL_BOARD_YAW_SCALE (100.0F)

class Class_GimbalBoard
{
public:
    /**
     * @brief 绑定板间链路使用的 FDCAN 总线。
     * @param motor_hfdcan 总线句柄，非空。
     * @return true 表示句柄合法并已记录。
     */
    bool Init(FDCAN_HandleTypeDef* motor_hfdcan);

    /**
     * @brief 发送 0x065：转发遥控关键通道。
     * @param sbus_channels SBUS 通道，已减中位，范围 [-1024, 1023]。
     */
    bool SendRemoteChannels(const int16_t sbus_channels[GIMBAL_BOARD_CHANNEL_COUNT]);

    /**
     * @brief 发送 0x070：底盘 Yaw 角度。
     * @param dm_yaw_rad     DM 云台电机角度，单位 rad。
     * @param ground_yaw_rad 地面系 Yaw 角度，单位 rad。
     * @details 两者均按 (角度 - 180) * 100 编码为 int16，与云台板既有解析一致。
     */
    bool SendChassisYaw(float dm_yaw_rad, float ground_yaw_rad);

    /**
     * @brief 发送 0x075：机器人状态。
     * @param heat_limit 枪管热量上限；裁判系统未接入时填 0。
     * @param cooling    冷却值；裁判系统未接入时填 0。
     * @param robot_id   机器人 ID；裁判系统未接入时填 0。
     */
    bool SendRobotStatus(uint16_t heat_limit, uint16_t cooling, uint8_t robot_id);

private:
    /** 经 CAN BSP 周期通道下发一帧，不改变总线绑定状态。 */
    bool Transmit(uint32_t id, const uint8_t data[8]);

    FDCAN_HandleTypeDef* hfdcan = nullptr;
};

#endif /* GIMBAL_BOARD_H */

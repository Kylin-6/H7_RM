/**
 * @file gimbal_board.h
 * @brief 底盘板到云台板的板间 CAN 链路发送端（老步兵配置）。
 * @details
 * 老步兵是双板架构：底盘板负责底盘电机、Yaw 轴、SBUS 遥控接收与姿态解算，云台板
 * 负责云台 Pitch 与发射机构。两板之间用一组自定义标准帧通信，本驱动承载移植自
 * rm/demo（User/bsp/bsp_CAN.c 的 CAN_BSP_SendGimbalPitch / SendGimbalYaw /
 * SendRobotStatus）的三个下行帧：
 *
 * - 0x065 遥控关键通道（火控开关 / 发射速度 / Pitch 轴，三个大端 int16，byte6 版本/模式/许可、byte7 序号）
 * - 0x070 底盘 Yaw（Yaw 电机角度与地面系 Yaw，两个大端 int16，单位 0.01°）
 * - 0x075 机器人状态（枪管热量上限 / 冷却值 / 机器人 ID，裁判系统未接入时为 0）
 *
 * 全部经框架 CAN BSP 的周期通道下发，同一 (总线, ID) 只保留最新值，因此调用方
 * 按固定周期刷新即可，不需要自己排队重发。通道选择与极性属于整机约定，由调用方
 * （Application/Input）决定；本驱动只做帧编码。
 * @note  本板只发不收：链路单向上没有应答，驱动不提供在线判定。
 * @author  Kylin-6（原始实现）/ H7_BSP 移植
 */

#ifndef GIMBAL_BOARD_H
#define GIMBAL_BOARD_H

#include "bsp_can.h"
#include "fdcan.h"
#include "keyboard_protocol.h"

#include <stdint.h>

/** 下行帧 ID：遥控关键通道。 */
#define GIMBAL_BOARD_ID_REMOTE_CHANNELS (0x065U)
/** 下行帧 ID：底盘 Yaw。 */
#define GIMBAL_BOARD_ID_CHASSIS_YAW (0x070U)
/** 下行帧 ID：机器人状态。 */
#define GIMBAL_BOARD_ID_ROBOT_STATUS (0x075U)

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
     * @param fire_switch 火控开关通道，已减中位，范围 [-1024, 1023]。
     * @param shoot_speed 发射速度（拨盘）通道，语义同上。
     * @param pitch       云台俯仰轴通道，语义同上。
     * @details 三个通道按大端 int16 依次写入 byte 0~5；链路失效时由调用方传 0，
     *          byte6 标记版本 1、所选来源及健康许可，byte7 递增序号；非遥控档通道为零。
     */
    bool SendRemoteChannels(int16_t fire_switch, int16_t shoot_speed, int16_t pitch, ReceiverMode mode, bool permitted);

    /**
     * @brief 发送 0x070：底盘 Yaw 角度。
     * @param dm_yaw_rad     Yaw 电机角度，输入单位 rad。
     * @param ground_yaw_rad 地面系 Yaw 角度，输入单位 rad。
     * @details 线上单位是度：int16 = (yaw_deg - 180) * 100，即 0.01°/LSB，
     *          rad → degree 只在本 Encode 边界发生；超出 int16 值域时按边界夹取，
     *          不静默回绕到另一侧角度。
     * @note  地面系 Yaw 目前没有独立来源，调用方按老工程约定用 Yaw 电机角度代替。
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
    uint8_t remote_sequence = 0U;
};

#endif /* GIMBAL_BOARD_H */

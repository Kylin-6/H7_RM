#include "board_config.h"

const BoardHardware &BoardConfig_Get(void)
{
    /*
     * 老步兵底盘板：本板负责底盘四路 DM 麦轮、Yaw 轴
     * DM 电机、SBUS 遥控接收，并经 FDCAN2 向云台板下发 0x065/0x070/0x075 下行帧。
     *
     * 1. 底盘四轮与 Yaw 轴同在 FDCAN1；Yaw 是整机云台偏航轴，但电机挂在本板，
     *    因此由 Chassis 持有，见 Application/Chassis/Chassis.cpp。
     * 2. 板载 BMI088 提供 Yaw 角速度前馈与姿态；未安装 W25Q64JV，不做 Flash 初始化。
     * 3. WS2812 用作武装状态指示；USB 调试不接入。
     */
    static const BoardHardware hardware{
        &hfdcan1, // gimbal_yaw_bus：Yaw 轴 DM 电机
        nullptr,  // gimbal_pitch_bus（本板不控云台俯仰）
        &hfdcan1, // chassis_wheel_bus：四路底盘 DM 电机
        nullptr,  // chassis_steer_bus（麦轮无舵向电机）
        nullptr,  // shoot_bus（本板不控发射）
        true,     // imu：BMI088
        false,    // flash：未安装 W25Q64JV
        true,     // adc
        true,     // power
        true,     // indicators：WS2812
        false,    // usb_debug
        &hfdcan2, // remote_forward_bus：0x065/0x070/0x075 下行云台板
    };
    return hardware;
}

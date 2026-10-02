#include "board_config.h"

const BoardHardware &BoardConfig_Get(void)
{
    /*
     * 老步兵云台板（LEGACY_INFANTRY_GIMBAL）：
     * 1. Yaw 轴由底盘板主控（双板分工），云台板不接 Yaw 电机；
     *    Pitch 由单轴 Gimbal 的 IMU 力矩闭环控制。
     * 2. 电源开关由 Init 内的 LEGACY_INFANTRY_GIMBAL 分支固定为只开 5V，
     *    两路 24V 保持关闭，电机使用既有外部供电；未经硬件确认不得改变。
     * 3. 板上未安装 W25Q64JV，BMI088 硬件故障（姿态来自 FDCAN3 的 DM-IMU），
     *    ADC 采样链路不可用，均不初始化。
     * 4. DM3519 摩擦轮在 FDCAN1，M2006 拨弹盘在 FDCAN2。
     */
    static const BoardHardware hardware{
        nullptr,  // gimbal_yaw_bus（本板不拥有 Yaw）
        &hfdcan1, // gimbal_pitch_bus
        nullptr,  // chassis_wheel_bus（本板不控底盘）
        nullptr,  // chassis_steer_bus
        &hfdcan1, // shoot_bus
        false,    // imu（BMI088 硬件故障，姿态来自 DM-IMU）
        false,    // flash（未安装 W25Q64JV）
        false,    // adc
        true,     // power（仅板载 5V）
        true,     // indicators
        true,     // usb_debug
        &hfdcan3, // external_imu_bus：DM-IMU
        &hfdcan2, // shoot_loader_bus：M2006
        &hfdcan2, // remote_forward_bus：0x065
    };
    return hardware;
}

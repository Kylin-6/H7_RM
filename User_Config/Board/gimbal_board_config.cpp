#include "board_config.h"

const BoardHardware &BoardConfig_Get(void)
{
    /*
     * 老步兵云台板（LEGACY_INFANTRY_GIMBAL）：
     * 1. Yaw 轴由底盘板主控（双板分工），云台板不接 Yaw 电机；
     *    Pitch 由 Application/Pitch 直接驱动，不走框架 Gimbal 装配，
     *    因此框架云台两条总线不绑定。
     * 2. 电源开关由 Init 内的 LEGACY_INFANTRY_GIMBAL 分支固定为只开 5V，
     *    两路 24V 保持关闭，电机使用既有外部供电；未经硬件确认不得改变。
     * 3. 板上未安装 W25Q64JV，BMI088 硬件故障（姿态来自 FDCAN3 的 DM-IMU），
     *    ADC 采样链路不可用，均不初始化。
     * 4. 发射机构（DM3519 摩擦轮 + M2006 拨弹盘）在 FDCAN1。
     */
    static const BoardHardware hardware{
        nullptr,          // gimbal_yaw_bus（框架 Gimbal 不装配）
        nullptr,          // gimbal_pitch_bus（Pitch 由 Application/Pitch 驱动）
        nullptr,          // chassis_wheel_bus（本板不控底盘）
        nullptr,          // chassis_steer_bus
        &hfdcan1,         // shoot_bus
        false,            // imu（BMI088 硬件故障，姿态来自 DM-IMU）
        false,            // flash（未安装 W25Q64JV）
        false,            // adc
        true,             // power（仅板载 5V）
        true,             // indicators
        true,             // usb_debug
    };
    return hardware;
}

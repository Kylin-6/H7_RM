#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include "../physical_units.h"
#include "input_state.h"
#include <cstdint>

// 底盘机构与控制参数；总线归属由 BoardConfig 提供。

/**
 * 老步兵底盘机构与控制参数：FDCAN1 上四路 DM 麦轮 + 一路 Yaw DM 电机（MIT 速度环）。
 * 数值沿用老工程（rm/demo 的 APP/ChassisTask.c、APP/GimbalTask.c 与 User/bsp/bsp_def.h）
 * 与老步兵测试分支的实车版本，未经重新标定不得改动。
 *
 * 量纲约定：老步兵的三轴速度仍是实车验证过的抽象量纲（与麦轮预混后的 DM 轮速同量纲，
 * 即 rad/s 量级的轮速和），没有可信的 m/s 标定，因此不伪造 SI 换算。输入仲裁按框架
 * SI 边界（INPUT_MAX_*）校验，Input 侧用 * _ToSi 归一化、本模块用 * _FromSi 还原，
 * 两处共用本文件中的同一份比例定义。
 */
struct InfantryChassisConfig
{
    /** 三轴输入上限，抽象速度单位。 */
    float velocity_x_max = 30.0f;
    float velocity_y_max = 30.0f;
    float angular_velocity_max = 50.0f;
    /** 麦轮单轮目标限幅，与三轴同量纲。 */
    float wheel_speed_max = 30.0f;
    /** 控制路径周期与 1 kHz 调度下的分频：2 对应 2 ms，与老工程一致。 */
    float control_dt_s = 0.002f;
    uint8_t control_divider = 2U;
    /** 三轴非对称速率限制，单位 抽象速度/s。 */
    float x_accel_limit = 180.0f;
    float x_decel_limit = 180.0f;
    float x_release_limit = 120.0f;
    float x_reverse_limit = 300.0f;
    float y_accel_limit = 180.0f;
    float y_decel_limit = 180.0f;
    float y_release_limit = 120.0f;
    float y_reverse_limit = 300.0f;
    float w_accel_limit = 350.0f;
    float w_decel_limit = 350.0f;
    float w_release_limit = 250.0f;
    float w_reverse_limit = 600.0f;
    /** 三轴 S 曲线 jerk 上限，单位 抽象速度/s²；约 10 ms 建立最大加速度。 */
    float x_jerk_limit = 18000.0f;
    float y_jerk_limit = 18000.0f;
    float w_jerk_limit = 35000.0f;
    /** 零速吸附门限，同量纲。 */
    float planning_threshold = 0.1f;

    /** 四路底盘 DM 电机（速度模式）节点 ID 与主控接收 ID。 */
    uint8_t motor_id[4] = {0x01U, 0x02U, 0x03U, 0x04U};
    uint16_t motor_master_id[4] = {0x60U, 0x61U, 0x62U, 0x63U};
    /** 与电机端 PMAX/VMAX/TMAX 一致：位置 ±12.5 rad、速度 ±30 rad/s、转矩 ±10 N·m。 */
    float motor_position_max_rad = 12.5f;
    float motor_velocity_max_rad_s = 30.0f;
    float motor_torque_max_nm = 10.0f;

    /** Yaw 轴 DM 电机（MIT 模式）节点 ID 与主控接收 ID。 */
    uint8_t yaw_motor_id = 0x03U;
    uint16_t yaw_motor_master_id = 0x05U;
    float yaw_position_max_rad = 3.14f;
    float yaw_velocity_max_rad_s = 30.0f;
    float yaw_torque_max_nm = 10.0f;
    /** 摇杆给出的 Yaw 速度上限，rad/s。 */
    float yaw_speed_max_rad_s = 8.0f;
    /** 底盘自转补偿后的 Yaw 总速度上限，rad/s。 */
    float yaw_total_speed_max_rad_s = 15.0f;
    /** Yaw S 曲线试验参数：加速度 rad/s²、jerk rad/s³。 */
    float yaw_trajectory_accel_max = 150.0f;
    float yaw_trajectory_jerk_max = 7500.0f;
    /** 机体系 Z 轴角速度前馈增益，用于抑制底盘自转耦合。 */
    float yaw_rate_feedforward_gain = 1.0f;
    /** Yaw 速率限制：加速度上限随摇杆比例在 min/max 之间线性插值。 */
    float yaw_accel_limit_min = 60.0f;
    float yaw_accel_limit_max = 150.0f;
    float yaw_decel_limit = 120.0f;
    float yaw_release_limit = 75.0f;
    float yaw_reverse_limit = 250.0f;
    /** MIT 参数：位置增益恒为 0，即纯速度 + 阻尼控制，位置目标固定为 0。 */
    float yaw_mit_kp = 0.0f;
    float yaw_mit_kd_center = 1.4f;
    float yaw_mit_kd_moving = 1.0f;
    float yaw_mit_kd_reverse = 1.6f;
    float yaw_mit_kd_slew_rate = 5.0f;
    /** 力矩前馈：由速度规划的加速度换算，含增益、限幅与变化率限制，N·m。 */
    float yaw_mit_torque_gain = 0.002f;
    float yaw_mit_torque_max_nm = 0.35f;
    float yaw_mit_torque_slew_rate = 8.0f;

    /** 云台跟随：机械安装下 Yaw 角为该值时对应底盘正前方，rad。 */
    float follow_forward_rad = 3.14159265f;
    /** 跟随云台时底盘角速度的比例增益，1/s。 */
    float follow_kp = 8.0f;
    /** 跟随旋转独立上限，抽象速度单位；不限制小陀螺目标。 */
    float follow_rotation_max = 5.0f;
    /** 跟随角度死区，rad（机械标定输入 2°）。 */
    float follow_deadband_rad = 2.0f * 3.14159265f / 180.0f;
};

constexpr InfantryChassisConfig kInfantryChassisConfig{};

/** 抽象速度 → 框架 SI 仲裁边界（Input 侧提交前调用）。 */
inline float Chassis_TranslateX_ToSi(float value)
{
    return value * (INPUT_MAX_TRANSLATION_M_S / kInfantryChassisConfig.velocity_x_max);
}

inline float Chassis_TranslateY_ToSi(float value)
{
    return value * (INPUT_MAX_TRANSLATION_M_S / kInfantryChassisConfig.velocity_y_max);
}

inline float Chassis_Rotation_ToSi(float value)
{
    return value * (INPUT_MAX_ROTATION_RAD_S / kInfantryChassisConfig.angular_velocity_max);
}

/** SI 仲裁边界 → 抽象速度（本模块还原目标时调用）。 */
inline float Chassis_TranslateX_FromSi(float value)
{
    return value * (kInfantryChassisConfig.velocity_x_max / INPUT_MAX_TRANSLATION_M_S);
}

inline float Chassis_TranslateY_FromSi(float value)
{
    return value * (kInfantryChassisConfig.velocity_y_max / INPUT_MAX_TRANSLATION_M_S);
}

inline float Chassis_Rotation_FromSi(float value)
{
    return value * (kInfantryChassisConfig.angular_velocity_max / INPUT_MAX_ROTATION_RAD_S);
}

#endif

#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include "../physical_units.h"
#include "input_state.h"
#include <cstdint>

// 底盘机构与控制参数；总线归属由 BoardConfig 提供。
struct ChassisPidConfig
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
};

struct ChassisConfig
{
    float half_length_m = 0.163f; // 几何中心到轮模块的纵向距离，影响旋转速度分量。
    float half_width_m = 0.163f; // 几何中心到轮模块的横向距离，须大于零。
    float wheel_radius_m = 0.058f; // 有效滚动半径，用于 m/s 与输出轴 rad/s 换算。
    float feedback_alpha = 0.032258f; // 每个 1 ms 周期的一阶平滑权重，不是 100 Hz 发布周期的权重。
    float stop_speed_m_s = 0.001f; // 近零速度阈值，低于它时行走轮停转并保持当前舵角。
    // 轮索引与运动学数组一致；这是机械偏置，不能替代增量编码器的上电寻零。
    float steer_offset_rad[4] = {
        DegToRad(102.5f), DegToRad(12.5f),
        DegToRad(137.5f), DegToRad(145.0f)};
    uint8_t motor_id[4] = {1U, 2U, 3U, 4U}; // 电机编号而非 CAN 报文 ID；两组在各自总线上使用相同索引。
    ChassisPidConfig wheel_speed_pid{4.5f, 0.05f, 0.0f, 3000.0f, 16000.0f};
    ChassisPidConfig steer_angle_pid{30.0f, 0.2f, 0.0f,
                                     DegToRad(200.0f), DegToRad(1000.0f)};
    ChassisPidConfig steer_speed_pid{4.0f, 4.0f, 0.0f, 3000.0f, 15000.0f};
};

constexpr ChassisConfig kChassisConfig{};

#if LEGACY_INFANTRY_CHASSIS

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
struct LegacyInfantryChassisConfig
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
    float yaw_speed_max_rad_s = 15.0f;
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
};

constexpr LegacyInfantryChassisConfig kLegacyChassisConfig{};

/** 抽象速度 → 框架 SI 仲裁边界（Input 侧提交前调用）。 */
inline float LegacyChassis_TranslateX_ToSi(float value)
{
    return value * (INPUT_MAX_TRANSLATION_M_S / kLegacyChassisConfig.velocity_x_max);
}

inline float LegacyChassis_TranslateY_ToSi(float value)
{
    return value * (INPUT_MAX_TRANSLATION_M_S / kLegacyChassisConfig.velocity_y_max);
}

inline float LegacyChassis_Rotation_ToSi(float value)
{
    return value * (INPUT_MAX_ROTATION_RAD_S / kLegacyChassisConfig.angular_velocity_max);
}

/** SI 仲裁边界 → 抽象速度（本模块还原目标时调用）。 */
inline float LegacyChassis_TranslateX_FromSi(float value)
{
    return value * (kLegacyChassisConfig.velocity_x_max / INPUT_MAX_TRANSLATION_M_S);
}

inline float LegacyChassis_TranslateY_FromSi(float value)
{
    return value * (kLegacyChassisConfig.velocity_y_max / INPUT_MAX_TRANSLATION_M_S);
}

inline float LegacyChassis_Rotation_FromSi(float value)
{
    return value * (kLegacyChassisConfig.angular_velocity_max / INPUT_MAX_ROTATION_RAD_S);
}

#endif /* LEGACY_INFANTRY_CHASSIS */

#endif

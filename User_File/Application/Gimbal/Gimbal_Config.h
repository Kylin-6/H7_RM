#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#include "board_config.h"
#include <cstdint>

// 本分支的云台 App 只有 Pitch；Yaw 属于底盘板，不保存 Yaw 电机或控制参数。
// 内部角度 rad、角速度 rad/s、力矩 N·m。机构标定在这里，总线在 BoardConfig。
struct Struct_Gimbal_PitchTorque_Config
{
    /** IMU 外环位置刚度，N·m/rad。 */
    float position_kp = 0.52f;
    /** 目标斜坡限速，rad/s（Class_Slope）。 */
    float target_rate_rad_s = 3.0f;
    /** 目标速度前馈：正 / 负方向增益不对称，N·m·s/rad。 */
    float ff_velocity_positive = 0.012f;
    float ff_velocity_negative = 0.018f;
    /** IMU 角速度阻尼系数，N·m·s/rad。 */
    float imu_velocity_damping = 0.060f;
    /** IMU 角速度低通时间常数，s（Class_Filter_IIR_First_Order）。 */
    float imu_velocity_filter_tau_s = 0.010f; // 0：INS 来源已滤波，直接使用。
    /** Stribeck 摩擦：静摩擦 / 库仑摩擦力矩（正负方向不对称），N·m。 */
    float static_friction_positive_nm = 0.038f;
    float static_friction_negative_nm = 0.040f;
    float coulomb_friction_positive_nm = 0.015f;
    float coulomb_friction_negative_nm = 0.020f;
    /** 静摩擦→库仑过渡特征速度，rad/s；误差→补偿方向增益，1/s；tanh 平滑速度，rad/s。 */
    float stribeck_velocity_rad_s = 0.100f;
    float stribeck_error_gain = 3.0f;
    float stribeck_smooth_rad_s = 0.080f;
    /** 低带宽扰动估计：静止时学习重力/负载，运动时保持，停机清零。 */
    float disturbance_integral_gain = 0.80f;
    float disturbance_max_nm = 0.150f;
    float disturbance_target_speed_rad_s = 0.10f;
    float disturbance_actual_speed_rad_s = 0.10f;
    /** 输出力矩总限幅，N·m。 */
    float torque_limit_nm = 0.5f;
    /** 力矩符号：电机正方向与 IMU Pitch 正方向相反时为 -1。 */
    float torque_sign = 1.0f;
    /** 每次使能前（含恢复）延迟下发使能帧，ms；期间不发 MIT 指令。 */
    uint32_t enable_delay_ms = 2000U;
};

struct Struct_Gimbal_Motor_Config
{
    FDCAN_HandleTypeDef *bus = nullptr;
    uint8_t id = 1;
    uint16_t feedback_id = 0x101;
    bool reverse = false;
    // Meta 达妙协议示例，必须与电机端 PMAX/VMAX/TMAX 一致，不代表所有型号通用。
    float position_max = 12.5f;
    float velocity_max = 45.0f;
    float torque_max = 18.0f;
};

struct Struct_Gimbal_Config
{
    Struct_Gimbal_Motor_Config pitch;
    Struct_Gimbal_PitchTorque_Config pitch_torque;
    float pitch_min = -0.6981317f; // -40 deg。
    float pitch_max = 0.2617994f;  // +15 deg。
    uint64_t ins_max_age_us = 100000U;
};

inline Struct_Gimbal_Config Gimbal_Default_Config()
{
    Struct_Gimbal_Config config;
    config.pitch.bus = BoardConfig_Get().gimbal_pitch_bus;
    config.pitch.id = 0x09U;
    config.pitch.feedback_id = 0x019U;
    config.pitch.position_max = 3.14f;
    config.pitch.velocity_max = 30.0f;
    config.pitch.torque_max = 10.0f;
    // 电机正方向与 DM-IMU Pitch 正方向相反，只在最终力矩边界取负。
    config.pitch_torque.torque_sign = -1.0f;
    // DM-IMU 桥已差分并滤波；不二次低通、不更换已标定的反馈源。
    config.pitch_torque.imu_velocity_filter_tau_s = 0.0f;
    return config;
}

#endif

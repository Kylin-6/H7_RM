#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#include "fdcan.h"
#include "board_config.h"

enum class GimbalGyroAxis : uint8_t { X, Y, Z };

/** 轴配置：DualAxis = 双轴都由本板控制；PitchOnly = 只有 Pitch 由本板控制
 * （老步兵双板分工下 Yaw 由底盘板主控），Yaw 目标被忽略、Yaw 电机不初始化。 */
enum class GimbalAxisMode : uint8_t { DualAxis, PitchOnly };

/** Pitch 控制结构：MotorMit = 目标换算后由电机端 MIT kp/kd 闭环（框架默认）；
 * ImuTorque = IMU 角度外环 PID + 前馈/阻尼/摩擦补偿，纯力矩 t_ff 下发
 * （老步兵云台板低摩擦机构的实测控制结构）。 */
enum class GimbalPitchControl : uint8_t { MotorMit, ImuTorque };

/**
 * ImuTorque 模式参数。默认值取老步兵云台板 `Application/Pitch` 的实测标定，
 * 摩擦/扰动参数依赖具体机构，换机构必须重新标定。
 */
struct Struct_Gimbal_PitchTorque_Config
{
    /** IMU 外环位置刚度，N·m/rad。 */
    float position_kp = 0.42f;
    /** 目标斜坡限速，rad/s（Class_Slope）。 */
    float target_rate_rad_s = 3.0f;
    /** 目标速度前馈：正 / 负方向增益不对称，N·m·s/rad。 */
    float ff_velocity_positive = 0.012f;
    float ff_velocity_negative = 0.018f;
    /** IMU 角速度阻尼系数，N·m·s/rad。 */
    float imu_velocity_damping = 0.043f;
    /** IMU 角速度低通时间常数，s（Class_Filter_IIR_First_Order）。 */
    float imu_velocity_filter_tau_s = 0.010f;
    /** Stribeck 摩擦：静摩擦 / 库仑摩擦力矩（正负方向不对称），N·m。 */
    float static_friction_positive_nm = 0.038f;
    float static_friction_negative_nm = 0.040f;
    float coulomb_friction_positive_nm = 0.015f;
    float coulomb_friction_negative_nm = 0.020f;
    /** 静摩擦→库仑过渡特征速度，rad/s；误差→补偿方向增益，1/s；tanh 平滑速度，rad/s。 */
    float stribeck_velocity_rad_s = 0.100f;
    float stribeck_error_gain = 3.0f;
    float stribeck_smooth_rad_s = 0.080f;
    /** 低带宽扰动估计（静止时学习重力/负载）：积分增益、限幅、学习窗口、衰减。 */
    float disturbance_integral_gain = 0.40f;
    float disturbance_max_nm = 0.030f;
    float disturbance_target_speed_rad_s = 0.10f;
    float disturbance_actual_speed_rad_s = 0.10f;
    float disturbance_decay_tau_s = 0.20f;
    /** 输出力矩总限幅，N·m。 */
    float torque_limit_nm = 0.5f;
    /** 力矩符号：电机正方向与 IMU Pitch 正方向相反时为 -1。 */
    float torque_sign = 1.0f;
    /** 每次使能前（含恢复）延迟下发使能的时间，ms；期间不发 MIT 指令。 */
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
    GimbalAxisMode axis_mode = GimbalAxisMode::DualAxis;
    GimbalPitchControl pitch_control = GimbalPitchControl::MotorMit;
    /** ImuTorque 模式专用参数；MotorMit 模式忽略。 */
    Struct_Gimbal_PitchTorque_Config pitch_torque;
    Struct_Gimbal_Motor_Config yaw;
    Struct_Gimbal_Motor_Config pitch;
    GimbalGyroAxis yaw_gyro_axis = GimbalGyroAxis::Z;
    GimbalGyroAxis pitch_gyro_axis = GimbalGyroAxis::Y;
    float yaw_gyro_sign = 1.0f;
    float pitch_gyro_sign = 1.0f;
    // basic_framework 角度比例增益与 500 deg/s 上限，内部统一弧度。
    float yaw_angle_kp = 8.0f;
    float yaw_speed_limit = 8.72664626f;
    // 待实机整定：转矩环不是 QD4310 电流环，默认不产生 Yaw 主动转矩。
    float yaw_speed_kp = 0.0f;
    float yaw_speed_ki = 0.0f;
    float yaw_speed_kd = 0.0f;
    float yaw_integral_limit = 0.0f;
    float yaw_torque_limit = 18.0f;
    // Meta 小米 Pitch 的 MIT 控制示例，增益及机械限位必须重新核对。
    float pitch_kp = 20.0f;
    float pitch_kd = 1.0f;
    float pitch_min = -1.5f;
    float pitch_max = 0.5f;
    float pitch_speed_limit = 1.0f;
    float pitch_motor_per_imu = 1.0f;
};

inline Struct_Gimbal_Config Gimbal_Default_Config()
{
    Struct_Gimbal_Config config;
    config.yaw.bus = BoardConfig_Get().gimbal_yaw_bus;
    config.pitch.bus = BoardConfig_Get().gimbal_pitch_bus;
    config.pitch.id = 2;
    config.pitch.feedback_id = 0x102;
    return config;
}

#endif

/**
 * @file Pitch.cpp
 * @brief Pitch 轴应用实现（老步兵云台板配置）。
 * @details 控制律与老步兵云台板工程（H7_RM）的 `Application/Pitch` 逐项一致：
 *          位置环 PID、目标速度前馈、IMU 角速度阻尼、连续 Stribeck 摩擦补偿和
 *          低带宽扰动估计的系数与计算顺序均未改动，只有底层 CAN / 时间戳接口换成
 *          本工程的 Device 与 BSP 实现。
 * @author Kylin-6（原始实现）/ H7_BSP 移植
 */

#include "Pitch.h"

#include "alg_basic.h"
#include "alg_filter_iir.h"
#include "alg_pid.h"
#include "alg_slope.h"
#include "alg_trajectory.h"
#include "dvc_dm_imu.h"
#include "fdcan.h"
#include "stm32h7xx_hal.h"

#include <cmath>

namespace
{
/** 角度单位换算。 */
constexpr float kDegreeToRadian = 0.0174532925f;
/** 控制周期，与 1 kHz 控制任务一致。 */
constexpr float kControlPeriodS = 0.001f;
/** DM-IMU 数据帧标称周期，用于差分求角速度。 */
constexpr float kImuSamplePeriodS = 0.001f;

/** 使能流程状态：失能、等待使能延迟、已使能。 */
enum class EnableState : uint8_t
{
    DISABLED,
    ARMING,
    ENABLED,
};

Class_DMMotor pitch_motor;
Class_PID pitch_position_pid;

/*
 * 目标规划与滤波均复用框架算法库：
 * - 手工目标路径：Class_Trajectory 三阶在线 S 曲线（限速度/加速度/加加速度）；
 * - 遥控目标路径：Class_Slope 斜率限幅；
 * - 电机反馈速度：Class_Filter_IIR_First_Order 一阶低通。
 * 唯一保留手写的是 IMU 差分角速度低通：其采样间隔随丢帧数变化，
 * 框架一阶 IIR 假定固定采样率，不适用（见 Pitch_Update 内注释）。
 */
Class_Trajectory pitch_trajectory;
Class_Slope pitch_remote_slope;
Class_Filter_IIR_First_Order pitch_motor_velocity_filter;

/**
 * @brief Pitch 位置环 PID 参数。
 *
 * 输入为目标 / 当前角度 (rad)，输出为 MIT 前馈力矩 (N·m)。
 * D_T 与 1 kHz 控制任务周期一致；Out_Max 限制外环最大力矩。
 */
PID_InitTypeDef pitch_position_pid_init = {
    .K_P = 0.42f,                    // 位置刚度：提高低摩擦机构的闭环跟随带宽
    .K_I = 0.0f,                     // 积分系数，当前不使用积分环节
    .K_D = 0.0f,                     // 微分系数，当前不使用微分环节
    .K_F = 0.0f,                     // 前馈系数，当前不使用前馈补偿
    .I_Out_Max = 0.0f,               // 积分输出限幅，未启用积分时无效
    .Out_Max = PITCH_MAX_TORQUE_NM,  // PID 总输出限幅，单位 N·m
    .D_T = 0.001f,                   // PID 计算周期，单位 s（1 kHz）
    .Dead_Zone = 0.0f,               // 误差死区，单位 rad
    .I_Variable_Speed_A = 0.0f,      // 变速积分阈值 A，当前不使用
    .I_Variable_Speed_B = 0.0f,      // 变速积分阈值 B，当前不使用
    .I_Separate_Threshold = 0.0f,    // 积分分离误差阈值，当前不使用
    .D_First = PID_D_First_DISABLE}; // 是否启用微分先行，当前关闭

bool pitch_initialized;
bool pitch_manual_target;
float pitch_manual_target_angle;
float pitch_target_angle;
bool pitch_trajectory_initialized;
bool pitch_remote_target_initialized;
float pitch_output_torque;
float pitch_motor_velocity_filtered;
bool pitch_imu_velocity_initialized;
uint32_t pitch_imu_last_sequence;
float pitch_imu_last_angle;
float pitch_imu_velocity_filtered;
float pitch_disturbance_torque;
EnableState pitch_enable_state;
uint32_t pitch_enable_arm_tick;

/** 一阶低通截止频率换算：tau = 1 / (2*pi*fc)。 */
constexpr float kMotorVelocityFilterCutoffHz =
    1.0f / (6.283185307179586f * PITCH_VELOCITY_FILTER_TAU_S);
/** 采样频率，与 1 kHz 控制任务一致。 */
constexpr float kSamplingFrequencyHz = 1.0f / kControlPeriodS;

/**
 * @brief 推进使能状态；ARMING 期间等到 PITCH_ENABLE_DELAY_MS 后下发使能帧。
 * @return true 表示当前允许下发控制指令。
 */
bool UpdateEnableState(bool enabled)
{
    switch (pitch_enable_state)
    {
    case EnableState::DISABLED:
        if (enabled)
        {
            /* 每次使能前重新计时，保证指令恢复后不会立即带力矩启动。 */
            pitch_enable_arm_tick = HAL_GetTick();
            pitch_enable_state = EnableState::ARMING;
        }
        break;

    case EnableState::ARMING:
        if (!enabled)
        {
            pitch_enable_state = EnableState::DISABLED;
        }
        else if ((HAL_GetTick() - pitch_enable_arm_tick) >= PITCH_ENABLE_DELAY_MS)
        {
            /* 插入队列可能暂满；只有使能帧成功入队后才允许开始 MIT 输出。 */
            if (pitch_motor.Enable())
            {
                pitch_enable_state = EnableState::ENABLED;
            }
        }
        break;

    case EnableState::ENABLED:
    default:
        if (!enabled)
        {
            pitch_motor.Disable();
            pitch_enable_state = EnableState::DISABLED;
        }
        break;
    }

    return pitch_enable_state == EnableState::ENABLED;
}

} // namespace

bool Pitch_Init(void)
{
    if (pitch_initialized)
    {
        return true;
    }

    pitch_manual_target = false;
    pitch_manual_target_angle = 0.0f;
    pitch_target_angle = 0.0f;
    pitch_trajectory_initialized = false;
    pitch_remote_target_initialized = false;
    pitch_output_torque = 0.0f;
    pitch_motor_velocity_filtered = 0.0f;
    pitch_imu_velocity_initialized = false;
    pitch_imu_last_sequence = 0U;
    pitch_imu_last_angle = 0.0f;
    pitch_imu_velocity_filtered = 0.0f;
    pitch_disturbance_torque = 0.0f;
    pitch_enable_state = EnableState::DISABLED;
    pitch_enable_arm_tick = 0U;

    /* 目标规划与滤波组件：常量均为编译期正值，Init 不会失败。 */
    (void)pitch_trajectory.Init(PITCH_TRAJECTORY_MAX_VELOCITY_RAD_S,
                                PITCH_TRAJECTORY_MAX_ACCEL_RAD_S2,
                                PITCH_TRAJECTORY_MAX_JERK_RAD_S3,
                                kControlPeriodS);
    {
        const float max_target_step =
            PITCH_REMOTE_TARGET_VELOCITY_MAX_RAD_S * kControlPeriodS;
        pitch_remote_slope.Init(max_target_step, max_target_step,
                                Slope_First_TARGET);
    }
    pitch_motor_velocity_filter.Init(kMotorVelocityFilterCutoffHz,
                                     kSamplingFrequencyHz);

    /* DM-IMU 是本轴唯一的角度反馈来源；注册失败时仍允许初始化，由上层判故障。 */
    const bool imu_ok = DM_IMU_Init(&hfdcan3,
                                    DM_IMU_DEFAULT_CAN_ID,
                                    DM_IMU_DEFAULT_MST_ID);

    const bool motor_ok = pitch_motor.Init(&hfdcan1,
                                           PITCH_MOTOR_ID,
                                           PITCH_MOTOR_FEEDBACK_ID,
                                           Enum_DMMotor_Mode::MIT,
                                           false,
                                           PITCH_MOTOR_P_MAX_RAD,
                                           PITCH_MOTOR_V_MAX_RAD_S,
                                           PITCH_MOTOR_T_MAX_NM);

    pitch_position_pid.Init(pitch_position_pid_init.K_P,
                            pitch_position_pid_init.K_I,
                            pitch_position_pid_init.K_D,
                            pitch_position_pid_init.K_F,
                            pitch_position_pid_init.I_Out_Max,
                            pitch_position_pid_init.Out_Max,
                            pitch_position_pid_init.D_T,
                            pitch_position_pid_init.Dead_Zone,
                            pitch_position_pid_init.I_Variable_Speed_A,
                            pitch_position_pid_init.I_Variable_Speed_B,
                            pitch_position_pid_init.I_Separate_Threshold,
                            pitch_position_pid_init.D_First);

    pitch_initialized = imu_ok && motor_ok;
    return pitch_initialized;
}

void Pitch_SetTargetAngle(float angle_rad)
{
    pitch_manual_target_angle =
        Basic_Math_Constrain(angle_rad, PITCH_TARGET_MIN_RAD, PITCH_TARGET_MAX_RAD);
    pitch_manual_target = true;
}

void Pitch_ClearTargetAngle(void)
{
    pitch_manual_target = false;
    pitch_trajectory_initialized = false;
    pitch_remote_target_initialized = false;
}

void Pitch_Update(float requested_target_rad, bool target_valid, bool enabled)
{
    if (!pitch_initialized)
    {
        return;
    }

    /* 控制环每 1 ms 请求一次欧拉角；本周期使用最近收到的一帧。 */
    DM_IMU_RequestEuler();

    /* 没有有效的陀螺仪角度时禁止使能和运动（与云台板原实现一致）。 */
    float imu_pitch_deg;
    uint32_t imu_pitch_sequence;
    if (!DM_IMU_GetPitchSample(&imu_pitch_deg, &imu_pitch_sequence))
    {
        /* 运行中姿态链路丢失时立即撤销使能，恢复后重新等待安全延时。 */
        if (pitch_enable_state == EnableState::ENABLED)
        {
            pitch_motor.Disable();
        }
        pitch_enable_state = EnableState::DISABLED;
        pitch_remote_target_initialized = false;
        pitch_trajectory_initialized = false;
        pitch_imu_velocity_initialized = false;
        return;
    }

    /* 使能延迟在收到有效 DM-IMU 数据后才开始计时。 */
    if (!UpdateEnableState(enabled))
    {
        return;
    }

    if (pitch_manual_target)
    {
        requested_target_rad = pitch_manual_target_angle;
    }
    else if (!target_valid)
    {
        /* 尚未收到上层目标，保持不动。 */
        return;
    }

    requested_target_rad =
        Basic_Math_Constrain(requested_target_rad, PITCH_TARGET_MIN_RAD,
                             PITCH_TARGET_MAX_RAD);

    const float imu_pitch_rad = imu_pitch_deg * kDegreeToRadian;
    float target_rad;
    float target_velocity_rad_s;
    if (pitch_manual_target)
    {
        /* 手工目标路径：框架三阶 S 曲线，限速度/加速度/加加速度，从当前
         * IMU 角度起步（路径切换时由 _initialized 标志触发 Reset）。 */
        if (!pitch_trajectory_initialized)
        {
            pitch_trajectory.Reset(imu_pitch_rad);
            pitch_trajectory_initialized = true;
        }
        pitch_trajectory.Set_Target_Position(requested_target_rad);
        pitch_trajectory.TIM_Calculate_PeriodElapsedCallback();
        target_rad = pitch_trajectory.Get_Position();
        target_velocity_rad_s = pitch_trajectory.Get_Velocity();
        /* 两条路径互斥：进入手工路径即作废斜坡状态。 */
    }
    else
    {
        /* 遥控目标路径：框架斜坡限幅，从当前 IMU 角度起步。
         * 每周期把 Now_Real 对齐到上一周期输出，保持纯斜坡语义；
         * 前馈速度 = 本周期实际步长 / 周期，与原实现逐项一致。 */
        if (!pitch_remote_target_initialized)
        {
            pitch_remote_slope.Reset(imu_pitch_rad);
            pitch_remote_target_initialized = true;
        }
        const float previous_target = pitch_remote_slope.Get_Out();
        pitch_remote_slope.Set_Now_Real(previous_target);
        pitch_remote_slope.Set_Target(requested_target_rad);
        pitch_remote_slope.TIM_Calculate_PeriodElapsedCallback();
        target_rad = pitch_remote_slope.Get_Out();
        target_velocity_rad_s =
            (target_rad - previous_target) / kControlPeriodS;
        /* 两条路径互斥：进入遥控路径即作废 S 曲线状态。 */
        pitch_trajectory_initialized = false;
    }

    /* 保存规划后的目标，供遥测 / 上层读取。 */
    pitch_target_angle = target_rad;

    const float position_error_rad = target_rad - imu_pitch_rad;

    if (!pitch_imu_velocity_initialized)
    {
        pitch_imu_last_sequence = imu_pitch_sequence;
        pitch_imu_last_angle = imu_pitch_rad;
        pitch_imu_velocity_initialized = true;
    }
    else if (imu_pitch_sequence != pitch_imu_last_sequence)
    {
        const uint32_t sample_count = imu_pitch_sequence - pitch_imu_last_sequence;
        const float sample_period = kImuSamplePeriodS * static_cast<float>(sample_count);
        const float imu_velocity =
            Basic_Math_Constrain((imu_pitch_rad - pitch_imu_last_angle) / sample_period,
                                 -PITCH_IMU_VELOCITY_MAX_RAD_S,
                                 PITCH_IMU_VELOCITY_MAX_RAD_S);
        /* 框架 Class_Filter_IIR_First_Order 假定固定采样率（Init 时定死 alpha）；
         * 此处采样间隔随 DM-IMU 丢帧数变化，需按实际间隔重算 alpha，故保留手写。 */
        const float imu_velocity_alpha =
            sample_period / (PITCH_IMU_VELOCITY_FILTER_TAU_S + sample_period);
        pitch_imu_velocity_filtered +=
            imu_velocity_alpha * (imu_velocity - pitch_imu_velocity_filtered);
        pitch_imu_last_sequence = imu_pitch_sequence;
        pitch_imu_last_angle = imu_pitch_rad;
    }

    /* 电机速度只做滤波与遥测，不参与控制（阻尼系数为 0）。
     * 框架一阶 IIR：截止频率由 tau 换算，首帧自动对齐输入（原实现从 0 收敛，
     * 电机使能前速度为 0，两者稳态一致，框架版无启动爬升）。 */
    pitch_motor_velocity_filter.Set_Now(pitch_motor.feedback.velocity);
    pitch_motor_velocity_filter.TIM_Calculate_PeriodElapsedCallback();
    pitch_motor_velocity_filtered = pitch_motor_velocity_filter.Get_Out();

    /* 位置环反馈使用 DM-IMU pitch，不使用电机单圈编码器角度。 */
    pitch_position_pid.Set_Target(target_rad);
    pitch_position_pid.Set_Now(imu_pitch_rad);
    pitch_position_pid.TIM_Calculate_PeriodElapsedCallback();

    const float abs_imu_speed_rad_s = std::fabs(pitch_imu_velocity_filtered);

    /* 目标速度小前馈改善跟随，方向明确的 IMU 角速度负反馈提供阻尼。 */
    const float target_velocity_gain =
        pitch_manual_target
            ? PITCH_VELOCITY_DAMPING
            : (target_velocity_rad_s >= 0.0f ? PITCH_POSITIVE_VELOCITY_FEEDFORWARD
                                             : PITCH_NEGATIVE_VELOCITY_FEEDFORWARD);
    pitch_output_torque = pitch_position_pid.Get_Out() +
                          target_velocity_gain * target_velocity_rad_s +
                          -PITCH_IMU_VELOCITY_DAMPING * pitch_imu_velocity_filtered;

    /* 连续 Stribeck 摩擦补偿：低速接近静摩擦，高速平滑过渡到库仑摩擦。 */
    const float stribeck_speed_ratio =
        abs_imu_speed_rad_s / PITCH_STRIBECK_VELOCITY_RAD_S;
    const float friction_direction_input =
        target_velocity_rad_s + PITCH_STRIBECK_ERROR_GAIN * position_error_rad;
    const float static_friction_torque = friction_direction_input >= 0.0f
                                             ? PITCH_POSITIVE_STATIC_FRICTION_TORQUE_NM
                                             : PITCH_NEGATIVE_STATIC_FRICTION_TORQUE_NM;
    const float coulomb_friction_torque = friction_direction_input >= 0.0f
                                              ? PITCH_POSITIVE_COULOMB_FRICTION_TORQUE_NM
                                              : PITCH_NEGATIVE_COULOMB_FRICTION_TORQUE_NM;
    const float friction_magnitude =
        coulomb_friction_torque +
        (static_friction_torque - coulomb_friction_torque) *
            std::exp(-(stribeck_speed_ratio * stribeck_speed_ratio));
    pitch_output_torque +=
        friction_magnitude *
        std::tanh(friction_direction_input / PITCH_STRIBECK_DIRECTION_SMOOTH_RAD_S);

    /* 低带宽扰动估计：只在接近静止时学习未建模的重力 / 负载力矩。 */
    if (std::fabs(target_velocity_rad_s) < PITCH_DISTURBANCE_TARGET_SPEED_RAD_S &&
        abs_imu_speed_rad_s < PITCH_DISTURBANCE_ACTUAL_SPEED_RAD_S)
    {
        pitch_disturbance_torque +=
            PITCH_DISTURBANCE_INTEGRAL_GAIN * position_error_rad * kControlPeriodS;
        pitch_disturbance_torque =
            Basic_Math_Constrain(pitch_disturbance_torque,
                                 -PITCH_DISTURBANCE_TORQUE_MAX_NM,
                                 PITCH_DISTURBANCE_TORQUE_MAX_NM);
    }
    else
    {
        pitch_disturbance_torque -=
            pitch_disturbance_torque * kControlPeriodS / PITCH_DISTURBANCE_DECAY_TAU_S;
    }
    pitch_output_torque += pitch_disturbance_torque;

    pitch_output_torque = Basic_Math_Constrain(pitch_output_torque,
                                               -PITCH_MAX_TORQUE_NM,
                                               PITCH_MAX_TORQUE_NM);

    /* 不使用电机内部位置环：外环基于 DM-IMU，通过 t_ff 下发力矩。
     * 云台板的 Pitch 电机方向与 IMU pitch 正方向相反。 */
    pitch_motor.SetMIT(0.0f, 0.0f, 0.0f, 0.0f, -pitch_output_torque);
}

float Pitch_GetTargetAngle(void)
{
    return pitch_target_angle;
}

bool Pitch_GetImuPitchDeg(float *pitch_deg)
{
    return DM_IMU_GetPitch(pitch_deg);
}

float Pitch_GetOutputTorque(void)
{
    return pitch_output_torque;
}

float Pitch_GetDisturbanceTorque(void)
{
    return pitch_disturbance_torque;
}

float Pitch_GetPositionRad(void)
{
    return pitch_motor.feedback.position;
}

uint16_t Pitch_GetEncoder(void)
{
    /* Class_DMMotor 不暴露原始值，按位置解码公式逆运算还原 0~65535 编码器值。 */
    return static_cast<uint16_t>(
        Basic_Math_Float_To_Int(pitch_motor.feedback.position,
                                -PITCH_MOTOR_P_MAX_RAD,
                                PITCH_MOTOR_P_MAX_RAD,
                                0,
                                0xFFFF));
}

bool Pitch_IsEnabled(void)
{
    return pitch_enable_state == EnableState::ENABLED;
}

bool Pitch_IsImuValid(void)
{
    float pitch_deg;
    return DM_IMU_GetPitch(&pitch_deg);
}

float Pitch_GetImuVelocityRadS(void)
{
    return pitch_imu_velocity_filtered;
}

Class_DMMotor *Pitch_GetMotor(void)
{
    return &pitch_motor;
}

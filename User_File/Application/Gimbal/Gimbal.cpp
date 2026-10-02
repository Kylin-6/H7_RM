/**
 * @file Gimbal.cpp
 * @brief 老步兵单 Pitch 云台：框架消息入口、私有设备所有权和 IMU 力矩闭环。
 * @details Yaw 由底盘板主控，本板不注册 Yaw。Pitch 电机编码器用于健康判断，
 *          角度/角速度来自统一 INS Topic；DM-IMU 的适配与换算由设备桥完成。
 */
#include "Gimbal.h"
#include "message_center.h"
#if GIMBAL
#include "alg_filter_iir.h"
#include "alg_pid.h"
#include "alg_slope.h"
#include "dmmotor.h"
#include "stm32h7xx_hal.h"
#endif
#include <cmath>

namespace
{
constexpr float CONTROL_PERIOD_S = 0.001f;
struct GimbalContext
{
    INS_State ins{};
    bool ins_valid = false;
    uint8_t feedback_divider = 0U;
#if GIMBAL
    Struct_Gimbal_Config config{};
    Class_DMMotor pitch_motor;
    Struct_DMMotor_Snapshot motor_snapshot{};
    Class_PID position_pid;
    Class_Slope target_slope;
    Class_Filter_IIR_First_Order velocity_filter;
    GimbalCmd command{};
    bool initialized = false;
    bool motor_registered = false;
    bool arming = false;
    bool control_started = false;
    bool path_initialized = false;
    uint32_t arming_start_ms = 0U;
    float disturbance_torque_nm = 0.0f;
#endif
};
GimbalContext ctx;

#if GIMBAL
float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool ConfigValid(const Struct_Gimbal_Config& config)
{
    const auto& c = config.pitch_torque;
    const float positive[] = {c.position_kp, c.target_rate_rad_s,
                              c.stribeck_velocity_rad_s, c.stribeck_smooth_rad_s, c.disturbance_decay_tau_s,
                              c.torque_limit_nm};
    for (float value : positive)
    {
        if (!std::isfinite(value) || value <= 0.0f)
        {
            return false;
        }
    }
    const float nonnegative[] = {c.imu_velocity_filter_tau_s,
                                 c.ff_velocity_positive, c.ff_velocity_negative, c.imu_velocity_damping,
                                 c.static_friction_positive_nm, c.static_friction_negative_nm,
                                 c.coulomb_friction_positive_nm, c.coulomb_friction_negative_nm,
                                 c.stribeck_error_gain, c.disturbance_integral_gain, c.disturbance_max_nm,
                                 c.disturbance_target_speed_rad_s, c.disturbance_actual_speed_rad_s};
    for (float value : nonnegative)
    {
        if (!std::isfinite(value) || value < 0.0f)
        {
            return false;
        }
    }
    return (c.torque_sign == 1.0f || c.torque_sign == -1.0f) &&
           c.torque_limit_nm <= config.pitch.torque_max && config.ins_max_age_us > 0U &&
           std::isfinite(config.pitch_min) && std::isfinite(config.pitch_max) &&
           config.pitch_min < config.pitch_max;
}

void Stop()
{
    // 驱动处理边沿安全目标、失能帧及失败补交；重复请求不刷总线。
    if (ctx.motor_registered)
    {
        (void) ctx.pitch_motor.RequestEnabled(false);
    }
    ctx.arming = false;
    ctx.control_started = false;
    ctx.path_initialized = false;
    ctx.disturbance_torque_nm = 0.0f;
}

bool ControlPermitted()
{
    // 单轴 LOCK 沿用老步兵安全语义：失能；只有显式 IMU 目标允许输出。
    return ctx.initialized && ctx.command.mode == GimbalMode::IMU &&
           std::isfinite(ctx.command.pitch_angle_rad) && ctx.ins_valid &&
           std::isfinite(ctx.ins.pitch_rad) && std::isfinite(ctx.ins.gyro_y_rad_s) &&
           !ctx.motor_snapshot.fault;
}

void Control()
{
    const auto &c = ctx.config.pitch_torque;
    if (!ctx.path_initialized)
    {
        // 恢复时从当前姿态起步，清除外环历史与旧扰动估计，目标通过限速斜坡接入。
        ctx.position_pid = Class_PID{};
        ctx.position_pid.Init(c.position_kp, 0, 0, 0, 0, c.torque_limit_nm);
        ctx.target_slope.Reset(ctx.ins.pitch_rad);
        ctx.velocity_filter = Class_Filter_IIR_First_Order{};
        if (c.imu_velocity_filter_tau_s > 0.0f)
        {
            const float cutoff_hz = 1.0f / (6.283185307179586f * c.imu_velocity_filter_tau_s);
            ctx.velocity_filter.Init(cutoff_hz, 1.0f / CONTROL_PERIOD_S);
        }
        ctx.path_initialized = true;
    }
    const float previous = ctx.target_slope.Get_Out();
    ctx.target_slope.Set_Now_Real(previous);
    ctx.target_slope.Set_Target(Clamp(ctx.command.pitch_angle_rad,
                                      ctx.config.pitch_min, ctx.config.pitch_max));
    ctx.target_slope.TIM_Calculate_PeriodElapsedCallback();
    const float target_rad = ctx.target_slope.Get_Out();
    const float target_velocity_rad_s = (target_rad - previous) / CONTROL_PERIOD_S;
    const float error_rad = target_rad - ctx.ins.pitch_rad;
    float velocity_rad_s = ctx.ins.gyro_y_rad_s;
    if (c.imu_velocity_filter_tau_s > 0.0f)
    {
        ctx.velocity_filter.Set_Now(velocity_rad_s);
        ctx.velocity_filter.TIM_Calculate_PeriodElapsedCallback();
        velocity_rad_s = ctx.velocity_filter.Get_Out();
    }
    ctx.position_pid.Set_Target(target_rad);
    ctx.position_pid.Set_Now(ctx.ins.pitch_rad);
    ctx.position_pid.TIM_Calculate_PeriodElapsedCallback();

    // 原实车控制律：位置环 + 不对称目标速度前馈 - IMU 速度阻尼。
    const float ff_gain = target_velocity_rad_s >= 0.0f
                              ? c.ff_velocity_positive
                              : c.ff_velocity_negative;
    float torque_nm = ctx.position_pid.Get_Out() + ff_gain * target_velocity_rad_s -
                      c.imu_velocity_damping * velocity_rad_s;
    const float speed_ratio = std::fabs(velocity_rad_s) / c.stribeck_velocity_rad_s;
    const float direction = target_velocity_rad_s + c.stribeck_error_gain * error_rad;
    const float static_friction = direction >= 0.0f
                                      ? c.static_friction_positive_nm
                                      : c.static_friction_negative_nm;
    const float coulomb_friction = direction >= 0.0f
                                       ? c.coulomb_friction_positive_nm
                                       : c.coulomb_friction_negative_nm;
    torque_nm += (coulomb_friction + (static_friction - coulomb_friction) *
                                         std::exp(-(speed_ratio * speed_ratio))) *
                 std::tanh(direction / c.stribeck_smooth_rad_s);
    // 静止时学习重力/负载；运动时衰减，故障恢复时不重用旧补偿。
    if (std::fabs(target_velocity_rad_s) < c.disturbance_target_speed_rad_s &&
        std::fabs(velocity_rad_s) < c.disturbance_actual_speed_rad_s)
    {
        ctx.disturbance_torque_nm = Clamp(ctx.disturbance_torque_nm +
                                              c.disturbance_integral_gain * error_rad * CONTROL_PERIOD_S,
                                          -c.disturbance_max_nm, c.disturbance_max_nm);
    }
    else
    {
        ctx.disturbance_torque_nm -= ctx.disturbance_torque_nm *
                                     CONTROL_PERIOD_S / c.disturbance_decay_tau_s;
    }
    torque_nm = Clamp(torque_nm + ctx.disturbance_torque_nm,
                      -c.torque_limit_nm, c.torque_limit_nm);
    // kp/kd 恒为 0：电机侧只执行 t_ff，位置闭环由本板 IMU 外环完成。
    // 发布失败由下一 1 ms 周期提交最新目标，不阻塞等待 CAN。
    (void) ctx.pitch_motor.SetMIT(0.0f, 0.0f, 0.0f, 0.0f, c.torque_sign * torque_nm);
}
#endif

void PublishFeedback()
{
    if (++ctx.feedback_divider < 10U)
    {
        return;
    }
    ctx.feedback_divider = 0U;
    GimbalFeedback feedback{};
    feedback.ins_valid = ctx.ins_valid;
    if (ctx.ins_valid)
    {
        // Yaw 仅是姿态观测值，不代表本板拥有 Yaw 电机。
        feedback.yaw_rad = ctx.ins.yaw_rad;
        feedback.pitch_rad = ctx.ins.pitch_rad;
        feedback.yaw_speed_rad_s = ctx.ins.gyro_z_rad_s;
        feedback.pitch_speed_rad_s = ctx.ins.gyro_y_rad_s;
    }
#if GIMBAL
    feedback.enabled = ControlPermitted() && ctx.motor_snapshot.ready;
#endif
    MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
}
}

#if GIMBAL
bool Gimbal_Init(const Struct_Gimbal_Config& config)
{
    if (!ConfigValid(config))
    {
        return false;
    }
    ctx.config = config;
    ctx.motor_registered = ctx.pitch_motor.Init(config.pitch.bus, config.pitch.id,
                                                config.pitch.feedback_id, Enum_DMMotor_Mode::MIT, config.pitch.reverse,
                                                config.pitch.position_max, config.pitch.velocity_max, config.pitch.torque_max);
    ctx.initialized = ctx.motor_registered;
    const float max_step = config.pitch_torque.target_rate_rad_s * CONTROL_PERIOD_S;
    ctx.target_slope.Init(max_step, max_step, Slope_First_TARGET);
    return ctx.initialized;
}

Enum_Gimbal_Status Gimbal_GetStatus(void)
{
    if (!ctx.initialized) { return Gimbal_Status_CONFIG_ERROR; }
    if (ctx.command.mode != GimbalMode::IMU)
    {
        return Gimbal_Status_DISABLE;
    }
    if (!ControlPermitted())
    {
        return Gimbal_Status_FAULT;
    }
    return ctx.motor_snapshot.ready ? Gimbal_Status_READY : Gimbal_Status_ENABLING;
}
#endif

void Gimbal_Update(void)
{
#if GIMBAL
    ctx.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ctx.ins, ctx.config.ins_max_age_us);
    const auto message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.command = message.valid ? message.data : GimbalCmd{};
    ctx.motor_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
    // 启动时不能以电机在线作为发送 Enable 的前提：失能电机可能不主动反馈。
    // 已进入闭环后丢失反馈仍立即停机，随后按同样的延迟重新请求使能。
    if (!ControlPermitted() || (ctx.control_started && !ctx.motor_snapshot.online))
    {
        Stop();
    }
    else
    {
        if (!ctx.arming)
        {
            ctx.arming_start_ms = HAL_GetTick();
            ctx.arming = true;
        }
        // 每次健康许可恢复重新等待；使能帧与 MIT 输出均不在等待期发送。
        if (HAL_GetTick() - ctx.arming_start_ms >= ctx.config.pitch_torque.enable_delay_ms)
        {
            (void) ctx.pitch_motor.RequestEnabled(true);
            ctx.motor_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
            if (ctx.motor_snapshot.ready)
            {
                ctx.control_started = true;
                Control();
            }
            else
            {
                ctx.path_initialized = false;
            }
        }
    }
    ctx.motor_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
#else
    ctx.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ctx.ins, 100000U);
#endif
    PublishFeedback();
}

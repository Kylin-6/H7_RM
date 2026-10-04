/**
 * @file Gimbal.cpp
 * @brief 老步兵单 Pitch 云台：框架消息入口、私有设备所有权和 IMU 力矩闭环。
 * @details Yaw 由底盘板主控，本板不注册 Yaw。Pitch 电机编码器用于健康判断，
 *          角度/角速度来自统一 INS Topic；DM-IMU 的适配与换算由设备桥完成。
 *          应用按构建期源码选择编入 GimbalBoard（H7_APP_GIMBAL），文件内不再
 *          保留功能条件编译；关闭时不编译、不调度、不发布云台反馈。
 */
#include "Gimbal.h"
#include "message_center.h"

#include "alg_filter_iir.h"
#include "alg_pid.h"
#include "alg_slope.h"
#include "dmmotor.h"
#include "stm32h7xx_hal.h"

#include <cmath>

namespace
{
constexpr float CONTROL_PERIOD_S = 0.001f;

struct GimbalContext
{
    INS_State ins{};
    bool ins_valid = false;
    uint8_t feedback_divider = 0U;
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
};

GimbalContext ctx;

/**
 * @brief 将数值限制在闭区间 [minimum, maximum] 内。
 * @note 调用方保证上下界有序、输入有限；不承担参数校验。
 */
float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

/**
 * @brief 校验 IMU 力矩控制参数与机械限位。
 * @return 应用层约束全部满足时返回 true；总线、协议量程等由设备 Init 继续校验。
 */
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

/**
 * @brief 对已注册的 Pitch 电机请求失能，并复位武装与控制路径状态。
 * @note 驱动处理边沿安全目标、失能帧及失败补交；重复请求不刷总线。
 */
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

/**
 * @brief 推导当前周期是否允许主动输出。
 * @note 单轴 LOCK 沿用老步兵安全语义：失能；只有显式 IMU 目标允许输出。
 */
bool ControlPermitted()
{
    // 单轴 LOCK 沿用老步兵安全语义：失能；只有显式 IMU 目标允许输出。
    return ctx.initialized && ctx.command.mode == GimbalMode::IMU &&
           std::isfinite(ctx.command.pitch_angle_rad) && ctx.ins_valid &&
           std::isfinite(ctx.ins.pitch_rad) && std::isfinite(ctx.ins.gyro_y_rad_s) &&
           !ctx.motor_snapshot.fault;
}

/**
 * @brief 计算本周期 Pitch 力矩并以 MIT 纯前馈模式提交。
 * @note 控制律：位置环 + 不对称目标速度前馈 - IMU 速度阻尼 + Stribeck 摩擦补偿
 *       + 低带宽扰动估计；仅在功能许可且电机 ready 后调用。
 */
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

/**
 * @brief 每调用十次发布一次云台反馈；1 kHz 更新入口下对应 100 Hz。
 * @note 姿态来自 INS，INS 无效时姿态/速度为零；enabled 表示功能获许可且电机 ready。
 */
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
    feedback.enabled = ControlPermitted() && ctx.motor_snapshot.ready;
    MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
}
} // namespace

/**
 * @brief 启动阶段校验并复制配置、注册 Pitch 电机并初始化目标斜坡。
 * @param config 参数配置，复制后调用方无需保留其对象。
 * @return 配置有效且电机注册成功时返回 true；失败后更新入口禁止正常控制。
 * @note 同一 ControlTask 启动时仅调用一次；不等待反馈、不使能、不置零，
 *       也不修改电机端模式或持久化参数。
 */
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

/**
 * @brief 汇总云台启动、许可与电机状态，供诊断层只读采集。
 * @note 仅供同一 ControlTask 上下文的 Diagnostics 调用，不赋予设备控制权限。
 */
Struct_Gimbal_Diagnostic Gimbal_GetDiagnostic()
{
    Struct_Gimbal_Diagnostic d{};
    const auto m = ctx.pitch_motor.GetFeedbackSnapshot();
    d.initialized = ctx.initialized;
    d.ins_valid = ctx.ins_valid && std::isfinite(ctx.ins.pitch_rad) &&
                  std::isfinite(ctx.ins.gyro_y_rad_s);
    d.permitted = ctx.command.mode == GimbalMode::IMU;
    d.waiting = d.permitted && !m.ready;
    d.pitch = {ctx.arming && HAL_GetTick() - ctx.arming_start_ms >=
                   ctx.config.pitch_torque.enable_delay_ms,
               m.online, m.feedback.state > 1U, m.requested_enabled, m.ready};
    return d;
}

/**
 * @brief 根据最近一次更新留下的命令、INS 和电机快照推导应用状态。
 * @note 由同一任务上下文读取；不刷新设备快照、不执行控制或安排恢复。
 */
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

/**
 * @brief ControlTask 的 1 kHz 周期入口：读取快照、处理武装/许可并输出力矩。
 * @note 在 RobotCmd_Update 之后调用；禁用或 INS 无效时请求停机，
 *       设备故障和掉线输出由驱动保护。本入口不阻塞、不解析 CAN、不仲裁命令来源。
 */
void Gimbal_Update(void)
{
    // Fresh = 这份姿态是否可用于当前控制周期；设备 Online（liveness）由
    // 各 Device 内的 Daemon 判定，两个概念独立，不做重复的掉线计算。
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
    PublishFeedback();
}

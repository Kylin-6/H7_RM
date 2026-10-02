/**
 * @file Gimbal.cpp
 * @brief 云台应用：老步兵云台板路径与框架通用路径。
 * @details
 * - `LEGACY_INFANTRY_GIMBAL=1`（老步兵云台板）：Pitch 轴由 Application/Pitch 承担
 *   （DM MIT + DM-IMU），本文件负责模式门控、可选 Yaw 轴（QD4310 + INS，默认关闭）
 *   与云台反馈汇总。该路径保留合并前云台分支的实现，控制行为与实机验证一致。
 * - 其他板型：RoboMaster_H7 框架通用实现——配置驱动（Gimbal_Config.h）、双 DM 电机
 *   MIT 控制、命令序号门控与 ENABLING/FAULT 状态机。
 * 两条路径由编译期宏互斥选择。
 */

#include "Gimbal.h"
#include "message_center.h"
#include <cmath>

#if GIMBAL && LEGACY_INFANTRY_GIMBAL

/* ==========================================================================
 * 老步兵云台板实现
 * ========================================================================== */

static INS_State Gimbal_INS_State;
static bool Gimbal_INS_Valid = false;
static constexpr uint64_t GIMBAL_INS_MAX_AGE_US = 10000U;
static GimbalCmd Gimbal_Command;
static Subscriber<GimbalCmd> Gimbal_Command_Subscriber(
    MessageCenter::Gimbal_Command_Topic);
static Publisher<GimbalFeedback> Gimbal_Feedback_Publisher(
    MessageCenter::Gimbal_Feedback_Topic);
static uint8_t Gimbal_Message_Divider;

static GimbalMode Gimbal_Last_Mode = GimbalMode::DISABLED;

#if GIMBAL

#include "QD4310.h"
#include "alg_pid.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include <cmath>

#if LEGACY_INFANTRY_GIMBAL

/* ==========================================================================
 * 老步兵云台板实现
 *
 * 云台板只有 Pitch 轴参与实际控制，由 Application/Pitch 承担（DM MIT + DM-IMU）。
 * 本文件负责：Pitch 的模式门控、Yaw 轴（默认关闭）和云台反馈汇总。
 * ========================================================================== */

#include "Pitch.h"

/** 弧度 / 度换算。 */
static constexpr float GIMBAL_DEG_TO_RAD = 0.0174532925f;

LegacyGimbal_t Gimbal;

/**
 * @brief Yaw 轴速度内环参数（云台板原工程数值）。
 *
 * 输入目标为角速度，反馈来自 INS 状态的 Z 轴角速度，输出作为 QD4310 电流指令。
 */
PID_InitTypeDef Legacy_Yaw_Speed_PID_Init = {
    .K_P = 0.15f,
    .K_I = 0.63f,
    .K_D = 0.000f,
    .K_F = 0.0f,
    .I_Out_Max = 1.2f,
    .Out_Max = 1.65f,
    .D_T = 0.001f,
    .Dead_Zone = 0.01f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief Yaw 轴角度外环参数（云台板原工程数值）。
 *
 * 输入目标为 Yaw 目标角度，反馈来自 INS 状态的欧拉角；输出交给速度内环。
 */
PID_InitTypeDef Legacy_Yaw_Angle_PID_Init = {
    .K_P = 32.00f,
    .K_I = 0.00f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 15.0f,
    .Out_Max = 50.0f,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief 初始化 Pitch 轴（必需）与 Yaw 轴（可选）。
 *
 * Pitch 轴只做设备与参数初始化，使能帧由 Pitch 内部延迟下发，因此这里不会阻塞。
 * 云台板当前 BMI088 硬件故障，Yaw 轴默认不初始化；需要时打开
 * `H7_LEGACY_INFANTRY_GIMBAL_YAW`，届时沿用原工程的有限重试使能流程。
 */
void Gimbal_Init(void)
{
    Gimbal.Gimbal_FSM.Init();
    Gimbal.Target_Yaw_Angle = 0.0f;
    Gimbal.Target_Yaw_Speed = 0.0f;

    Pitch_Init();

#if LEGACY_INFANTRY_GIMBAL_YAW
    QD4310_Init(&Gimbal.Yaw_Motor, YAW_ID, &hfdcan2);

    Gimbal.Yaw_Speed_PID.Init(Legacy_Yaw_Speed_PID_Init.K_P,
                             Legacy_Yaw_Speed_PID_Init.K_I,
                             Legacy_Yaw_Speed_PID_Init.K_D,
                             Legacy_Yaw_Speed_PID_Init.K_F,
                             Legacy_Yaw_Speed_PID_Init.I_Out_Max,
                             Legacy_Yaw_Speed_PID_Init.Out_Max,
                             Legacy_Yaw_Speed_PID_Init.D_T,
                             Legacy_Yaw_Speed_PID_Init.Dead_Zone,
                             Legacy_Yaw_Speed_PID_Init.I_Variable_Speed_A,
                             Legacy_Yaw_Speed_PID_Init.I_Variable_Speed_B,
                             Legacy_Yaw_Speed_PID_Init.I_Separate_Threshold,
                             Legacy_Yaw_Speed_PID_Init.D_First);

    Gimbal.Yaw_Angle_PID.Init(Legacy_Yaw_Angle_PID_Init.K_P,
                             Legacy_Yaw_Angle_PID_Init.K_I,
                             Legacy_Yaw_Angle_PID_Init.K_D,
                             Legacy_Yaw_Angle_PID_Init.K_F,
                             Legacy_Yaw_Angle_PID_Init.I_Out_Max,
                             Legacy_Yaw_Angle_PID_Init.Out_Max,
                             Legacy_Yaw_Angle_PID_Init.D_T,
                             Legacy_Yaw_Angle_PID_Init.Dead_Zone,
                             Legacy_Yaw_Angle_PID_Init.I_Variable_Speed_A,
                             Legacy_Yaw_Angle_PID_Init.I_Variable_Speed_B,
                             Legacy_Yaw_Angle_PID_Init.I_Separate_Threshold,
                             Legacy_Yaw_Angle_PID_Init.D_First);

    // 原工程在此处无限重试并使能 Yaw 轴；框架改为最多 2 s，避免阻塞其他 Application。
    constexpr uint32_t GIMBAL_ENABLE_RETRY_COUNT = 100U;
    constexpr uint32_t GIMBAL_ENABLE_RETRY_DELAY_MS = 20U;
    for (uint32_t retry = 0U; retry < GIMBAL_ENABLE_RETRY_COUNT; ++retry)
    {
        if (Gimbal.Yaw_Motor.enabled)
        {
            Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_READY);
            return;
        }

        Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_YAW_ERROR);
        QD4310_Enable(&Gimbal.Yaw_Motor);
        osDelay(GIMBAL_ENABLE_RETRY_DELAY_MS);
    }
#else
    /* Yaw 轴未启用：状态机直接标记为就绪，表示云台板 Pitch 轴可用。 */
    Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_READY);
#endif
}

/**
 * @brief 执行一次云台闭环计算。
 *
 * Pitch 轴由 Pitch_Update 内部完成 DM-IMU 位置环 + MIT 力矩下发，这里只处理 Yaw。
 * 原工程中云台板不调度本函数，Yaw 轴打开后才有实际输出。
 */
void Gimbal_Loop(void)
{
#if LEGACY_INFANTRY_GIMBAL_YAW
    if (!Gimbal_INS_Valid)
    {
        return;
    }

    // Yaw 角度外环：使用 INS 状态的 Yaw 欧拉角，计算速度内环目标。
    Gimbal.Yaw_Angle_PID.Set_Target(Gimbal.Target_Yaw_Angle);
    Gimbal.Yaw_Angle_PID.Set_Now(Gimbal_INS_State.yaw_rad);
    Gimbal.Yaw_Angle_PID.TIM_Calculate_PeriodElapsedCallback();
    Gimbal.Target_Yaw_Speed = Gimbal.Yaw_Angle_PID.Get_Out();

    // Yaw 速度内环使用 INS 状态的机体系 Z 轴角速度反馈。
    Gimbal.Yaw_Speed_PID.Set_Target(Gimbal.Target_Yaw_Speed);
    Gimbal.Yaw_Speed_PID.Set_Now(Gimbal_INS_State.gyro_z_rad_s);
    Gimbal.Yaw_Speed_PID.TIM_Calculate_PeriodElapsedCallback();

    // Yaw 采用电流控制：速度 PID 输出直接作为电机电流指令。
    QD4310_SetCurrent(&Gimbal.Yaw_Motor, Gimbal.Yaw_Speed_PID.Get_Out());
#endif
}

/**
 * @brief 云台命令处理：老步兵云台板的模式语义。
 *
 * - `IMU`     Pitch 跟随命令目标角（板间通道经 Communication 滤波映射后下发）；
 * - `LOCK`    Pitch 保持当前规划目标，不接收新目标；
 * - `DISABLED`两轴失能，Pitch 主动下发失能帧。
 *
 * 云台板没有 Yaw 目标输入，Yaw 始终锁在使能时刻的 INS 角度。
 */
static void Gimbal_HandleCommand(const GimbalCmd &command)
{
#if LEGACY_INFANTRY_GIMBAL_YAW
    if (command.mode == GimbalMode::DISABLED)
    {
        if (Gimbal_Last_Mode != GimbalMode::DISABLED)
        {
            QD4310_Disable(&Gimbal.Yaw_Motor);
        }
    }
    else
    {
        if (Gimbal_Last_Mode == GimbalMode::DISABLED)
        {
            QD4310_Enable(&Gimbal.Yaw_Motor);
            /* 使能瞬间锁住当前姿态，避免目标跳向零点。 */
            if (Gimbal_INS_Valid && std::isfinite(Gimbal_INS_State.yaw_rad))
            {
                Gimbal.Target_Yaw_Angle = Gimbal_INS_State.yaw_rad;
            }
        }
        else if (command.mode == GimbalMode::LOCK)
        {
            Gimbal.Target_Yaw_Angle = Gimbal_INS_Valid ? Gimbal_INS_State.yaw_rad
                                                       : Gimbal.Target_Yaw_Angle;
        }
    }
#else
    (void)command;
#endif
    Gimbal_Last_Mode = command.mode;
}

#endif /* LEGACY_INFANTRY_GIMBAL */
#endif /* GIMBAL */

void Gimbal_Update(void)
{
    /* 高频姿态走静态 Topic，云台无需感知底层具体使用哪一种 IMU。 */
    INS_State ins_state;
    if (MessageCenter::INS_State_Topic.ReadFresh(ins_state,
                                                  GIMBAL_INS_MAX_AGE_US))
    {
        Gimbal_INS_State = ins_state;
        Gimbal_INS_Valid = true;
    }
    else
    {
        Gimbal_INS_Valid = false;
#if GIMBAL && (!LEGACY_INFANTRY_GIMBAL || LEGACY_INFANTRY_GIMBAL_YAW)
        /* Yaw 轴存在时，INS 失联兜底置零电流；云台板 Yaw 关闭时不做——
         * Yaw_Motor 未初始化（hfdcan 为空），且云台板的 INS 兜底由 Pitch 内部处理。 */
        if (Gimbal_Command.mode != GimbalMode::DISABLED &&
            Gimbal.Gimbal_FSM.Get_Now_Status_Serial() == Gimbal_Status_READY)
        {
            QD4310_SetCurrent(&Gimbal.Yaw_Motor, 0.0f);
        }
#endif
    }

    /* 没有新命令时继续沿用上一帧 Latest-Value。 */
    GimbalCmd command;
    if (Gimbal_Command_Subscriber.Read(command))
    {
        Gimbal_Command = command;
#if GIMBAL
        Gimbal_HandleCommand(command);
#endif
    }

#if GIMBAL
#if LEGACY_INFANTRY_GIMBAL
    /* 云台板：Pitch 轴只在 IMU 模式下使能——链路健康且确实有目标时才允许输出，
     * LOCK / DISABLED 都不更新目标，避免无目标时带力矩启动。与云台板原实现一致，
     * 不使用两轴 READY 门控，Yaw 轴故障不阻断 Pitch 输出。 */
    if (Gimbal_Command.mode == GimbalMode::IMU)
    {
        Pitch_Update(Gimbal_Command.pitch_angle_rad, true, true);
    }
    else
    {
        Pitch_Update(Pitch_GetTargetAngle(), false, false);
    }
    Gimbal_Loop();
#else
    /* 只有初始化完成且两轴就绪时才允许输出，故障状态不得继续下发控制量。 */
    if (Gimbal_Command.mode != GimbalMode::DISABLED &&
        Gimbal.Gimbal_FSM.Get_Now_Status_Serial() == Gimbal_Status_READY)
    {
        Gimbal_Loop();
    }
#endif
#endif

    /* 控制保持 1 kHz，反馈降频到 100 Hz，减少应用消息复制。 */
    Gimbal_Message_Divider++;
    if (Gimbal_Message_Divider >= 10U)
    {
        Gimbal_Message_Divider = 0U;
        GimbalFeedback feedback{};
        feedback.yaw_rad = Gimbal_INS_State.yaw_rad;
        feedback.pitch_rad = Gimbal_INS_State.pitch_rad;
        feedback.yaw_speed_rad_s = Gimbal_INS_State.gyro_z_rad_s;
        feedback.pitch_speed_rad_s = Gimbal_INS_State.gyro_x_rad_s;
        feedback.ins_valid = Gimbal_INS_Valid;
#if GIMBAL
#if LEGACY_INFANTRY_GIMBAL
        /* Pitch 反馈来自 DM-IMU：欧拉角与差分角速度。 */
        float pitch_deg = 0.0f;
        if (Pitch_GetImuPitchDeg(&pitch_deg))
        {
            feedback.pitch_rad = pitch_deg * GIMBAL_DEG_TO_RAD;
        }
        feedback.pitch_speed_rad_s = Pitch_GetImuVelocityRadS();
        feedback.enabled = Pitch_IsEnabled();
        feedback.ins_valid = Gimbal_INS_Valid || Pitch_IsImuValid();
#else
        feedback.enabled = Gimbal.Yaw_Motor.enabled && Gimbal.Pitch_Motor.enabled;
#endif
#endif
        Gimbal_Feedback_Publisher.Publish(feedback);
    }
}

#else
#include "Gimbal.h"
#include "message_center.h"
#if GIMBAL
#include "alg_pid.h"
#include "dmmotor.h"
#endif
#include <cmath>

namespace
{
constexpr uint64_t GIMBAL_INS_MAX_AGE_US = 10000U;
struct GimbalContext
{
    INS_State ins{};
    bool ins_valid = false;
    uint8_t feedback_divider = 0U;
#if GIMBAL
    Struct_Gimbal_Config config{};
    Class_DMMotor yaw_motor;
    Class_DMMotor pitch_motor;
    Class_PID yaw_angle_pid;
    Class_PID yaw_speed_pid;
    Struct_DMMotor_Snapshot yaw_snapshot{};
    Struct_DMMotor_Snapshot pitch_snapshot{};
    GimbalCmd command{};
    GimbalMode last_mode = GimbalMode::DISABLED;
    bool initialized = false;
    bool was_ready = false;
    float target_yaw_angle_rad = 0.0f;
    float target_pitch_angle_rad = 0.0f;
    float target_yaw_speed_rad_s = 0.0f;
    float target_pitch_speed_rad_s = 0.0f;
    bool yaw_registered = false;
    bool pitch_registered = false;
    uint32_t target_sequence = 0U;
#endif
};

GimbalContext ctx;

}

#if GIMBAL
namespace
{
constexpr float GIMBAL_PI = 3.14159265358979323846f;
float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool ConfigValid(const Struct_Gimbal_Config &c)
{
    // PitchOnly 模式下 Yaw 配置不参与控制，只校验 Pitch 相关字段。
    const bool dual = c.axis_mode == GimbalAxisMode::DualAxis;
    if (dual)
    {
        const float yaw_nonnegative[] = {c.yaw_angle_kp, c.yaw_speed_kp, c.yaw_speed_ki,
            c.yaw_speed_kd, c.yaw_integral_limit};
        for (float value : yaw_nonnegative)
        {
            if (!std::isfinite(value) || value < 0) { return false; }
        }
        const float yaw_positive[] = {c.yaw_speed_limit, c.yaw_torque_limit};
        for (float value : yaw_positive)
        {
            if (!std::isfinite(value) || value <= 0) { return false; }
        }
    }
    const float pitch_nonnegative[] = {c.pitch_kp, c.pitch_kd};
    for (float value : pitch_nonnegative)
    {
        if (!std::isfinite(value) || value < 0) { return false; }
    }
    const float pitch_positive[] = {c.pitch_speed_limit, c.pitch_motor_per_imu};
    for (float value : pitch_positive)
    {
        if (!std::isfinite(value) || value <= 0) { return false; }
    }
    return (!dual ||
            !(c.yaw.bus == c.pitch.bus &&
              (c.yaw.id == c.pitch.id || c.yaw.feedback_id == c.pitch.feedback_id))) &&
           c.yaw_gyro_axis <= GimbalGyroAxis::Z && c.pitch_gyro_axis <= GimbalGyroAxis::Z &&
           (c.yaw_gyro_sign == 1 || c.yaw_gyro_sign == -1) &&
           (c.pitch_gyro_sign == 1 || c.pitch_gyro_sign == -1) &&
           c.yaw_integral_limit <= c.yaw_torque_limit &&
           std::isfinite(c.pitch_min) && std::isfinite(c.pitch_max) &&
           c.pitch_min < c.pitch_max;
}

float Gyro(GimbalGyroAxis axis, float sign)
{
    const float rates[] = {ctx.ins.gyro_x_rad_s, ctx.ins.gyro_y_rad_s,
                           ctx.ins.gyro_z_rad_s};
    return sign * rates[static_cast<unsigned>(axis)];
}

void ResetControllers()
{
    // PID::Init 保留历史，因此先重建值对象，清除积分、微分及目标历史。
    ctx.yaw_angle_pid = Class_PID{};
    ctx.yaw_speed_pid = Class_PID{};
    ctx.yaw_angle_pid.Init(ctx.config.yaw_angle_kp, 0, 0, 0, 0, ctx.config.yaw_speed_limit);
    ctx.yaw_speed_pid.Init(ctx.config.yaw_speed_kp, ctx.config.yaw_speed_ki,
        ctx.config.yaw_speed_kd, 0, ctx.config.yaw_integral_limit, ctx.config.yaw_torque_limit);
}

void CapturePose(uint32_t sequence)
{
    ResetControllers();
    ctx.target_yaw_angle_rad = ctx.ins.yaw_rad;
    ctx.target_pitch_angle_rad = ctx.ins.pitch_rad;
    ctx.target_yaw_speed_rad_s = ctx.target_pitch_speed_rad_s = 0;
    // 恢复前已发布的目标全部丢弃；IMU 只接受之后的新序号。
    ctx.target_sequence = sequence;
}

void Stop()
{
    // 首次失能请求或失能边沿立即尝试发布安全目标；补交与协议纠正交给 ServiceAll。
    if (ctx.yaw_registered) { (void)ctx.yaw_motor.RequestEnabled(false); }
    if (ctx.pitch_registered) { (void)ctx.pitch_motor.RequestEnabled(false); }
}

void Control(const Struct_DMMotor_Snapshot &pitch)
{
    if (ctx.config.axis_mode == GimbalAxisMode::PitchOnly)
    {
        // 单轴模式：Yaw 由底盘板控制，本板只输出 Pitch MIT 目标。
        const float position = pitch.feedback.position + ctx.config.pitch_motor_per_imu *
                              (ctx.target_pitch_angle_rad - ctx.ins.pitch_rad);
        const float velocity = pitch.feedback.velocity + ctx.config.pitch_motor_per_imu *
                              (ctx.target_pitch_speed_rad_s - Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign));
        (void)ctx.pitch_motor.SetMIT(Clamp(position, ctx.config.pitch_min, ctx.config.pitch_max),
            Clamp(velocity, -ctx.config.pitch_speed_limit, ctx.config.pitch_speed_limit), ctx.config.pitch_kp, ctx.config.pitch_kd, 0);
        return;
    }
    // Yaw 复用现有 PID：最短角误差生成角速度，再由速度环生成转矩。
    const float error = std::remainder(ctx.target_yaw_angle_rad - ctx.ins.yaw_rad, 2 * GIMBAL_PI);
    ctx.yaw_angle_pid.Set_Target(error);
    ctx.yaw_angle_pid.Set_Now(0);
    ctx.yaw_angle_pid.TIM_Calculate_PeriodElapsedCallback();
    const float speed = ctx.yaw_angle_pid.Get_Out() + ctx.target_yaw_speed_rad_s;
    ctx.yaw_speed_pid.Set_Target(Clamp(speed, -ctx.config.yaw_speed_limit, ctx.config.yaw_speed_limit));
    ctx.yaw_speed_pid.Set_Now(Gyro(ctx.config.yaw_gyro_axis, ctx.config.yaw_gyro_sign));
    ctx.yaw_speed_pid.TIM_Calculate_PeriodElapsedCallback();
    const float torque = ctx.yaw_speed_pid.Get_Out();
    // Pitch 将 INS 姿态/角速度误差换算为电机目标；位置、速度均基于同一次反馈快照。
    const float position = pitch.feedback.position + ctx.config.pitch_motor_per_imu *
                          (ctx.target_pitch_angle_rad - ctx.ins.pitch_rad);
    const float velocity = pitch.feedback.velocity + ctx.config.pitch_motor_per_imu *
                          (ctx.target_pitch_speed_rad_s - Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign));
    (void)ctx.yaw_motor.SetTorque(Clamp(torque, -ctx.config.yaw_torque_limit, ctx.config.yaw_torque_limit));
    (void)ctx.pitch_motor.SetMIT(Clamp(position, ctx.config.pitch_min, ctx.config.pitch_max),
        Clamp(velocity, -ctx.config.pitch_speed_limit, ctx.config.pitch_speed_limit), ctx.config.pitch_kp, ctx.config.pitch_kd, 0);
}

void UpdateTarget(const TopicSnapshot<GimbalCmd> &message)
{
    // LOCK 只在进入时捕获姿态；IMU 只接受新序号，避免恢复后重放旧目标。
    if (ctx.command.mode == GimbalMode::LOCK && ctx.last_mode != GimbalMode::LOCK)
    {
        CapturePose(message.sequence);
    }
    if (ctx.command.mode == GimbalMode::IMU && message.sequence != ctx.target_sequence)
    {
        ctx.target_yaw_angle_rad = ctx.command.yaw_angle_rad;
        ctx.target_pitch_angle_rad = ctx.command.pitch_angle_rad;
        ctx.target_yaw_speed_rad_s = ctx.command.yaw_speed_rad_s;
        ctx.target_pitch_speed_rad_s = ctx.command.pitch_speed_rad_s;
        ctx.target_sequence = message.sequence;
    }
    ctx.last_mode = ctx.command.mode;
}
} // namespace

bool Gimbal_Init(const Struct_Gimbal_Config &requested)
{
    if (!ConfigValid(requested))
    {
        ctx.initialized = false;
        return false;
    }
    ctx.config = requested;
    // PitchOnly（单轴）模式不初始化 Yaw：老步兵双板分工下 Yaw 由底盘板主控。
    ctx.yaw_registered =
        ctx.config.axis_mode == GimbalAxisMode::DualAxis &&
        ctx.yaw_motor.Init(ctx.config.yaw.bus, ctx.config.yaw.id, ctx.config.yaw.feedback_id,
            Enum_DMMotor_Mode::MIT, ctx.config.yaw.reverse, ctx.config.yaw.position_max,
            ctx.config.yaw.velocity_max, ctx.config.yaw.torque_max);
    ctx.pitch_registered = ctx.pitch_motor.Init(ctx.config.pitch.bus, ctx.config.pitch.id, ctx.config.pitch.feedback_id,
        Enum_DMMotor_Mode::MIT, ctx.config.pitch.reverse, ctx.config.pitch.position_max,
        ctx.config.pitch.velocity_max, ctx.config.pitch.torque_max);
    ctx.initialized = ctx.yaw_registered && ctx.pitch_registered;
    ctx.command = {};
    ctx.was_ready = false;
    ResetControllers();
    return ctx.initialized;
}

Enum_Gimbal_Status Gimbal_GetStatus(void)
{
    // 状态由当前输入与电机快照推导，仅用于观察，不安排重试或驱动状态迁移。
    if (!ctx.initialized) { return Gimbal_Status_CONFIG_ERROR; }
    if (ctx.command.mode == GimbalMode::DISABLED) { return Gimbal_Status_DISABLE; }
    const bool dual = ctx.config.axis_mode == GimbalAxisMode::DualAxis;
    if (!ctx.ins_valid || ctx.pitch_snapshot.fault || (dual && ctx.yaw_snapshot.fault))
    {
        return Gimbal_Status_FAULT;
    }
    const bool ready = ctx.pitch_snapshot.ready && (!dual || ctx.yaw_snapshot.ready);
    return ready ? Gimbal_Status_READY : Gimbal_Status_ENABLING;
}
#endif

static void PublishFeedback(void)
{
    if (++ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0;
        GimbalFeedback feedback{};
        if (ctx.ins_valid)
        {
            feedback.yaw_rad = ctx.ins.yaw_rad;
            feedback.pitch_rad = ctx.ins.pitch_rad;
#if GIMBAL
            feedback.yaw_speed_rad_s = Gyro(ctx.config.yaw_gyro_axis, ctx.config.yaw_gyro_sign);
            feedback.pitch_speed_rad_s = Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign);
#else
            feedback.yaw_speed_rad_s = ctx.ins.gyro_z_rad_s;
            feedback.pitch_speed_rad_s = ctx.ins.gyro_y_rad_s;
#endif
        }
        feedback.ins_valid = ctx.ins_valid;
#if GIMBAL
        feedback.enabled = ctx.pitch_snapshot.ready &&
            (ctx.config.axis_mode != GimbalAxisMode::DualAxis || ctx.yaw_snapshot.ready);
#endif
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
    }
}

void Gimbal_Update(void)
{
    ctx.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ctx.ins, GIMBAL_INS_MAX_AGE_US);
#if GIMBAL
    const auto message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.command = message.valid ? message.data : GimbalCmd{};
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();

    if (!ctx.initialized || !message.valid ||
        ctx.command.mode == GimbalMode::DISABLED || !ctx.ins_valid ||
        ctx.yaw_snapshot.fault || ctx.pitch_snapshot.fault)
    {
        Stop();
        ctx.was_ready = false;
        ctx.last_mode = GimbalMode::DISABLED;
        ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
        ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
        PublishFeedback();
        return;
    }

    // 每周期表达输出许可；驱动只处理请求边沿，重复使能不会覆盖正常周期目标。
    const bool dual = ctx.config.axis_mode == GimbalAxisMode::DualAxis;
    if (dual) { (void)ctx.yaw_motor.RequestEnabled(true); }
    (void)ctx.pitch_motor.RequestEnabled(true);
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
    if (!ctx.pitch_snapshot.ready || (dual && !ctx.yaw_snapshot.ready))
    {
        // 等待受控轴反馈就绪；安全输出和低频协议纠正由 DMMotor 维护。
        ctx.was_ready = false;
        PublishFeedback();
        return;
    }

    if (!ctx.was_ready)
    {
        CapturePose(message.sequence);
        ctx.last_mode = ctx.command.mode;
        ctx.was_ready = true;
    }
    UpdateTarget(message);
    Control(ctx.pitch_snapshot);
#endif
    PublishFeedback();
}

#endif

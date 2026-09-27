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

/* ==========================================================================
 * 框架通用实现（RoboMaster_H7）
 * ========================================================================== */

#include <cmath>

static INS_State Gimbal_INS_State;
static bool Gimbal_INS_Valid = false;
static constexpr uint64_t GIMBAL_INS_MAX_AGE_US = 10000U;
static uint8_t Gimbal_Message_Divider;

static bool Gimbal_INS_Finite(const INS_State &ins)
{
    return std::isfinite(ins.yaw_rad) && std::isfinite(ins.pitch_rad) &&
           std::isfinite(ins.roll_rad) && std::isfinite(ins.gyro_x_rad_s) &&
           std::isfinite(ins.gyro_y_rad_s) && std::isfinite(ins.gyro_z_rad_s);
}

#if GIMBAL
#include "sys_timestamp.h"

Struct_Gimbal Gimbal;

namespace
{
constexpr float GIMBAL_PI = 3.14159265358979323846f;
constexpr uint64_t RETRY_US = 20000U;
constexpr uint64_t ENABLE_TIMEOUT_US = 2000000U;
constexpr uint64_t BACKOFF_US = 1000000U;
constexpr uint64_t STABLE_US = 100000U;
Struct_Gimbal_Config config;
bool yaw_registered, pitch_registered;
GimbalMode last_mode = GimbalMode::DISABLED;
uint64_t state_since, stable_since, last_enable, last_disable;
bool stabilizing, enable_sent, disable_sent;
uint32_t target_sequence;

float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool MotorConfigValid(const Struct_Gimbal_Motor_Config &motor)
{
    return (motor.bus == &hfdcan1 || motor.bus == &hfdcan2 || motor.bus == &hfdcan3) &&
           motor.id != 0 && motor.feedback_id <= 0x7ff &&
           std::isfinite(motor.position_max) && motor.position_max > 0 &&
           std::isfinite(motor.velocity_max) && motor.velocity_max > 0 &&
           std::isfinite(motor.torque_max) && motor.torque_max > 0;
}

bool ConfigValid(const Struct_Gimbal_Config &c)
{
    const float nonnegative[] = {c.yaw_angle_kp, c.yaw_speed_kp, c.yaw_speed_ki,
        c.yaw_speed_kd, c.yaw_integral_limit, c.pitch_kp, c.pitch_kd};
    for (float value : nonnegative)
    {
        if (!std::isfinite(value) || value < 0) { return false; }
    }
    const float positive[] = {c.yaw_speed_limit, c.yaw_torque_limit,
                              c.pitch_speed_limit, c.pitch_motor_per_imu};
    for (float value : positive)
    {
        if (!std::isfinite(value) || value <= 0) { return false; }
    }
    return MotorConfigValid(c.yaw) && MotorConfigValid(c.pitch) &&
           !(c.yaw.bus == c.pitch.bus &&
             (c.yaw.id == c.pitch.id || c.yaw.feedback_id == c.pitch.feedback_id)) &&
           c.yaw_gyro_axis <= GimbalGyroAxis::Z && c.pitch_gyro_axis <= GimbalGyroAxis::Z &&
           (c.yaw_gyro_sign == 1 || c.yaw_gyro_sign == -1) &&
           (c.pitch_gyro_sign == 1 || c.pitch_gyro_sign == -1) &&
           c.yaw_speed_limit <= c.yaw.velocity_max &&
           c.yaw_torque_limit <= c.yaw.torque_max &&
           c.yaw_integral_limit <= c.yaw_torque_limit &&
           c.pitch_kp <= 500 && c.pitch_kd <= 5 &&
           std::isfinite(c.pitch_min) && std::isfinite(c.pitch_max) &&
           c.pitch_min < c.pitch_max && c.pitch_min >= -c.pitch.position_max &&
           c.pitch_max <= c.pitch.position_max && c.pitch_speed_limit <= c.pitch.velocity_max;
}

bool CommandValid(const GimbalCmd &command)
{
    return (command.mode == GimbalMode::DISABLED || command.mode == GimbalMode::IMU ||
            command.mode == GimbalMode::LOCK) &&
           std::isfinite(command.yaw_angle_rad) && std::isfinite(command.pitch_angle_rad) &&
           std::isfinite(command.yaw_speed_rad_s) && std::isfinite(command.pitch_speed_rad_s);
}

bool FeedbackFinite(const Struct_DMMotor_Snapshot &snapshot)
{
    const auto &f = snapshot.feedback;
    return std::isfinite(f.position) && std::isfinite(f.total_position) &&
           std::isfinite(f.velocity) && std::isfinite(f.torque) &&
           std::isfinite(f.mos_temperature) && std::isfinite(f.rotor_temperature);
}

bool MotorFault(const Struct_DMMotor_Snapshot &snapshot)
{
    // 只有新鲜反馈中的状态才可用；0 为失能，1 为使能，其余状态不自动清错。
    return snapshot.online && snapshot.feedback.state > 1;
}

float Gyro(GimbalGyroAxis axis, float sign)
{
    const float rates[] = {Gimbal_INS_State.gyro_x_rad_s, Gimbal_INS_State.gyro_y_rad_s,
                           Gimbal_INS_State.gyro_z_rad_s};
    return sign * rates[static_cast<unsigned>(axis)];
}

void ResetControllers()
{
    // PID::Init 保留历史，因此先重建值对象，清除积分、微分及目标历史。
    Gimbal.Yaw_Angle_PID = Class_PID{};
    Gimbal.Yaw_Speed_PID = Class_PID{};
    Gimbal.Yaw_Angle_PID.Init(config.yaw_angle_kp, 0, 0, 0, 0, config.yaw_speed_limit);
    Gimbal.Yaw_Speed_PID.Init(config.yaw_speed_kp, config.yaw_speed_ki,
        config.yaw_speed_kd, 0, config.yaw_integral_limit, config.yaw_torque_limit);
}

void CapturePose(uint32_t sequence)
{
    ResetControllers();
    Gimbal.Target_Yaw_Angle = Gimbal_INS_State.yaw_rad;
    Gimbal.Target_Pitch_Angle = Gimbal_INS_State.pitch_rad;
    Gimbal.Target_Yaw_Speed = Gimbal.Target_Pitch_Speed = 0;
    // 恢复前已发布的目标全部丢弃；IMU 只接受之后的新序号。
    target_sequence = sequence;
}

void SetState(Enum_Gimbal_Status state, uint64_t now)
{
    if (Gimbal.status != state)
    {
        Gimbal.status = state;
        state_since = now;
        stabilizing = false;
        enable_sent = disable_sent = false;
    }
}

bool ZeroOutput()
{
    // 两轴都尝试，不能用短路表达式跳过第二轴；覆盖尚未发出的旧周期帧。
    const bool yaw_ok = !yaw_registered || Gimbal.Yaw_Motor.SetTorque(0);
    const bool pitch_ok = !pitch_registered || Gimbal.Pitch_Motor.SetTorque(0);
    return yaw_ok && pitch_ok;
}

void Stop(uint64_t now, const Struct_DMMotor_Snapshot &yaw,
          const Struct_DMMotor_Snapshot &pitch)
{
    (void)ZeroOutput(); // 失败时下个周期继续覆盖，不将发布失败当作停机成功。
    if (!disable_sent || now - last_disable >= RETRY_US)
    {
        if (yaw_registered && (!yaw.online || yaw.feedback.state != 0))
        {
            (void)Gimbal.Yaw_Motor.Disable();
        }
        if (pitch_registered && (!pitch.online || pitch.feedback.state != 0))
        {
            (void)Gimbal.Pitch_Motor.Disable();
        }
        last_disable = now;
        disable_sent = true;
    }
}

bool Control(const Struct_DMMotor_Snapshot &pitch)
{
    const float error = std::remainder(Gimbal.Target_Yaw_Angle - Gimbal_INS_State.yaw_rad, 2 * GIMBAL_PI);
    if (!std::isfinite(error)) { return false; }
    Gimbal.Yaw_Angle_PID.Set_Target(error);
    Gimbal.Yaw_Angle_PID.Set_Now(0);
    Gimbal.Yaw_Angle_PID.TIM_Calculate_PeriodElapsedCallback();
    const float speed = Gimbal.Yaw_Angle_PID.Get_Out() + Gimbal.Target_Yaw_Speed;
    if (!std::isfinite(speed)) { return false; }
    Gimbal.Yaw_Speed_PID.Set_Target(Clamp(speed, -config.yaw_speed_limit, config.yaw_speed_limit));
    Gimbal.Yaw_Speed_PID.Set_Now(Gyro(config.yaw_gyro_axis, config.yaw_gyro_sign));
    Gimbal.Yaw_Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    const float torque = Gimbal.Yaw_Speed_PID.Get_Out();
    const float position = pitch.feedback.position + config.pitch_motor_per_imu *
                          (Gimbal.Target_Pitch_Angle - Gimbal_INS_State.pitch_rad);
    const float velocity = pitch.feedback.velocity + config.pitch_motor_per_imu *
                          (Gimbal.Target_Pitch_Speed - Gyro(config.pitch_gyro_axis, config.pitch_gyro_sign));
    if (!std::isfinite(torque) || !std::isfinite(position) || !std::isfinite(velocity)) { return false; }
    const bool yaw_ok = Gimbal.Yaw_Motor.SetTorque(Clamp(torque, -config.yaw_torque_limit, config.yaw_torque_limit));
    const bool pitch_ok = Gimbal.Pitch_Motor.SetMIT(Clamp(position, config.pitch_min, config.pitch_max),
        Clamp(velocity, -config.pitch_speed_limit, config.pitch_speed_limit), config.pitch_kp, config.pitch_kd, 0);
    return yaw_ok && pitch_ok;
}

void UpdateControl(const TopicSnapshot<GimbalCmd> &message,
                   const Struct_DMMotor_Snapshot &yaw, const Struct_DMMotor_Snapshot &pitch)
{
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    if (Gimbal.status == Gimbal_Status_CONFIG_ERROR)
    {
        Stop(now, yaw, pitch);
        return;
    }
    const GimbalCmd command = message.valid ? message.data : GimbalCmd{};
    const bool valid = CommandValid(command) && Gimbal_INS_Valid &&
                       FeedbackFinite(yaw) && FeedbackFinite(pitch) &&
                       !MotorFault(yaw) && !MotorFault(pitch);
    const bool healthy = yaw.online && yaw.enabled && pitch.online && pitch.enabled;
    if (command.mode == GimbalMode::DISABLED)
    {
        SetState(Gimbal_Status_DISABLE, now);
        Stop(now, yaw, pitch);
        last_mode = GimbalMode::DISABLED;
        return;
    }
    if (!valid || (Gimbal.status == Gimbal_Status_READY && !healthy))
    {
        SetState(Gimbal_Status_FAULT, now);
    }
    if (Gimbal.status == Gimbal_Status_FAULT)
    {
        Stop(now, yaw, pitch);
        if (valid && now - state_since >= BACKOFF_US)
        {
            SetState(Gimbal_Status_ENABLING, now);
        }
        return;
    }
    if (Gimbal.status == Gimbal_Status_DISABLE)
    {
        SetState(Gimbal_Status_ENABLING, now);
    }
    if (Gimbal.status == Gimbal_Status_ENABLING)
    {
        if (!ZeroOutput() || now - state_since >= ENABLE_TIMEOUT_US)
        {
            SetState(Gimbal_Status_FAULT, now);
            Stop(now, yaw, pitch);
            return;
        }
        if (!enable_sent || now - last_enable >= RETRY_US)
        {
            if (!yaw.online || !yaw.enabled) { (void)Gimbal.Yaw_Motor.Enable(); }
            if (!pitch.online || !pitch.enabled) { (void)Gimbal.Pitch_Motor.Enable(); }
            last_enable = now;
            enable_sent = true;
        }
        if (!healthy) { stabilizing = false; }
        else if (!stabilizing) { stable_since = now; stabilizing = true; }
        else if (now - stable_since >= STABLE_US)
        {
            CapturePose(message.sequence);
            last_mode = command.mode;
            SetState(Gimbal_Status_READY, now);
        }
        return;
    }
    if (command.mode == GimbalMode::LOCK && last_mode != GimbalMode::LOCK)
    {
        CapturePose(message.sequence);
    }
    if (command.mode == GimbalMode::IMU && message.sequence != target_sequence)
    {
        Gimbal.Target_Yaw_Angle = command.yaw_angle_rad;
        Gimbal.Target_Pitch_Angle = command.pitch_angle_rad;
        Gimbal.Target_Yaw_Speed = command.yaw_speed_rad_s;
        Gimbal.Target_Pitch_Speed = command.pitch_speed_rad_s;
        target_sequence = message.sequence;
    }
    last_mode = command.mode;
    if (!Control(pitch))
    {
        SetState(Gimbal_Status_FAULT, now);
        Stop(now, yaw, pitch);
    }
}
} // namespace

bool Gimbal_Init(const Struct_Gimbal_Config &requested)
{
    if (!ConfigValid(requested))
    {
        Gimbal.status = Gimbal_Status_CONFIG_ERROR;
        return false;
    }
    config = requested;
    Gimbal.Yaw_Motor.SetAutoEnableOnOffline(false);
    Gimbal.Pitch_Motor.SetAutoEnableOnOffline(false);
    yaw_registered = Gimbal.Yaw_Motor.Init(config.yaw.bus, config.yaw.id, config.yaw.feedback_id,
        Enum_DMMotor_Mode::MIT, config.yaw.reverse, config.yaw.position_max,
        config.yaw.velocity_max, config.yaw.torque_max);
    pitch_registered = Gimbal.Pitch_Motor.Init(config.pitch.bus, config.pitch.id, config.pitch.feedback_id,
        Enum_DMMotor_Mode::MIT, config.pitch.reverse, config.pitch.position_max,
        config.pitch.velocity_max, config.pitch.torque_max);
    Gimbal.status = yaw_registered && pitch_registered ? Gimbal_Status_DISABLE : Gimbal_Status_CONFIG_ERROR;
    ResetControllers();
    return yaw_registered && pitch_registered;
}
#endif

void Gimbal_Update(void)
{
    Gimbal_INS_Valid = MessageCenter::INS_State_Topic.ReadFresh(Gimbal_INS_State, GIMBAL_INS_MAX_AGE_US) &&
                       Gimbal_INS_Finite(Gimbal_INS_State);
#if GIMBAL
    const auto yaw = Gimbal.Yaw_Motor.GetFeedbackSnapshot();
    const auto pitch = Gimbal.Pitch_Motor.GetFeedbackSnapshot();
    UpdateControl(MessageCenter::Gimbal_Command_Topic.ReadWithMeta(), yaw, pitch);
#endif
    if (++Gimbal_Message_Divider >= 10U)
    {
        Gimbal_Message_Divider = 0;
        GimbalFeedback feedback{};
        if (Gimbal_INS_Valid)
        {
            feedback.yaw_rad = Gimbal_INS_State.yaw_rad;
            feedback.pitch_rad = Gimbal_INS_State.pitch_rad;
#if GIMBAL
            feedback.yaw_speed_rad_s = Gyro(config.yaw_gyro_axis, config.yaw_gyro_sign);
            feedback.pitch_speed_rad_s = Gyro(config.pitch_gyro_axis, config.pitch_gyro_sign);
#else
            feedback.yaw_speed_rad_s = Gimbal_INS_State.gyro_z_rad_s;
            feedback.pitch_speed_rad_s = Gimbal_INS_State.gyro_y_rad_s;
#endif
        }
        feedback.ins_valid = Gimbal_INS_Valid;
#if GIMBAL
        feedback.enabled = yaw.online && yaw.enabled && pitch.online && pitch.enabled;
#endif
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
    }
}

#endif

#include "Gimbal.h"
#include "message_center.h"
#include <cmath>

static INS_State Gimbal_INS_State;
static bool Gimbal_INS_Valid = false;
static constexpr uint64_t GIMBAL_INS_MAX_AGE_US = 10000U;
static uint8_t Gimbal_Message_Divider;
#if LEGACY_INFANTRY
static GimbalMode Gimbal_Last_Mode = GimbalMode::DISABLED;
static GimbalCmd Gimbal_Command;
#endif

#if LEGACY_INFANTRY

#include "SpeedPlanning.h"
#include "fdcan.h"
#include <cmath>

/* ============================== 控制参数 ============================== */
/* 全部取自 demo 的 APP/GimbalTask.c 与 User/bsp/bsp_def.h。 */

/** 偏航摇杆速度上限，rad/s；必须与 Com.cpp 的 GIMBAL_TARGET_YAW_SPEED_MAX 一致。 */
static constexpr float GIMBAL_YAW_SPEED_MAX = 15.0f;
/** 机体系 Z 轴角速度前馈增益，用于抑制底盘自转耦合。 */
static constexpr float GIMBAL_YAW_RATE_FEEDFORWARD_GAIN = 1.0f;
/**
 * 速度规划控制周期。老工程的控制路径是 2 ms，这里保持一致：Control_Task 虽然是
 * 1 kHz，但云台下发按 2 分频执行，避免与底盘帧、板间帧叠加后压满 1 Mbps 总线。
 */
static constexpr float GIMBAL_CONTROL_DT = 0.002f;
/** Control_Task 的 1 kHz 调度下，云台控制路径的执行分频（2 对应 2 ms）。 */
static constexpr uint8_t GIMBAL_CONTROL_DIVIDER = 2U;
/** 使能重发的分频基准是 2 ms，50 表示每 100 ms 重发一次使能命令。 */
static constexpr uint8_t GIMBAL_ENABLE_RETRY_DIVIDER = 50U;
/** 禁用期间每 20 ms 重发零速和失能。 */
static constexpr uint8_t GIMBAL_DISABLE_RETRY_DIVIDER = 10U;
/** 零速吸附门限。 */
static constexpr float GIMBAL_PLANNING_THRESHOLD = 0.1f;

/** 偏航速率限制：加速度上限随摇杆比例在 MIN 与 MAX 之间线性插值。 */
static constexpr float GIMBAL_YAW_ACCEL_LIMIT_MIN = 60.0f;
static constexpr float GIMBAL_YAW_ACCEL_LIMIT_MAX = 150.0f;
static constexpr float GIMBAL_YAW_DECEL_LIMIT = 120.0f;
static constexpr float GIMBAL_YAW_RELEASE_LIMIT = 75.0f;
static constexpr float GIMBAL_YAW_REVERSE_LIMIT = 250.0f;

/** MIT 参数：位置增益恒为 0，即纯速度 + 阻尼控制，位置目标固定为 0。 */
static constexpr float GIMBAL_YAW_MIT_KP = 0.0f;
static constexpr float GIMBAL_YAW_MIT_KD_CENTER = 1.4f;
static constexpr float GIMBAL_YAW_MIT_KD_MOVING = 1.0f;
static constexpr float GIMBAL_YAW_MIT_KD_REVERSE = 1.6f;
static constexpr float GIMBAL_YAW_MIT_KD_SLEW_RATE = 5.0f;
/** 力矩前馈：由速度规划的加速度换算，含增益、限幅与变化率限制。 */
static constexpr float GIMBAL_YAW_MIT_TORQUE_GAIN = 0.002f;
static constexpr float GIMBAL_YAW_MIT_TORQUE_MAX = 0.35f;
static constexpr float GIMBAL_YAW_MIT_TORQUE_SLEW = 8.0f;

/** DM 云台电机量程，与 demo 的 DM_drv.h 一致。 */
static constexpr float GIMBAL_MOTOR_POSITION_MAX_RAD = 3.14f;
static constexpr float GIMBAL_MOTOR_VELOCITY_MAX_RAD_S = 30.0f;
static constexpr float GIMBAL_MOTOR_TORQUE_MAX_NM = 10.0f;

DMGimbal_t Gimbal;
static bool Gimbal_Yaw_Output_Enabled;
static uint8_t Gimbal_Loop_Divider;
static uint8_t Gimbal_Enable_Retry_Divider;
static uint8_t Gimbal_Disable_Retry_Divider;

/** 把 value 约束到 [minimum, maximum]。 */
static float Gimbal_Constrain(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/** 以最大步长 maximum_delta 逼近 target，用于增益与力矩前馈的逐周期限速。 */
static float Gimbal_MoveTowards(float current, float target, float maximum_delta)
{
    const float delta = target - current;

    if (fabsf(delta) <= maximum_delta)
    {
        return target;
    }
    return current + (delta > 0.0f ? maximum_delta : -maximum_delta);
}

void Gimbal_Init(void)
{
    Gimbal_Yaw_Output_Enabled = false;
    Gimbal_Loop_Divider = 0U;
    Gimbal_Enable_Retry_Divider = 0U;
    Gimbal_Disable_Retry_Divider = 0U;
    Gimbal.Yaw_Speed_Command = 0.0f;
    Gimbal.Yaw_Mit_Kd = GIMBAL_YAW_MIT_KD_CENTER;
    Gimbal.Yaw_Mit_Torque_Feedforward = 0.0f;
    SpeedPlanning_Init(&Gimbal.Yaw_Speed_Planning, 0.0f);

    (void)Gimbal.Yaw_Motor.Init(&hfdcan1,
                              GIMBAL_YAW_MOTOR_CAN_ID,
                              GIMBAL_YAW_MOTOR_MASTER_ID,
                              Enum_DMMotor_Mode::MIT,
                              false,
                              GIMBAL_MOTOR_POSITION_MAX_RAD,
                              GIMBAL_MOTOR_VELOCITY_MAX_RAD_S,
                              GIMBAL_MOTOR_TORQUE_MAX_NM);
    /* 反馈注册失败时也尝试停止已配置的电机。 */
    Gimbal.Yaw_Motor.SetMIT(0.0f, 0.0f, GIMBAL_YAW_MIT_KP,
                            GIMBAL_YAW_MIT_KD_CENTER, 0.0f);
    (void)Gimbal.Yaw_Motor.Disable();
}

void Gimbal_Loop(void)
{
    /* 摇杆目标速度：死区与指数整形已在 Communication 层完成。 */
    const float stick_speed_command = Gimbal_Constrain(
        Gimbal_Command.yaw_speed_rad_s, -GIMBAL_YAW_SPEED_MAX, GIMBAL_YAW_SPEED_MAX);

    /* INS 不可用时仅停用角速度补偿，保留遥控器 yaw 速度控制。 */
    const float chassis_yaw_rate =
        Gimbal_INS_Valid ? Gimbal_INS_State.gyro_z_rad_s : 0.0f;
    float yaw_speed_command =
        stick_speed_command - GIMBAL_YAW_RATE_FEEDFORWARD_GAIN * chassis_yaw_rate;
    yaw_speed_command =
        Gimbal_Constrain(yaw_speed_command, -GIMBAL_YAW_SPEED_MAX, GIMBAL_YAW_SPEED_MAX);

    const float yaw_stick_ratio = fabsf(stick_speed_command) / GIMBAL_YAW_SPEED_MAX;
    const float yaw_speed_previous = Gimbal.Yaw_Speed_Planning.current_speed;
    const float yaw_acceleration_limit =
        GIMBAL_YAW_ACCEL_LIMIT_MIN +
        (GIMBAL_YAW_ACCEL_LIMIT_MAX - GIMBAL_YAW_ACCEL_LIMIT_MIN) * yaw_stick_ratio;

    const float yaw_speed = SpeedPlanning_UpdateRateLimited(
        yaw_speed_command, &Gimbal.Yaw_Speed_Planning, GIMBAL_CONTROL_DT,
        yaw_acceleration_limit, GIMBAL_YAW_DECEL_LIMIT,
        GIMBAL_YAW_RELEASE_LIMIT, GIMBAL_YAW_REVERSE_LIMIT,
        GIMBAL_PLANNING_THRESHOLD);
    Gimbal.Yaw_Speed_Command = yaw_speed;

    /* 阻尼随摇杆推进减小、反向瞬间提高，逐周期限速避免阶跃。 */
    float mit_kd_target = GIMBAL_YAW_MIT_KD_CENTER +
                          (GIMBAL_YAW_MIT_KD_MOVING - GIMBAL_YAW_MIT_KD_CENTER) * yaw_stick_ratio;
    if (yaw_speed_previous * stick_speed_command < 0.0f)
    {
        mit_kd_target = GIMBAL_YAW_MIT_KD_REVERSE;
    }
    Gimbal.Yaw_Mit_Kd = Gimbal_MoveTowards(Gimbal.Yaw_Mit_Kd, mit_kd_target,
                                           GIMBAL_YAW_MIT_KD_SLEW_RATE * GIMBAL_CONTROL_DT);

    /* 由速度规划的等效加速度换算前馈力矩，并限幅、限速。 */
    const float yaw_acceleration_command =
        (yaw_speed - yaw_speed_previous) / GIMBAL_CONTROL_DT;
    const float mit_torque_target =
        Gimbal_Constrain(yaw_acceleration_command * GIMBAL_YAW_MIT_TORQUE_GAIN,
                         -GIMBAL_YAW_MIT_TORQUE_MAX, GIMBAL_YAW_MIT_TORQUE_MAX);
    Gimbal.Yaw_Mit_Torque_Feedforward = Gimbal_MoveTowards(
        Gimbal.Yaw_Mit_Torque_Feedforward, mit_torque_target,
        GIMBAL_YAW_MIT_TORQUE_SLEW * GIMBAL_CONTROL_DT);

    /* MIT 下发：位置目标 0、位置增益 0，速度由电机内部闭环，阻尼与外力矩由本层给。 */
    Gimbal.Yaw_Motor.SetMIT(0.0f,
                            yaw_speed,
                            GIMBAL_YAW_MIT_KP,
                            Gimbal.Yaw_Mit_Kd,
                            Gimbal.Yaw_Mit_Torque_Feedforward);
}

#endif /* LEGACY_INFANTRY */

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
#if LEGACY_INFANTRY
    /* 没有新命令时继续沿用上一帧 Latest-Value。 */
    GimbalCmd command;
    if (MessageCenter::Gimbal_Command_Topic.Read(command))
    {
        Gimbal_Command = command;
        if (command.mode == GimbalMode::DISABLED)
        {
            if (Gimbal_Last_Mode != GimbalMode::DISABLED)
            {
                Gimbal.Yaw_Motor.SetMIT(0.0f, 0.0f, GIMBAL_YAW_MIT_KP,
                                        GIMBAL_YAW_MIT_KD_CENTER, 0.0f);
                (void)Gimbal.Yaw_Motor.Disable();
                Gimbal_Yaw_Output_Enabled = false;
                Gimbal_Disable_Retry_Divider = 0U;
                Gimbal.Yaw_Speed_Command = 0.0f;
                Gimbal.Yaw_Mit_Torque_Feedforward = 0.0f;
                SpeedPlanning_Init(&Gimbal.Yaw_Speed_Planning, 0.0f);
            }
        }
        else if (Gimbal_Last_Mode == GimbalMode::DISABLED)
        {
            (void)Gimbal.Yaw_Motor.Enable();
            Gimbal_Yaw_Output_Enabled = true;
        }
        Gimbal_Last_Mode = command.mode;
    }

    /* 云台控制按 2 ms 执行：与老工程一致，同时控制 CAN 总线负载。 */
    Gimbal_Loop_Divider++;
    if (Gimbal_Loop_Divider >= GIMBAL_CONTROL_DIVIDER)
    {
        Gimbal_Loop_Divider = 0U;
        /* 只有命令要求使能时才下发；禁用状态保持电机失能。 */
        if (Gimbal_Yaw_Output_Enabled)
        {
            Gimbal_Loop();

            /* 与底盘同理：使能命令只在状态跳变时下发一次，丢失后没有第二次机会。 */
            Gimbal_Enable_Retry_Divider++;
            if (Gimbal_Enable_Retry_Divider >= GIMBAL_ENABLE_RETRY_DIVIDER)
            {
                Gimbal_Enable_Retry_Divider = 0U;
                (void)Gimbal.Yaw_Motor.Enable();
            }
        }
        else
        {
            Gimbal_Disable_Retry_Divider++;
            if (Gimbal_Disable_Retry_Divider >= GIMBAL_DISABLE_RETRY_DIVIDER)
            {
                Gimbal_Disable_Retry_Divider = 0U;
                Gimbal.Yaw_Motor.SetMIT(0.0f, 0.0f, GIMBAL_YAW_MIT_KP,
                                        GIMBAL_YAW_MIT_KD_CENTER, 0.0f);
                (void)Gimbal.Yaw_Motor.Disable();
            }
        }
    }
#endif

    /* 控制保持 1 kHz，反馈降频到 100 Hz，减少应用消息复制。 */
    Gimbal_Message_Divider++;
    if (Gimbal_Message_Divider >= 10U)
    {
        Gimbal_Message_Divider = 0U;
        GimbalFeedback feedback{};
#if LEGACY_INFANTRY
        /* 老步兵模式下 yaw 反馈取云台电机机械角，供底盘跟随与板间链路使用。 */
        feedback.yaw_rad = Gimbal.Yaw_Motor.feedback.position;
        feedback.pitch_rad = 0.0f;
        feedback.yaw_speed_rad_s = Gimbal.Yaw_Speed_Command;
        feedback.pitch_speed_rad_s = 0.0f;
#else
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
#if GIMBAL
        feedback.enabled = yaw.online && yaw.enabled && pitch.online && pitch.enabled;
#endif
#endif
        feedback.ins_valid = Gimbal_INS_Valid;
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
    }
}
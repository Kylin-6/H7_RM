#include "Gimbal.h"
#include "message_center.h"

static INS_State Gimbal_INS_State;
static bool Gimbal_INS_Valid = false;
static constexpr uint64_t GIMBAL_INS_MAX_AGE_US = 10000U;
static GimbalCmd Gimbal_Command;
static Subscriber<GimbalCmd> Gimbal_Command_Subscriber(
    MessageCenter::Gimbal_Command_Topic);
static Publisher<GimbalFeedback> Gimbal_Feedback_Publisher(
    MessageCenter::Gimbal_Feedback_Topic);
static uint8_t Gimbal_Message_Divider;
#if GIMBAL || LEGACY_INFANTRY
static GimbalMode Gimbal_Last_Mode = GimbalMode::DISABLED;
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
    Gimbal.Yaw_Speed_Command = 0.0f;
    Gimbal.Yaw_Mit_Kd = GIMBAL_YAW_MIT_KD_CENTER;
    Gimbal.Yaw_Mit_Torque_Feedforward = 0.0f;
    SpeedPlanning_Init(&Gimbal.Yaw_Speed_Planning, 0.0f);

    const bool initialized =
        Gimbal.Yaw_Motor.Init(&hfdcan1,
                              GIMBAL_YAW_MOTOR_CAN_ID,
                              GIMBAL_YAW_MOTOR_MASTER_ID,
                              Enum_DMMotor_Mode::MIT,
                              false,
                              GIMBAL_MOTOR_POSITION_MAX_RAD,
                              GIMBAL_MOTOR_VELOCITY_MAX_RAD_S,
                              GIMBAL_MOTOR_TORQUE_MAX_NM);
    if (initialized)
    {
        /* 上电默认失能，等 RobotCmd 的云台命令进入使能模式后再输出。 */
        (void)Gimbal.Yaw_Motor.Disable();
    }
}

void Gimbal_Loop(void)
{
    /* 摇杆目标速度：死区与指数整形已在 Communication 层完成。 */
    const float stick_speed_command = Gimbal_Constrain(
        Gimbal_Command.yaw_speed_rad_s, -GIMBAL_YAW_SPEED_MAX, GIMBAL_YAW_SPEED_MAX);

    /* 机体系 Z 轴角速度前馈，抑制底盘旋转对云台的耦合。 */
    float yaw_speed_command =
        stick_speed_command - GIMBAL_YAW_RATE_FEEDFORWARD_GAIN * Gimbal_INS_State.gyro_z_rad_s;
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

#elif GIMBAL

#include "QD4310.h"
#include "alg_pid.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include <cmath>

QDGimbal_t Gimbal;

float Gimbal_Clamp(float value, float minimum, float maximum)
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

/**
 * @brief Yaw 轴速度内环参数。
 *
 * 输入目标为角速度，反馈来自 INS 状态的 Z 轴角速度，输出作为 QD4310 电流指令。
 * 该环负责快速抑制速度误差；它的输出上限同时限制 Yaw 电机的最大控制电流。
 */
PID_InitTypeDef Yaw_Speed_PID_Init = {
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
 * @brief Yaw 轴角度外环参数。
 *
 * 输入目标为 Yaw 目标角度，反馈来自 INS 状态的欧拉角；输出不是电流，而是交给
 * Yaw 速度内环的目标角速度。Kp=32.00 是当前已使用的参数，不在本次注释修改中调整。
 */
PID_InitTypeDef Yaw_Angle_PID_Init = {
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
 * @brief Pitch 轴速度 PID 参数。
 *
 * 该组参数保留给手动速度控制；矩形追踪时 Pitch 直接使用 QD4310 内置位置环，
 * 不经过这组 PID。
 */
PID_InitTypeDef Pitch_Speed_PID_Init = {
    .K_P = 0.1f,
    .K_I = 0.0f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 0.0f,
    .Out_Max = 10000.0f,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief 预留的 Pitch 角度外环参数。
 *
 * 当前没有初始化和计算这组 PID；保留在此处供后续完成 Pitch 轴辨识、量纲确认
 * 和参数整定后使用。
 */
PID_InitTypeDef Pitch_Angle_PID_Init = {
    .K_P = 0.01f,
    .K_I = 0.0f,
    .K_D = 0.0f,
    .K_F = 0.0f,
    .I_Out_Max = 0.0f,
    .Out_Max = 10000.0f,
    .D_T = 0.001f,
    .Dead_Zone = 0.0f,
    .I_Variable_Speed_A = 0.0f,
    .I_Variable_Speed_B = 0.0f,
    .I_Separate_Threshold = 0.0f,
    .D_First = PID_D_First_DISABLE};

/**
 * @brief 初始化云台状态机、两台 QD4310 电机和已启用的 PID。
 *
 * Yaw 电机连接 FDCAN2，Pitch 电机连接 FDCAN1。初始化末尾会每 20 ms 检查一次
 * 两台电机的 enabled 标志并发送使能命令。最多等待2秒；超时会保留错误状态并
 * 返回，避免阻塞同一控制任务中的其他Application初始化。
 */
void Gimbal_Init(void)
{
    // 初始化云台状态机，并绑定两台电机的 CAN 总线和节点 ID。
    Gimbal.Gimbal_FSM.Init();
    QD4310_Init(&Gimbal.Yaw_Motor, YAW_ID, &hfdcan2);
    QD4310_Init(&Gimbal.Pitch_Motor, PITCH_ID, &hfdcan1);

    // Yaw 速度内环：角速度误差 -> 电流指令。
    Gimbal.Yaw_Speed_PID.Init(Yaw_Speed_PID_Init.K_P,
                              Yaw_Speed_PID_Init.K_I,
                              Yaw_Speed_PID_Init.K_D,
                              Yaw_Speed_PID_Init.K_F,
                              Yaw_Speed_PID_Init.I_Out_Max,
                              Yaw_Speed_PID_Init.Out_Max,
                              Yaw_Speed_PID_Init.D_T,
                              Yaw_Speed_PID_Init.Dead_Zone,
                              Yaw_Speed_PID_Init.I_Variable_Speed_A,
                              Yaw_Speed_PID_Init.I_Variable_Speed_B,
                              Yaw_Speed_PID_Init.I_Separate_Threshold,
                              Yaw_Speed_PID_Init.D_First);

    // Pitch 速度 PID：保留给手动速度模式，视觉追踪不使用它。
    Gimbal.Pitch_Speed_PID.Init(Pitch_Speed_PID_Init.K_P,
                                Pitch_Speed_PID_Init.K_I,
                                Pitch_Speed_PID_Init.K_D,
                                Pitch_Speed_PID_Init.K_F,
                                Pitch_Speed_PID_Init.I_Out_Max,
                                Pitch_Speed_PID_Init.Out_Max,
                                Pitch_Speed_PID_Init.D_T,
                                Pitch_Speed_PID_Init.Dead_Zone,
                                Pitch_Speed_PID_Init.I_Variable_Speed_A,
                                Pitch_Speed_PID_Init.I_Variable_Speed_B,
                                Pitch_Speed_PID_Init.I_Separate_Threshold,
                                Pitch_Speed_PID_Init.D_First);

    // Yaw 角度外环：角度误差 -> 目标角速度。
    Gimbal.Yaw_Angle_PID.Init(Yaw_Angle_PID_Init.K_P,
                              Yaw_Angle_PID_Init.K_I,
                              Yaw_Angle_PID_Init.K_D,
                              Yaw_Angle_PID_Init.K_F,
                              Yaw_Angle_PID_Init.I_Out_Max,
                              Yaw_Angle_PID_Init.Out_Max,
                              Yaw_Angle_PID_Init.D_T,
                              Yaw_Angle_PID_Init.Dead_Zone,
                              Yaw_Angle_PID_Init.I_Variable_Speed_A,
                              Yaw_Angle_PID_Init.I_Variable_Speed_B,
                              Yaw_Angle_PID_Init.I_Separate_Threshold,
                              Yaw_Angle_PID_Init.D_First);

    // Pitch_Angle_PID 暂未整定, 待 pitch 辨识后启用
    // Gimbal.Pitch_Angle_PID.Init(Pitch_Angle_PID_Init.K_P, ...);
    Gimbal.Target_Pitch_Angle = 0.0f;
    Gimbal.Target_Yaw_Angle = 0.0f;
    Gimbal.Target_Pitch_Speed = 10.0f;
    Gimbal.Target_Yaw_Speed = 0.0f;

    // 最多等待2秒。电机断线时必须返回，不能阻塞其他Application初始化。
    constexpr uint32_t GIMBAL_ENABLE_RETRY_COUNT = 100U;
    constexpr uint32_t GIMBAL_ENABLE_RETRY_DELAY_MS = 20U;
    for (uint32_t retry = 0U; retry < GIMBAL_ENABLE_RETRY_COUNT; ++retry)
    {
        if (Gimbal.Pitch_Motor.enabled && Gimbal.Yaw_Motor.enabled)
        {
            // 电机刚使能时锁住当前姿态，避免在第一帧视觉数据到达前跳向零点。
            INS_State ins_state;
            if (MessageCenter::INS_State_Topic.Read(ins_state) &&
                std::isfinite(ins_state.yaw_rad))
            {
                Gimbal.Target_Yaw_Angle = ins_state.yaw_rad;
            }
            if (std::isfinite(Gimbal.Pitch_Motor.angle))
            {
                Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
                    Gimbal.Pitch_Motor.angle,
                    GIMBAL_PITCH_MIN_ANGLE_RAD,
                    GIMBAL_PITCH_MAX_ANGLE_RAD);
            }
            Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_READY);
            return;
        }

        if (!Gimbal.Pitch_Motor.enabled)
        {
            Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_PITCH_ERROR);
            QD4310_Enable(&Gimbal.Pitch_Motor);
        }
        if (!Gimbal.Yaw_Motor.enabled)
        {
            Gimbal.Gimbal_FSM.Set_Status(Gimbal_Status_YAW_ERROR);
            QD4310_Enable(&Gimbal.Yaw_Motor);
        }
        osDelay(GIMBAL_ENABLE_RETRY_DELAY_MS);
    }

    // 超时后保留明确的故障轴状态并返回，让底盘、发射和RobotCmd继续初始化。
    Gimbal.Gimbal_FSM.Set_Status(!Gimbal.Pitch_Motor.enabled
        ? Gimbal_Status_PITCH_ERROR : Gimbal_Status_YAW_ERROR);
}

/**
 * @brief 设置两轴目标角度。
 * @param yaw_angle Yaw 目标角度，供 Yaw 角度外环使用。
 * @param pitch_angle Pitch 电机内置位置环目标角度，单位为弧度。
 */
void Gimbal_SetTargetAngle(float yaw_angle, float pitch_angle)
{
    Gimbal.Target_Yaw_Angle = yaw_angle;
    Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
        pitch_angle,
        GIMBAL_PITCH_MIN_ANGLE_RAD,
        GIMBAL_PITCH_MAX_ANGLE_RAD);
}

/**
 * @brief 设置两轴目标角速度。
 * @param yaw_speed Yaw 目标角速度；下一次循环会被 Yaw 角度外环输出覆盖。
 * @param pitch_speed Pitch 手动速度 PID 目标值；矩形追踪模式不使用。
 */
void Gimbal_SetTargetSpeed(float yaw_speed, float pitch_speed)
{
    Gimbal.Target_Yaw_Speed = yaw_speed;
    Gimbal.Target_Pitch_Speed = pitch_speed;
}

/**
 * @brief 执行一次云台控制计算并向两台电机下发命令。
 *
 * 当前控制路径：Yaw 使用“视觉误差 -> 目标角度 -> 角度 PID -> 目标角速度 ->
 * 速度 PID -> 电流指令”；Pitch 使用“视觉误差 -> 目标角度 -> QD4310 内置位置环”。
 * 视觉丢失时保持最后目标，不发送失能命令。
 */
void Gimbal_Loop(void)
{
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
    // Pitch 采用 QD4310 内置位置环；下发前再次限幅，手动目标也不能绕过机械范围。
    Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
        Gimbal.Target_Pitch_Angle,
        GIMBAL_PITCH_MIN_ANGLE_RAD,
        GIMBAL_PITCH_MAX_ANGLE_RAD);

    
    QD4310_SetAngle(&Gimbal.Pitch_Motor, Gimbal.Target_Pitch_Angle);
}

#endif

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
#if GIMBAL
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
#if LEGACY_INFANTRY
        if (command.mode == GimbalMode::DISABLED)
        {
            if (Gimbal_Last_Mode != GimbalMode::DISABLED)
            {
                (void)Gimbal.Yaw_Motor.Disable();
                Gimbal_Yaw_Output_Enabled = false;
            }
        }
        else if (Gimbal_Last_Mode == GimbalMode::DISABLED)
        {
            (void)Gimbal.Yaw_Motor.Enable();
            Gimbal_Yaw_Output_Enabled = true;
        }
        Gimbal_Last_Mode = command.mode;
#elif GIMBAL
        if (command.mode == GimbalMode::DISABLED)
        {
            if (Gimbal_Last_Mode != GimbalMode::DISABLED)
            {
                QD4310_Disable(&Gimbal.Yaw_Motor);
                QD4310_Disable(&Gimbal.Pitch_Motor);
            }
        }
        else
        {
            if (Gimbal_Last_Mode == GimbalMode::DISABLED)
            {
                QD4310_Enable(&Gimbal.Yaw_Motor);
                QD4310_Enable(&Gimbal.Pitch_Motor);
            }
            if (command.mode == GimbalMode::IMU)
            {
                Gimbal_SetTargetAngle(command.yaw_angle_rad,
                                      command.pitch_angle_rad);
                Gimbal_SetTargetSpeed(command.yaw_speed_rad_s,
                                      command.pitch_speed_rad_s);
            }
            else if (Gimbal_Last_Mode != GimbalMode::LOCK)
            {
                if (Gimbal_INS_Valid)
                {
                    Gimbal.Target_Yaw_Angle = Gimbal_INS_State.yaw_rad;
                }
                Gimbal.Target_Pitch_Angle = Gimbal_Clamp(
                    Gimbal.Pitch_Motor.angle,
                    GIMBAL_PITCH_MIN_ANGLE_RAD,
                    GIMBAL_PITCH_MAX_ANGLE_RAD);
            }
        }
        Gimbal_Last_Mode = command.mode;
#endif
    }

#if LEGACY_INFANTRY
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
    }
#elif GIMBAL
    /* 只有初始化完成且两轴就绪时才允许输出，故障状态不得继续下发控制量。 */
    if (Gimbal_Command.mode != GimbalMode::DISABLED &&
        Gimbal.Gimbal_FSM.Get_Now_Status_Serial() == Gimbal_Status_READY)
    {
        Gimbal_Loop();
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
        feedback.ins_valid = Gimbal_INS_Valid;
        feedback.enabled = Gimbal_Yaw_Output_Enabled;
#else
        feedback.yaw_rad = Gimbal_INS_State.yaw_rad;
        feedback.pitch_rad = Gimbal_INS_State.pitch_rad;
        feedback.yaw_speed_rad_s = Gimbal_INS_State.gyro_z_rad_s;
        feedback.pitch_speed_rad_s = Gimbal_INS_State.gyro_x_rad_s;
        feedback.ins_valid = Gimbal_INS_Valid;
#if GIMBAL
        feedback.enabled = Gimbal.Yaw_Motor.enabled && Gimbal.Pitch_Motor.enabled;
#endif
#endif
        Gimbal_Feedback_Publisher.Publish(feedback);
    }
}

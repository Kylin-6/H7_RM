/**
 * @file Shoot.cpp
 * @brief 摩擦轮与拨弹盘应用。
 *
 * 两种硬件实现由编译开关区分：
 * - 默认（`SHOOT`）：DJI 摩擦轮 + DJI 拨弹盘，参考 Meta-Embedded-NG 移植；
 * - `LEGACY_INFANTRY_GIMBAL`：老步兵云台板，DM3519 摩擦轮 + M2006 拨弹盘，
 *   移植自老步兵云台板工程（H7_RM），含扳机状态机、卡弹处理与热量估计。
 *
 * 两种实现共用 `ShootCmd` / `ShootFeedback` / `ShootEvent` 消息端点：老步兵云台板
 * 配置下 `shoot_mode` 表示扳机按下与否，`loader_speed_deg_s` 表示波轮映射后的拨弹盘
 * 输出速度（rad/s，与框架既有实现一致，详见 Application/README.md）。
 */

#include "Shoot.h"

#include "message_center.h"

static Subscriber<ShootCmd> Shoot_Command_Subscriber(
    MessageCenter::Shoot_Command_Topic);
static Publisher<ShootFeedback> Shoot_Feedback_Publisher(
    MessageCenter::Shoot_Feedback_Topic);
static ShootCmd Shoot_Command;
static ShootFeedback Shoot_Feedback;
static uint8_t Shoot_Feedback_Divider;

#if SHOOT && LEGACY_INFANTRY_GIMBAL

/* ==========================================================================
 * 老步兵云台板实现：DM3519 摩擦轮（FDCAN1）+ M2006 拨弹盘（FDCAN2）
 *
 * 与云台板原实现保持一致的语义：
 * - 摩擦轮是速度模式 DM 电机，扳机按下即正反转恒速，松扳机后仍延时 300 ms 继续转；
 * - 拨弹盘由「短按单发 / 长按连发」状态机驱动，连发速度来自波轮档位；
 * - 单发用角度环推一颗弹的距离，到位后锁住实际角度抑制回弹；
 * - 堵转按相电流阈值确认 300 ms 后回退 15 度；
 * - 热量按摩擦轮力矩突变点估计单发累积，超过上限后停止拨弹。
 * ========================================================================== */

#include "dji_motor.h"
#include "dmmotor.h"
#include "fdcan.h"
#include "stm32h7xx_hal.h"
#include "sys_timestamp.h"

#include <cmath>

namespace
{
constexpr float SHOOT_PI = 3.14159265358979323846f;

/* 硬件 ID 与云台板原工程一致。 */
constexpr uint8_t FRICTION_LEFT_ID = 0x07U;
constexpr uint16_t FRICTION_LEFT_FEEDBACK_ID = 0x027U;
constexpr uint8_t FRICTION_RIGHT_ID = 0x08U;
constexpr uint16_t FRICTION_RIGHT_FEEDBACK_ID = 0x028U;
constexpr uint8_t LOADER_ID = 1U;

/* 反馈与使能重试。 */
constexpr uint32_t FEEDBACK_TIMEOUT_MS = 100U;
constexpr uint32_t ENABLE_RETRY_MS = 100U;

/* 机构参数。 */
constexpr float FRICTION_SPEED_RAD_S = 25.0f;
constexpr float FRICTION_READY_TOLERANCE_RAD_S = 1.0f;
constexpr float DM3519_VELOCITY_MAX_RAD_S = 200.0f;
constexpr float DM3519_TORQUE_MAX_NM = 10.0f;
constexpr float DM3519_POSITION_MAX_RAD = 12.5f;
constexpr float LOADER_MAX_ROTOR_RPM = 4500.0f;
constexpr float M2006_GEAR_RATIO = 36.0f;
constexpr float M2006_RPM_PER_OUTPUT_RAD_S =
    M2006_GEAR_RATIO * 60.0f / (2.0f * SHOOT_PI);
constexpr float LOADER_SPEED_KP = 17.0f * M2006_RPM_PER_OUTPUT_RAD_S;
constexpr float LOADER_SPEED_KI =
    2.0f * M2006_RPM_PER_OUTPUT_RAD_S / 0.001f;
constexpr float LOADER_SINGLE_SPEED_KP = 10.0f * 180.0f / SHOOT_PI;
constexpr float LOADER_SINGLE_SPEED_KI = 1.0f * 180.0f / SHOOT_PI;
constexpr float LOADER_SINGLE_SPEED_LIMIT_RAD_S = 400.0f * SHOOT_PI / 180.0f;
constexpr float ONE_BULLET_OUTPUT_DEG = 35.0f;
constexpr float EXTERNAL_REDUCTION_RATIO = 54.74f / 25.16f;
constexpr float ONE_BULLET_MOTOR_OUTPUT_RAD =
    ONE_BULLET_OUTPUT_DEG * EXTERNAL_REDUCTION_RATIO * SHOOT_PI / 180.0f;
constexpr float SINGLE_DONE_ANGLE_RAD = 2.0f * SHOOT_PI / 180.0f;
constexpr uint32_t SINGLE_TIMEOUT_MS = 1000U;
constexpr uint32_t SINGLE_HOLD_MS = 100U;
constexpr uint32_t POST_SHOT_FRICTION_MS = 300U;
constexpr uint32_t LONG_PRESS_MS = 300U;

/* 安全参数，集中在一起便于按实车标定。 */
constexpr int16_t JAM_CURRENT_THRESHOLD = 3800;
constexpr uint32_t JAM_CONFIRM_MS = 300U;
constexpr uint32_t JAM_HANDLE_MS = 200U;
constexpr float JAM_BACKOFF_RAD = 15.0f * EXTERNAL_REDUCTION_RATIO * SHOOT_PI / 180.0f;
constexpr float HEAT_LEFT_TORQUE_THRESHOLD_NM = -0.6f;
constexpr float HEAT_RIGHT_TORQUE_THRESHOLD_NM = 0.5f;
constexpr uint32_t HEAT_CONFIRM_MS = 20U;
constexpr float HEAT_PER_SHOT = 10.0f;
constexpr float HEAT_COOL_PER_SECOND = 35.0f;
constexpr float HEAT_LIMIT = 220.0f;

enum class FireState : uint8_t
{
    IDLE,
    PRESSING,
    SINGLE,
    BURST,
    POST_SHOT,
};

enum class JamState : uint8_t
{
    NORMAL,
    SUSPECT,
    HANDLING,
};

Class_DMMotor Friction_Left;
Class_DMMotor Friction_Right;
Class_DJIMotor Loader;
Class_DJIMotor_Group Loader_Group;

FireState Fire_State;
JamState Jam_State;
uint32_t Press_Start_Tick;
uint32_t Single_Start_Tick;
uint32_t Single_Hold_Start_Tick;
uint32_t Post_Shot_Start_Tick;
uint32_t Jam_Start_Tick;
uint32_t Jam_Handle_Start_Tick;
uint32_t Heat_Start_Tick;
uint32_t Last_Loop_Tick;
uint32_t Last_Enable_Tick;
float Single_Target_Angle;
bool Single_Holding;
float Jam_Target_Angle;
float Estimated_Heat;
bool Heat_Suspect;
bool Heat_Latched;
bool Shoot_Initialized;

PID_InitTypeDef MakePID(float kp, float ki, float kd,
                        float integral_limit, float output_limit)
{
    PID_InitTypeDef pid{};
    pid.K_P = kp;
    pid.K_I = ki;
    pid.K_D = kd;
    pid.I_Out_Max = integral_limit;
    pid.Out_Max = output_limit;
    pid.D_T = 0.001f;
    return pid;
}

bool LoaderFeedbackFresh(void)
{
    const uint64_t timestamp = Loader.Get_Last_Feedback_Timestamp_Us();
    return timestamp != 0U &&
           SYS_Timestamp.Get_Now_Microsecond() - timestamp <=
               static_cast<uint64_t>(FEEDBACK_TIMEOUT_MS) * 1000U;
}

bool FrictionReady(void)
{
    return Friction_Left.IsOnline() &&
           Friction_Right.IsOnline() &&
           std::fabs(Friction_Left.feedback.velocity + FRICTION_SPEED_RAD_S) <=
               FRICTION_READY_TOLERANCE_RAD_S &&
           std::fabs(Friction_Right.feedback.velocity - FRICTION_SPEED_RAD_S) <=
               FRICTION_READY_TOLERANCE_RAD_S;
}

void SetLoaderStopped(void)
{
    // 停火时直接输出零电流，不用高增益速度环在零速附近反复制动。
    Loader.speed_pid.Set_Integral_Error(0.0f);
    Loader.angle_pid.Set_Integral_Error(0.0f);
    Loader.Set_Outer_Loop(DJI_MOTOR_OPEN_LOOP);
    Loader_Group.Control(0.0f);
}

void StopAll(void)
{
    Friction_Left.SetSpeed(0.0f);
    Friction_Right.SetSpeed(0.0f);
    SetLoaderStopped();
    Fire_State = FireState::IDLE;
    Single_Holding = false;
    Jam_State = JamState::NORMAL;
    Heat_Suspect = false;
    Heat_Latched = false;
}

void UpdateHeat(uint32_t now, bool loader_active)
{
    Estimated_Heat -= HEAT_COOL_PER_SECOND *
                      static_cast<float>(now - Last_Loop_Tick) * 0.001f;
    if (Estimated_Heat < 0.0f)
        Estimated_Heat = 0.0f;

    const bool spike = loader_active &&
                       Friction_Left.feedback.torque <= HEAT_LEFT_TORQUE_THRESHOLD_NM &&
                       Friction_Right.feedback.torque >= HEAT_RIGHT_TORQUE_THRESHOLD_NM;
    if (!spike)
    {
        Heat_Suspect = false;
        Heat_Latched = false;
    }
    else if (!Heat_Latched)
    {
        if (!Heat_Suspect)
        {
            Heat_Suspect = true;
            Heat_Start_Tick = now;
        }
        else if (now - Heat_Start_Tick >= HEAT_CONFIRM_MS)
        {
            Estimated_Heat += HEAT_PER_SHOT;
            Heat_Latched = true;
            Heat_Suspect = false;
        }
    }
}

bool UpdateJam(uint32_t now, bool loader_active)
{
    if (Jam_State == JamState::HANDLING)
    {
        Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
        Loader_Group.Control(Jam_Target_Angle);
        if (now - Jam_Handle_Start_Tick >= JAM_HANDLE_MS)
            Jam_State = JamState::NORMAL;
        return true;
    }

    if (!loader_active ||
        std::abs(Loader.feedback.current_raw) <= JAM_CURRENT_THRESHOLD)
    {
        Jam_State = JamState::NORMAL;
        return false;
    }

    if (Jam_State == JamState::NORMAL)
    {
        Jam_State = JamState::SUSPECT;
        Jam_Start_Tick = now;
    }
    else if (now - Jam_Start_Tick >= JAM_CONFIRM_MS)
    {
        Jam_State = JamState::HANDLING;
        Jam_Handle_Start_Tick = now;
        Jam_Target_Angle = Loader.feedback.output_total_angle - JAM_BACKOFF_RAD;
        Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
        Loader_Group.Control(Jam_Target_Angle);
        return true;
    }
    return false;
}

/** 初始化两台摩擦轮与拨弹盘；返回 false 时上层保持不控制硬件。 */
bool Shoot_Legacy_Init(void)
{
    if (Shoot_Initialized)
        return true;

    const bool left_ok = Friction_Left.Init(&hfdcan1, FRICTION_LEFT_ID,
                                            FRICTION_LEFT_FEEDBACK_ID,
                                            Enum_DMMotor_Mode::SPEED, false,
                                            DM3519_POSITION_MAX_RAD,
                                            DM3519_VELOCITY_MAX_RAD_S,
                                            DM3519_TORQUE_MAX_NM);
    const bool right_ok = Friction_Right.Init(&hfdcan1, FRICTION_RIGHT_ID,
                                              FRICTION_RIGHT_FEEDBACK_ID,
                                              Enum_DMMotor_Mode::SPEED, false,
                                              DM3519_POSITION_MAX_RAD,
                                              DM3519_VELOCITY_MAX_RAD_S,
                                              DM3519_TORQUE_MAX_NM);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = &hfdcan2;
    loader_config.can_id = LOADER_ID;
    loader_config.motor_type = Enum_DJIMotor_Type::M2006;
    loader_config.gear_ratio = M2006_GEAR_RATIO;
    loader_config.feedback_timeout_ms = FEEDBACK_TIMEOUT_MS;
    loader_config.close_loop = DJI_MOTOR_SPEED_LOOP | DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 原工程 PID 以转子 rpm 计算且积分未乘 dt；换算到当前输出轴 rad/s、1 ms PID。
    loader_config.speed_pid = MakePID(LOADER_SPEED_KP, LOADER_SPEED_KI, 0.0f,
                                      6000.0f, 10000.0f);
    loader_config.angle_pid = MakePID(10.0f, 0.0f, 0.0f, 0.0f,
                                      LOADER_SINGLE_SPEED_LIMIT_RAD_S);

    const bool loader_ok = Loader.Init(loader_config) && Loader_Group.Init(&Loader);

    Press_Start_Tick = 0U;
    Single_Start_Tick = 0U;
    Single_Hold_Start_Tick = 0U;
    Post_Shot_Start_Tick = 0U;
    Jam_Start_Tick = 0U;
    Jam_Handle_Start_Tick = 0U;
    Heat_Start_Tick = 0U;
    Single_Target_Angle = 0.0f;
    Single_Holding = false;
    Jam_Target_Angle = 0.0f;
    Estimated_Heat = 0.0f;
    Heat_Suspect = false;
    Heat_Latched = false;

    Shoot_Initialized = left_ok && right_ok && loader_ok;
    Last_Loop_Tick = HAL_GetTick();
    Last_Enable_Tick = Last_Loop_Tick;
    if (Shoot_Initialized)
    {
        // 与云台板一致：电机内部已预先配置为速度模式，上电只发送使能帧。
        Friction_Left.Enable();
        Friction_Right.Enable();
        StopAll();
    }
    return Shoot_Initialized;
}

/**
 * @brief 老步兵云台板的 1 kHz 发射控制。
 *
 * 输入来自 ShootCmd：`shoot_mode == ON` 表示扳机按下，`loader_speed_deg_s` 表示
 * 波轮映射后的连发拨弹盘速度（rad/s）。通道失效由 Communication 解除 shoot_mode，
 * 本函数随即停火并复位状态机。
 */
void Shoot_Legacy_Loop(void)
{
    const uint32_t now = HAL_GetTick();
    const bool trigger_pressed = Shoot_Command.shoot_mode == ShootMode::ON;

    /*
     * 原工程用电机上报的使能状态决定是否重发使能帧；框架的 Class_DMMotor 只暴露
     * 在线状态，因此改为「反馈离线即重发使能」，重发间隔与原工程一致。
     */
    if ((!Friction_Left.IsOnline() || !Friction_Right.IsOnline()) &&
        now - Last_Enable_Tick >= ENABLE_RETRY_MS)
    {
        if (!Friction_Left.IsOnline())
            Friction_Left.Enable();
        if (!Friction_Right.IsOnline())
            Friction_Right.Enable();
        Last_Enable_Tick = now;
    }

    /* 扳机状态机：短按单发、长按连发、松扳机后摩擦轮延时停转。 */
    if (Fire_State == FireState::IDLE && trigger_pressed)
    {
        Fire_State = FireState::PRESSING;
        Press_Start_Tick = now;
    }
    else if (Fire_State == FireState::PRESSING)
    {
        if (trigger_pressed && now - Press_Start_Tick >= LONG_PRESS_MS)
            Fire_State = FireState::BURST;
        else if (!trigger_pressed)
        {
            Fire_State = FireState::SINGLE;
            Single_Target_Angle = Loader.feedback.output_total_angle +
                                  ONE_BULLET_MOTOR_OUTPUT_RAD;
            Single_Start_Tick = now;
            Single_Holding = false;
        }
    }
    else if (Fire_State == FireState::BURST && !trigger_pressed)
    {
        Fire_State = FireState::IDLE;
    }
    else if (Fire_State == FireState::POST_SHOT && trigger_pressed)
    {
        Fire_State = FireState::PRESSING;
        Press_Start_Tick = now;
    }
    else if (Fire_State == FireState::POST_SHOT &&
             now - Post_Shot_Start_Tick >= POST_SHOT_FRICTION_MS)
    {
        Fire_State = FireState::IDLE;
    }

    /* 反馈瞬时丢失时仅停止电机，保留已识别的短按单发请求。 */
    if (!Friction_Left.IsOnline() || !Friction_Right.IsOnline())
    {
        Friction_Left.SetSpeed(0.0f);
        Friction_Right.SetSpeed(0.0f);
        SetLoaderStopped();
        Last_Loop_Tick = now;
        return;
    }

    const bool firing_requested = Fire_State != FireState::IDLE;
    Friction_Left.SetSpeed(firing_requested ? -FRICTION_SPEED_RAD_S : 0.0f);
    Friction_Right.SetSpeed(firing_requested ? FRICTION_SPEED_RAD_S : 0.0f);

    bool loader_active = false;
    if (LoaderFeedbackFresh() && FrictionReady() && Estimated_Heat < HEAT_LIMIT)
    {
        if (Fire_State == FireState::SINGLE)
        {
            loader_active = true;
            Loader.speed_pid.Set_K_P(LOADER_SINGLE_SPEED_KP);
            Loader.speed_pid.Set_K_I(LOADER_SINGLE_SPEED_KI);
            Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
            Loader_Group.Control(Single_Target_Angle);
            if (!Single_Holding &&
                (std::fabs(Single_Target_Angle - Loader.feedback.output_total_angle) <=
                     SINGLE_DONE_ANGLE_RAD ||
                 now - Single_Start_Tick >= SINGLE_TIMEOUT_MS))
            {
                // 锁住结束时的实际角度，抑制惯性超调和机械回弹。
                Single_Target_Angle = Loader.feedback.output_total_angle;
                Single_Hold_Start_Tick = now;
                Single_Holding = true;
            }
            else if (Single_Holding && now - Single_Hold_Start_Tick >= SINGLE_HOLD_MS)
            {
                Single_Holding = false;
                Post_Shot_Start_Tick = now;
                Fire_State = FireState::POST_SHOT;
            }
        }
        else if (Fire_State == FireState::BURST)
        {
            loader_active = true;
            Loader.speed_pid.Set_K_P(LOADER_SPEED_KP);
            Loader.speed_pid.Set_K_I(LOADER_SPEED_KI);
            Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
            Loader_Group.Control(Shoot_Command.loader_speed_deg_s);
        }
        else
        {
            SetLoaderStopped();
        }
    }
    else
    {
        SetLoaderStopped();
    }

    if (UpdateJam(now, loader_active))
        loader_active = true;
    UpdateHeat(now, loader_active);
    Last_Loop_Tick = now;
}

/** 把云台板发射状态写入框架的统一反馈结构。 */
void Shoot_Legacy_UpdateFeedback(void)
{
    Shoot_Feedback.friction_left_speed_deg_s = Friction_Left.feedback.velocity;
    Shoot_Feedback.friction_right_speed_deg_s = Friction_Right.feedback.velocity;
    Shoot_Feedback.loader_angle_deg = Loader.feedback.output_total_angle;
    Shoot_Feedback.loader_speed_deg_s = Loader.feedback.output_speed;
    Shoot_Feedback.enabled = Shoot_Initialized;
    Shoot_Feedback.online = Friction_Left.IsOnline() &&
                            Friction_Right.IsOnline() &&
                            Loader.online;
}
} // namespace

#endif /* SHOOT && LEGACY_INFANTRY_GIMBAL */

#if SHOOT && !LEGACY_INFANTRY_GIMBAL

/* ==========================================================================
 * 默认实现：DJI 摩擦轮 + DJI 拨弹盘
 * ========================================================================== */

#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>

static constexpr float SHOOT_DEFAULT_FRICTION_SPEED_DEG_S = 40000.0f;
static constexpr float SHOOT_DEFAULT_RATE_HZ = 10.0f;
static constexpr float SHOOT_ONE_BULLET_ANGLE_DEG = 36.0f;
static constexpr float SHOOT_REVERSE_SPEED_DEG_S = -360.0f;

static Class_DJIMotor Shoot_Friction_Left;
static Class_DJIMotor Shoot_Friction_Right;
static Class_DJIMotor Shoot_Loader;
static Class_DJIMotor_Group Shoot_Friction_Group;
static Class_DJIMotor_Group Shoot_Loader_Group;
static bool Shoot_Initialized;
static bool Shoot_Output_Enabled;
static bool Shoot_Event_Angle_Active;
static float Shoot_Loader_Angle_Target;

static PID_InitTypeDef Shoot_MakePID(float kp, float ki, float kd,
                                    float integral_limit, float output_limit)
{
    PID_InitTypeDef pid{};
    pid.K_P = kp;
    pid.K_I = ki;
    pid.K_D = kd;
    pid.I_Out_Max = integral_limit;
    pid.Out_Max = output_limit;
    pid.D_T = 0.001f;
    return pid;
}

static void Shoot_SetEnabled(bool enabled)
{
    if (enabled == Shoot_Output_Enabled)
    {
        return;
    }
    Shoot_Output_Enabled = enabled;
    if (enabled)
    {
        Shoot_Friction_Group.Enable();
        Shoot_Loader_Group.Enable();
    }
    else
    {
        Shoot_Friction_Group.Disable();
        Shoot_Loader_Group.Disable();
    }
}

static void Shoot_ApplyCommand(void)
{
    /* ShootMode 是总使能；关闭后摩擦轮和拨弹盘都停止主动输出。 */
    const bool enabled = Shoot_Command.shoot_mode == ShootMode::ON;
    Shoot_SetEnabled(enabled);
    if (!enabled)
    {
        Shoot_Event_Angle_Active = false;
        return;
    }

    float friction_reference = 0.0f;
    if (Shoot_Command.friction_mode == FrictionMode::ON)
    {
        friction_reference = Shoot_Command.friction_speed_deg_s > 0.0f
            ? Shoot_Command.friction_speed_deg_s
            : SHOOT_DEFAULT_FRICTION_SPEED_DEG_S;
    }
    Shoot_Friction_Group.Control(friction_reference, friction_reference);

    float loader_reference = 0.0f;
    switch (Shoot_Command.loader_mode)
    {
    case LoaderMode::BURST:
    {
        Shoot_Event_Angle_Active = false;
        Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        const float rate = Shoot_Command.shoot_rate_hz > 0.0f
            ? Shoot_Command.shoot_rate_hz : SHOOT_DEFAULT_RATE_HZ;
        loader_reference = Shoot_Command.loader_speed_deg_s != 0.0f
            ? Shoot_Command.loader_speed_deg_s
            : rate * SHOOT_ONE_BULLET_ANGLE_DEG;
        break;
    }

    case LoaderMode::REVERSE:
        Shoot_Event_Angle_Active = false;
        Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        loader_reference = Shoot_Command.loader_speed_deg_s != 0.0f
            ? -std::fabs(Shoot_Command.loader_speed_deg_s)
            : SHOOT_REVERSE_SPEED_DEG_S;
        break;

    case LoaderMode::STOP:
    default:
    {
        ShootEvent event;
        if (MessageCenter::Shoot_Event_Queue.Pop(event))
        {
            if (!Shoot_Event_Angle_Active)
            {
                Shoot_Loader_Angle_Target =
                    Shoot_Loader.feedback.output_total_angle;
            }
            const float bullet_count =
                event.type == ShootEventType::ShootTriple ? 3.0f : 1.0f;
            Shoot_Loader_Angle_Target +=
                bullet_count * SHOOT_ONE_BULLET_ANGLE_DEG;
            Shoot_Event_Angle_Active = true;
        }
        if (Shoot_Event_Angle_Active)
        {
            Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
            loader_reference = Shoot_Loader_Angle_Target;
        }
        else
        {
            Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
            loader_reference = 0.0f;
        }
        break;
    }
    }

    Shoot_Loader_Group.Control(loader_reference);
}

static void Shoot_UpdateFeedback(void)
{
    Shoot_Feedback.friction_left_speed_deg_s =
        Shoot_Friction_Left.feedback.output_speed;
    Shoot_Feedback.friction_right_speed_deg_s =
        Shoot_Friction_Right.feedback.output_speed;
    Shoot_Feedback.loader_angle_deg = Shoot_Loader.feedback.output_total_angle;
    Shoot_Feedback.loader_speed_deg_s = Shoot_Loader.feedback.output_speed;
    Shoot_Feedback.enabled = Shoot_Output_Enabled;
    Shoot_Feedback.online = Shoot_Friction_Left.online &&
                            Shoot_Friction_Right.online &&
                            Shoot_Loader.online;
}

#endif /* SHOOT && !LEGACY_INFANTRY_GIMBAL */

bool Shoot_Init(void)
{
    Shoot_Command = {};
    Shoot_Feedback = {};
    Shoot_Feedback_Divider = 0U;

#if SHOOT && LEGACY_INFANTRY_GIMBAL
    return Shoot_Legacy_Init();
#elif SHOOT
    Struct_DJIMotor_Init_Config friction_config{};
    friction_config.hfdcan = &hfdcan3;
    friction_config.motor_type = Enum_DJIMotor_Type::M3508;
    friction_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.speed_pid = Shoot_MakePID(7.5f, 5.0f, 0.0f, 16000.0f, 16000.0f);

    friction_config.can_id = 3U;
    const bool left_initialized = Shoot_Friction_Left.Init(friction_config);
    friction_config.can_id = 2U;
    friction_config.reverse = true;
    const bool right_initialized = Shoot_Friction_Right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = &hfdcan3;
    loader_config.can_id = 8U;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508;
    loader_config.close_loop = DJI_MOTOR_CURRENT_LOOP |
                               DJI_MOTOR_SPEED_LOOP |
                               DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    loader_config.current_pid = Shoot_MakePID(1.0f, 50.0f, 0.0f, 12000.0f, 12000.0f);
    loader_config.speed_pid = Shoot_MakePID(7.5f, 20.0f, 0.0f, 12000.0f, 12000.0f);
    loader_config.angle_pid = Shoot_MakePID(10.0f, 0.0f, 0.0f, 0.0f, 360.0f);
    const bool loader_initialized = Shoot_Loader.Init(loader_config);

    Shoot_Initialized = left_initialized && right_initialized && loader_initialized &&
        Shoot_Friction_Group.Init(&Shoot_Friction_Left, &Shoot_Friction_Right) &&
        Shoot_Loader_Group.Init(&Shoot_Loader);
    Shoot_Output_Enabled = true;
    if (Shoot_Initialized)
    {
        Shoot_SetEnabled(false);
    }
    Shoot_Event_Angle_Active = false;
    Shoot_Loader_Angle_Target = 0.0f;
    return Shoot_Initialized;
#else
    return true;
#endif
}

void Shoot_Update(void)
{
    /* 每个控制周期读取最新命令；没有新消息时继续执行上一帧。 */
    ShootCmd command;
    if (Shoot_Command_Subscriber.Read(command))
    {
        Shoot_Command = command;
    }

    if (Shoot_Command.shoot_mode == ShootMode::OFF)
    {
        ShootEvent discarded_event;
        size_t pending_events = MessageCenter::Shoot_Event_Queue.Size();
        while (pending_events-- > 0U &&
               MessageCenter::Shoot_Event_Queue.Pop(discarded_event))
        {
        }
    }

#if SHOOT && LEGACY_INFANTRY_GIMBAL
    /* 老步兵云台板：扳机沿与长短按判定都在发射状态机内部，不使用事件队列。 */
    if (Shoot_Initialized)
    {
        Shoot_Legacy_Loop();
        Shoot_Legacy_UpdateFeedback();
    }
#elif SHOOT
    if (Shoot_Initialized)
    {
        Shoot_ApplyCommand();
        Shoot_UpdateFeedback();
    }
#endif

    /* 控制按 1 kHz 更新，应用层反馈降频到 100 Hz。 */
    Shoot_Feedback_Divider++;
    if (Shoot_Feedback_Divider >= 10U)
    {
        Shoot_Feedback_Divider = 0U;
        Shoot_Feedback_Publisher.Publish(Shoot_Feedback);
    }
}

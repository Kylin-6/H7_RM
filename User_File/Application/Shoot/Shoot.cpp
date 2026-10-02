/**
 * @file Shoot.cpp
 * @brief 摩擦轮与拨弹盘应用：老步兵云台板路径与框架通用路径。
 *
 * - `SHOOT && LEGACY_INFANTRY_GIMBAL`：老步兵云台板，DM3519 摩擦轮 + M2006 拨弹盘，
 *   移植自云台板原工程（H7_RM），含扳机状态机、单发/连发、卡弹检测回退、
 *   Post-shot friction 与热量估计，控制行为与实机验证一致。
 * - `SHOOT && !LEGACY_INFANTRY_GIMBAL`：RoboMaster_H7 框架通用实现，私有上下文
 *   收拢运行状态，M3508/C620 直驱摩擦轮（gear_ratio=1）默认目标 25 rad/s。
 *   未直接移植热量限制和堵转阈值：依赖实车机构与裁判系统数据。
 * 两条路径由编译期宏互斥选择。
 */

#include "Shoot.h"
#include "../physical_units.h"
#include "board_config.h"

#include "message_center.h"


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

#include "alg_fsm.h"
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
/* 拨弹盘直连 M2006 减速箱输出轴，7 个弹位均布一圈。 */
constexpr float ONE_BULLET_OUTPUT_DEG = 360.0f / 7.0f;
constexpr float ONE_BULLET_MOTOR_OUTPUT_RAD =
    ONE_BULLET_OUTPUT_DEG * SHOOT_PI / 180.0f;
constexpr float SINGLE_DONE_ANGLE_RAD = 2.0f * SHOOT_PI / 180.0f;
constexpr uint32_t SINGLE_TIMEOUT_MS = 1000U;
constexpr uint32_t SINGLE_HOLD_MS = 100U;
constexpr uint32_t POST_SHOT_FRICTION_MS = 300U;
constexpr uint32_t LONG_PRESS_MS = 300U;

/* 安全参数，集中在一起便于按实车标定。 */
constexpr int16_t JAM_CURRENT_THRESHOLD = 3800;
constexpr uint32_t JAM_CONFIRM_MS = 300U;
constexpr uint32_t JAM_HANDLE_MS = 200U;
constexpr float JAM_BACKOFF_RAD = 15.0f * SHOOT_PI / 180.0f;
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

/* 卡弹状态序号：作 Class_FSM 的状态编号使用，超时判定用框架的
 * Count_Time 周期计数（1 kHz 调用时单位即 ms），替代手写 tick 差值。 */
enum JamState : uint8_t
{
    JAM_NORMAL = 0,
    JAM_SUSPECT,
    JAM_HANDLING,
};

Class_FSM<3> Jam_FSM;

Class_DMMotor Friction_Left;
Class_DMMotor Friction_Right;
Class_DJIMotor Loader;
Class_DJIMotor_Group Loader_Group;

FireState Fire_State;
uint32_t Press_Start_Tick;
uint32_t Single_Start_Tick;
uint32_t Single_Hold_Start_Tick;
uint32_t Post_Shot_Start_Tick;
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
           Friction_Left.IsEnabled() &&
           Friction_Right.IsEnabled() &&
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
    Jam_FSM.Set_Status(JAM_NORMAL);
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

/**
 * @brief 卡弹状态机（框架 Class_FSM 驱动）。
 *
 * NORMAL --(电流超阈值)--> SUSPECT --(持续 300 ms)--> HANDLING --(回退 200 ms)--> NORMAL；
 * SUSPECT 期间条件消失直接回 NORMAL。各状态驻留时长由 Count_Time 周期计数判定，
 * 进入状态的清零动作由 Set_Status 完成，与原手写 tick 差值判定逐拍等价。
 *
 * @param loader_active 本周期拨弹盘是否在出弹。
 * @return true 表示本周期由卡弹状态机接管拨弹盘（回退或保持回退）。
 */
bool UpdateJam(bool loader_active)
{
    switch (Jam_FSM.Get_Now_Status_Serial())
    {
    case JAM_HANDLING:
        Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
        Loader_Group.Control(Jam_Target_Angle);
        if (Jam_FSM.Status[JAM_HANDLING].Count_Time >= JAM_HANDLE_MS)
            Jam_FSM.Set_Status(JAM_NORMAL);
        return true;

    case JAM_SUSPECT:
        if (!loader_active ||
            std::abs(Loader.feedback.current_raw) <= JAM_CURRENT_THRESHOLD)
        {
            Jam_FSM.Set_Status(JAM_NORMAL);
            return false;
        }
        if (Jam_FSM.Status[JAM_SUSPECT].Count_Time >= JAM_CONFIRM_MS)
        {
            Jam_FSM.Set_Status(JAM_HANDLING);
            Jam_Target_Angle = Loader.feedback.output_total_angle - JAM_BACKOFF_RAD;
            Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
            Loader_Group.Control(Jam_Target_Angle);
            return true;
        }
        return false;

    case JAM_NORMAL:
    default:
        if (loader_active &&
            std::abs(Loader.feedback.current_raw) > JAM_CURRENT_THRESHOLD)
        {
            Jam_FSM.Set_Status(JAM_SUSPECT);
        }
        return false;
    }
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
    Jam_FSM.Init(JAM_NORMAL);
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
        Friction_Left.RequestEnabled(true);
        Friction_Right.RequestEnabled(true);
        StopAll();
    }
    return Shoot_Initialized;
}

/**
 * @brief 老步兵云台板的 1 kHz 发射控制。
 *
 * 输入来自 ShootCmd：`shoot_mode == ON` 表示扳机按下，`loader_speed_rad_s` 表示
 * 波轮映射后的连发拨弹盘速度（rad/s）。通道失效由 Communication 解除 shoot_mode，
 * 本函数随即停火并复位状态机。
 */
void Shoot_Legacy_Loop(void)
{
    const uint32_t now = HAL_GetTick();
    const bool trigger_pressed = Shoot_Command.shoot_mode == ShootMode::ON;

    /* 在线只表示收到反馈；必须按驱动器状态码确认使能，未使能时周期重发。 */
    if ((!Friction_Left.IsOnline() || !Friction_Left.IsEnabled() ||
         !Friction_Right.IsOnline() || !Friction_Right.IsEnabled()) &&
        now - Last_Enable_Tick >= ENABLE_RETRY_MS)
    {
        if (!Friction_Left.IsOnline() || !Friction_Left.IsEnabled())
            Friction_Left.RequestEnabled(true);
        if (!Friction_Right.IsOnline() || !Friction_Right.IsEnabled())
            Friction_Right.RequestEnabled(true);
        Last_Enable_Tick = now;
    }

    /* 卡弹 FSM 状态驻留计数（1 kHz 下 Count_Time 单位即 ms），先于转移判定自增。 */
    Jam_FSM.TIM_Calculate_PeriodElapsedCallback();

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
            Loader_Group.Control(Shoot_Command.loader_speed_rad_s);
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

    if (UpdateJam(loader_active))
        loader_active = true;
    UpdateHeat(now, loader_active);
    Last_Loop_Tick = now;
}

/** 把云台板发射状态写入框架的统一反馈结构（速度 rad/s、角度 rad）。 */
void Shoot_Legacy_UpdateFeedback(void)
{
    Shoot_Feedback.friction_left_speed_rad_s = Friction_Left.feedback.velocity;
    Shoot_Feedback.friction_right_speed_rad_s = Friction_Right.feedback.velocity;
    Shoot_Feedback.loader_angle_rad = Loader.feedback.output_total_angle;
    Shoot_Feedback.loader_speed_rad_s = Loader.feedback.output_speed;
    Shoot_Feedback.enabled = Shoot_Initialized;
    Shoot_Feedback.online = Friction_Left.IsOnline() &&
                            Friction_Right.IsOnline() &&
                            Loader.IsOnline();
}
} // namespace

extern "C" void Shoot_GetDebug(float *initialized,
                               float *left_feedback,
                               float *right_feedback,
                               float *loader_feedback,
                               float *friction_ready,
                               float *left_velocity,
                               float *right_velocity,
                               float *left_target,
                               float *right_target,
                               float *fire_state,
                               float *press_duration_ms,
                               float *left_motor_state,
                               float *right_motor_state)
{
    if (initialized != nullptr)
        *initialized = Shoot_Initialized ? 1.0f : 0.0f;
    if (left_feedback != nullptr)
        *left_feedback = Friction_Left.IsOnline() ? 1.0f : 0.0f;
    if (right_feedback != nullptr)
        *right_feedback = Friction_Right.IsOnline() ? 1.0f : 0.0f;
    if (loader_feedback != nullptr)
        *loader_feedback = Loader.IsOnline() ? 1.0f : 0.0f;
    if (friction_ready != nullptr)
        *friction_ready = FrictionReady() ? 1.0f : 0.0f;
    if (left_velocity != nullptr)
        *left_velocity = Friction_Left.feedback.velocity;
    if (right_velocity != nullptr)
        *right_velocity = Friction_Right.feedback.velocity;
    /* 摩擦轮指令方向与云台板原实现一致：左轮取负、右轮取正。 */
    const bool friction_commanded = (Fire_State != FireState::IDLE);
    if (left_target != nullptr)
        *left_target = friction_commanded ? -FRICTION_SPEED_RAD_S : 0.0f;
    if (right_target != nullptr)
        *right_target = friction_commanded ? FRICTION_SPEED_RAD_S : 0.0f;
    if (fire_state != nullptr)
        *fire_state = static_cast<float>(Fire_State);
    if (press_duration_ms != nullptr)
        *press_duration_ms = (Fire_State == FireState::PRESSING)
                                 ? static_cast<float>(HAL_GetTick() - Press_Start_Tick)
                                 : 0.0f;
    if (left_motor_state != nullptr)
        *left_motor_state = static_cast<float>(Friction_Left.feedback.state);
    if (right_motor_state != nullptr)
        *right_motor_state = static_cast<float>(Friction_Right.feedback.state);
}

extern "C" void Shoot_GetLoaderDebug(Struct_Legacy_Loader_Debug *debug)
{
    if (debug == nullptr)
        return;

    const uint64_t timestamp_us = Loader.Get_Last_Feedback_Timestamp_Us();
    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    debug->encoder = static_cast<float>(Loader.feedback.encoder);
    debug->rotor_total_angle_degree = Loader.feedback.rotor_total_angle_degree;
    debug->output_total_angle_degree = Loader.feedback.output_total_angle_degree;
    debug->rotor_speed_rad_s = Loader.feedback.rotor_speed;
    debug->output_speed_rad_s = Loader.feedback.output_speed;
    debug->current_raw = static_cast<float>(Loader.feedback.current_raw);
    debug->feedback_age_ms = timestamp_us == 0U
                                 ? -1.0f
                                 : static_cast<float>(now_us - timestamp_us) * 0.001f;
    debug->speed_pid_out = Loader.feedback.pid.speed.out;
}

#endif /* SHOOT && LEGACY_INFANTRY_GIMBAL */

#if !LEGACY_INFANTRY_GIMBAL

/**
 * @file Shoot.cpp
 * @brief 摩擦轮与拨弹盘应用，参考 Meta-Embedded-NG 移植。
 *
 * 未直接移植热量限制和堵转阈值：这些参数依赖实车机构与裁判系统数据，当前
 * 工程尚不具备可靠标定条件。
 * 摩擦轮为 M3508/C620 直驱（gear_ratio=1），默认目标 25 rad/s。
 * 当前无就绪、卡弹回退、热量/裁判互锁或完整 FEEDING 状态机。
 */

#include "Shoot.h"
#include "Shoot_Config.h"
#include "board_config.h"

#include "message_center.h"

#if SHOOT
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
#endif

namespace
{
struct ShootContext
{
    Subscriber<ShootCmd> command_subscriber{MessageCenter::Shoot_Command_Topic};
    Publisher<ShootFeedback> feedback_publisher{MessageCenter::Shoot_Feedback_Topic};
    ShootCmd command{};
    ShootFeedback feedback{};
    uint8_t feedback_divider = 0U;
#if SHOOT
    Class_DJIMotor friction_left;
    Class_DJIMotor friction_right;
    Class_DJIMotor loader;
    Struct_DJIMotor_Motion_Snapshot friction_left_snapshot;
    Struct_DJIMotor_Motion_Snapshot friction_right_snapshot;
    Struct_DJIMotor_Motion_Snapshot loader_snapshot;
    Class_DJIMotor_Group friction_group;
    Class_DJIMotor_Group loader_group;
    bool initialized = false;
    bool event_angle_active = false; // 表示正在保持事件累加的角目标，不表示弹丸已完成发射。
    float loader_angle_target_rad = 0.0f;
#endif
};

ShootContext ctx;
}

#if SHOOT

static PID_InitTypeDef Shoot_MakePID(const ShootPidConfig &config)
{
    PID_InitTypeDef pid{};
    pid.K_P = config.kp;
    pid.K_I = config.ki;
    pid.K_D = config.kd;
    pid.I_Out_Max = config.integral_limit;
    pid.Out_Max = config.output_limit;
    pid.D_T = 0.001f;
    return pid;
}

static void Shoot_ApplyCommand(void)
{
    /* ShootMode 是总使能；关闭后摩擦轮和拨弹盘都停止主动输出。 */
    const bool enabled = ctx.command.shoot_mode == ShootMode::ON;
    (void)ctx.friction_group.RequestEnabled(enabled);
    (void)ctx.loader_group.RequestEnabled(enabled);
    if (!enabled)
    {
        ctx.event_angle_active = false;
        return;
    }

    float friction_reference_rad_s = 0.0f;
    if (ctx.command.friction_mode == FrictionMode::ON)
    {
        friction_reference_rad_s = ctx.command.friction_speed_rad_s > 0.0f
            ? ctx.command.friction_speed_rad_s
            : kShootConfig.default_friction_speed_rad_s;
    }
    ctx.friction_group.Control(friction_reference_rad_s, friction_reference_rad_s);

    float loader_speed_target_rad_s = 0.0f;
    switch (ctx.command.loader_mode)
    {
    case LoaderMode::BURST:
    {
        // 连发以角速度控制，退出之前的事件角度保持；射速乘单弹角得到 rad/s。
        ctx.event_angle_active = false;
        ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        const float rate = ctx.command.shoot_rate_hz > 0.0f
            ? ctx.command.shoot_rate_hz : kShootConfig.default_rate_hz;
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
            ? ctx.command.loader_speed_rad_s
            : rate * kShootConfig.one_bullet_angle_rad;
        break;
    }

    case LoaderMode::REVERSE:
        ctx.event_angle_active = false;
        ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
            ? -std::fabs(ctx.command.loader_speed_rad_s)
            : kShootConfig.reverse_speed_rad_s;
        break;

    case LoaderMode::STOP:
    default:
    {
        ShootEvent event;
        /* 每个 1 ms 周期最多取一个逻辑请求并累加目标角，不等待前一发物理完成。 */
        if (MessageCenter::Shoot_Event_Queue.Pop(event))
        {
            if (!ctx.event_angle_active)
            {
                // 首次动作从当前反馈角起步；后续动作继续累加，避免覆盖排队的弹位。
                ctx.loader_angle_target_rad =
                    ctx.loader_snapshot.output_total_angle;
            }
            const float bullet_count =
                event.type == ShootEventType::ShootTriple ? 3.0f : 1.0f;
            ctx.loader_angle_target_rad +=
                bullet_count * kShootConfig.one_bullet_angle_rad;
            ctx.event_angle_active = true;
        }
        if (ctx.event_angle_active)
        {
            ctx.loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
        }
        else
        {
            ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        }
        break;
    }
    }

    // 应用只选择目标和外环；角度/速度/电流 PID 及 CAN 发布复用 DJI 电机组接口。
    if (ctx.event_angle_active)
    {
        ctx.loader_group.Control(ctx.loader_angle_target_rad);
    }
    else
    {
        ctx.loader_group.Control(loader_speed_target_rad_s);
    }
}

static void Shoot_UpdateFeedback(void)
{
    ctx.feedback.friction_left_speed_rad_s =
        ctx.friction_left_snapshot.output_speed;
    ctx.feedback.friction_right_speed_rad_s =
        ctx.friction_right_snapshot.output_speed;
    ctx.feedback.loader_angle_rad = ctx.loader_snapshot.output_total_angle;
    ctx.feedback.loader_speed_rad_s = ctx.loader_snapshot.output_speed;
    ctx.feedback.enabled = ctx.command.shoot_mode == ShootMode::ON &&
                           ctx.friction_left_snapshot.ready &&
                           ctx.friction_right_snapshot.ready && ctx.loader_snapshot.ready;
    ctx.feedback.online = ctx.friction_left_snapshot.online &&
                            ctx.friction_right_snapshot.online &&
                            ctx.loader_snapshot.online;
}
#endif

bool Shoot_Init(void)
{
    ctx.command = {};
    ctx.feedback = {};
    ctx.feedback_divider = 0U;

#if SHOOT
    Struct_DJIMotor_Init_Config friction_config{};
    friction_config.hfdcan = BoardConfig_Get().shoot_bus;
    friction_config.motor_type = Enum_DJIMotor_Type::M3508;
    friction_config.gear_ratio = kShootConfig.friction_gear_ratio;
    friction_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入为 rad/s；增益无可信实车标定依据，启用前需重新整定。
    friction_config.speed_pid = Shoot_MakePID(kShootConfig.friction_speed_pid);

    friction_config.can_id = kShootConfig.friction_left_id;
    const bool left_initialized = ctx.friction_left.Init(friction_config);
    friction_config.can_id = kShootConfig.friction_right_id;
    friction_config.reverse = true;
    const bool right_initialized = ctx.friction_right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = BoardConfig_Get().shoot_bus;
    loader_config.can_id = kShootConfig.loader_id;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508;
    loader_config.close_loop = DJI_MOTOR_CURRENT_LOOP |
                               DJI_MOTOR_SPEED_LOOP |
                               DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    loader_config.current_pid = Shoot_MakePID(kShootConfig.loader_current_pid);
    loader_config.speed_pid = Shoot_MakePID(kShootConfig.loader_speed_pid);
    // 角度环输出是 rad/s；原 360 deg/s 限幅转换为 2π rad/s。
    loader_config.angle_pid = Shoot_MakePID(kShootConfig.loader_angle_pid);
    const bool loader_initialized = ctx.loader.Init(loader_config);

    ctx.initialized = left_initialized && right_initialized && loader_initialized &&
        ctx.friction_group.Init(&ctx.friction_left, &ctx.friction_right) &&
        ctx.loader_group.Init(&ctx.loader);
    if (ctx.initialized)
    {
        (void)ctx.friction_group.RequestEnabled(false);
        (void)ctx.loader_group.RequestEnabled(false);
    }
    ctx.event_angle_active = false;
    ctx.loader_angle_target_rad = 0.0f;
    return ctx.initialized;
#else
    return true;
#endif
}

void Shoot_Update(void)
{
    /* 每个控制周期读取最新命令；没有新消息时继续执行上一帧。 */
    ShootCmd command;
    if (ctx.command_subscriber.Read(command))
    {
        ctx.command = command;
    }

    if (ctx.command.shoot_mode == ShootMode::OFF)
    {
        // 清除本周期开始时已有的事件，避免重新使能后补射；按队列快照限制循环次数。
        ShootEvent discarded_event;
        size_t pending_events = MessageCenter::Shoot_Event_Queue.Size();
        while (pending_events-- > 0U &&
               MessageCenter::Shoot_Event_Queue.Pop(discarded_event))
        {
        }
    }

#if SHOOT
    if (ctx.initialized)
    {
        ctx.friction_left_snapshot = ctx.friction_left.GetMotionSnapshot();
        ctx.friction_right_snapshot = ctx.friction_right.GetMotionSnapshot();
        ctx.loader_snapshot = ctx.loader.GetMotionSnapshot();
        Shoot_ApplyCommand();
        Shoot_UpdateFeedback();
    }
#endif

    /* 控制按 1 kHz 更新，应用层反馈降频到 100 Hz。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
}

#endif /* !LEGACY_INFANTRY_GIMBAL */

#if LEGACY_INFANTRY_GIMBAL

bool Shoot_Init(void)
{
    Shoot_Command = {};
    Shoot_Feedback = {};
    Shoot_Feedback_Divider = 0U;

#if SHOOT && LEGACY_INFANTRY_GIMBAL
    return Shoot_Legacy_Init();
#elif SHOOT
    Struct_DJIMotor_Init_Config friction_config{};
    friction_config.hfdcan = BoardConfig_Get().shoot_bus;
    friction_config.motor_type = Enum_DJIMotor_Type::M3508;
    friction_config.gear_ratio = 1.0f; // 摩擦轮直驱，不使用 M3508 默认减速比 19。
    friction_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入为 rad/s；增益无可信实车标定依据，启用前需重新整定。
    friction_config.speed_pid = Shoot_MakePID(7.5f, 5.0f, 0.0f, 16000.0f, 16000.0f);

    friction_config.can_id = 3U;
    const bool left_initialized = Shoot_Friction_Left.Init(friction_config);
    friction_config.can_id = 2U;
    friction_config.reverse = true;
    const bool right_initialized = Shoot_Friction_Right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = BoardConfig_Get().shoot_bus;
    loader_config.can_id = 8U;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508;
    loader_config.close_loop = DJI_MOTOR_CURRENT_LOOP |
                               DJI_MOTOR_SPEED_LOOP |
                               DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    loader_config.current_pid = Shoot_MakePID(1.0f, 50.0f, 0.0f, 12000.0f, 12000.0f);
    loader_config.speed_pid = Shoot_MakePID(7.5f, 20.0f, 0.0f, 12000.0f, 12000.0f);
    // 角度环输出是 rad/s；原 360 deg/s 限幅转换为 2π rad/s。
    loader_config.angle_pid = Shoot_MakePID(10.0f, 0.0f, 0.0f,
                                            0.0f, DegToRad(360.0f));
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
    Shoot_Loader_Angle_Target_Rad = 0.0f;
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
        Shoot_Friction_Left_Snapshot = Shoot_Friction_Left.GetMotionSnapshot();
        Shoot_Friction_Right_Snapshot = Shoot_Friction_Right.GetMotionSnapshot();
        Shoot_Loader_Snapshot = Shoot_Loader.GetMotionSnapshot();
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

#endif /* LEGACY_INFANTRY_GIMBAL */

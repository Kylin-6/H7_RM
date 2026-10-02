/**
 * @file Shoot.cpp
 * @brief 摩擦轮与拨弹盘应用：统一消息入口与按板型装配的发射机构。
 *
 * - `SHOOT && LEGACY_INFANTRY_GIMBAL`：老步兵云台板，DM3519 摩擦轮 + M2006 拨弹盘，
 *   移植自云台板原工程（H7_RM），含事件单发/连续连发、卡弹检测回退、
 *   Post-shot friction 与热量估计，保留原机构控制参数；迁移后的事件与安全行为需上板复验。
 * - `SHOOT && !LEGACY_INFANTRY_GIMBAL`：RoboMaster_H7 框架通用实现，私有上下文
 *   收拢运行状态，M3508/C620 直驱摩擦轮（gear_ratio=1）默认目标 25 rad/s。
 *   未直接移植热量限制和堵转阈值：依赖实车机构与裁判系统数据。
 * 设备型号与机构控制由板型选择；命令、事件和反馈生命周期共用。
 */

#include "Shoot.h"
#include "Shoot_Config.h"
#include "board_config.h"


#include "message_center.h"

namespace
{
// 所有板型共用同一命令入口、事件队列和 100 Hz 反馈，不再各自维护消息链。
struct ShootApplication
{
    Subscriber<ShootCmd> command_subscriber{MessageCenter::Shoot_Command_Topic};
    Publisher<ShootFeedback> feedback_publisher{MessageCenter::Shoot_Feedback_Topic};
    ShootCmd command{};
    ShootFeedback feedback{};
    uint8_t feedback_divider = 0U;
};
ShootApplication app;
void DiscardEvents()
{
    ShootEvent discarded{};
    size_t pending = MessageCenter::Shoot_Event_Queue.Size();
    while (pending-- > 0U && MessageCenter::Shoot_Event_Queue.Pop(discarded))
    {
    }
}
} // namespace

#if SHOOT && LEGACY_INFANTRY_GIMBAL

/* ==========================================================================
 * 老步兵云台板实现：DM3519 摩擦轮（FDCAN1）+ M2006 拨弹盘（FDCAN2）
 *
 * 与云台板原实现保持一致的语义：
 * - 摩擦轮是速度模式 DM 电机，输入许可下按摩擦轮命令正反转恒速，松扳机后仍延时 300 ms 继续转；
 * - 拨弹盘由「事件单发 / 连续模式」状态机驱动，连发速度来自波轮档位；
 * - 单发用角度环推一颗弹的距离，到位后锁住实际角度抑制回弹；
 * - 堵转按相电流阈值确认 300 ms 后回退半个弹位；
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
using namespace InfantryShootConfig;

enum class FireState : uint8_t
{
    IDLE,
    PRESSING,
    SINGLE,
    BURST,
    POST_SHOT,
    BRAKING,
};

/* 卡弹状态序号；Count_Time 按 HAL tick 差维护，单位 ms。 */
enum JamState : uint8_t
{
    JAM_NORMAL = 0,
    JAM_SUSPECT,
    JAM_HANDLING,
    JAM_FAILED,
};

// 本板的唯一设备所有者；Task/Input/遥测不能拿到电机指针。
struct InfantryShootContext
{
    Class_FSM<4> Jam_FSM;
    Class_DMMotor Friction_Left;
    Class_DMMotor Friction_Right;
    Class_DJIMotor Loader;
    Class_DJIMotor_Group Loader_Group;
    FireState Fire_State;
    uint32_t Press_Start_Tick;
    uint32_t Single_Start_Tick;
    bool Single_Control_Started;
    uint32_t Single_Hold_Start_Tick;
    uint32_t Post_Shot_Start_Tick;
    uint32_t Heat_Start_Tick;
    uint32_t Jam_State_Start_Tick;
    uint32_t Last_Loop_Tick;
    float Single_Target_Angle;
    bool Single_Holding;
    float Jam_Target_Angle;
    float Estimated_Heat;
    bool Heat_Suspect;
    bool Heat_Latched;
    bool Shoot_Initialized;
    Struct_DMMotor_Snapshot left_snapshot{};
    Struct_DMMotor_Snapshot right_snapshot{};
    Struct_DJIMotor_Motion_Snapshot loader_snapshot{};
    decltype(Class_DJIMotor::feedback) loader_feedback{};
};
InfantryShootContext infantry{};

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
    return infantry.loader_snapshot.online;
}

bool FrictionReady(void)
{
    return infantry.left_snapshot.online &&
           infantry.right_snapshot.online &&
           infantry.left_snapshot.actual_enabled &&
           infantry.right_snapshot.actual_enabled &&
           std::fabs(infantry.left_snapshot.feedback.velocity + FRICTION_SPEED_RAD_S) <=
               FRICTION_READY_TOLERANCE_RAD_S &&
           std::fabs(infantry.right_snapshot.feedback.velocity - FRICTION_SPEED_RAD_S) <=
               FRICTION_READY_TOLERANCE_RAD_S;
}

void SetLoaderStopped(void)
{
    // 停火时直接输出零电流，不用高增益速度环在零速附近反复制动。
    infantry.Loader.speed_pid.Set_Integral_Error(0.0f);
    infantry.Loader.angle_pid.Set_Integral_Error(0.0f);
    infantry.Loader.Set_Outer_Loop(DJI_MOTOR_OPEN_LOOP);
    infantry.Loader_Group.Control(0.0f);
}

void SetJamState(JamState state, uint32_t now)
{
    infantry.Jam_FSM.Set_Status(state);
    infantry.Jam_State_Start_Tick = now;
}

void StopAll(void)
{
    infantry.Friction_Left.SetSpeed(0.0f);
    infantry.Friction_Right.SetSpeed(0.0f);
    SetLoaderStopped();
    infantry.Fire_State = FireState::IDLE;
    infantry.Single_Holding = false;
    infantry.Single_Control_Started = false;
    SetJamState(JAM_NORMAL, HAL_GetTick());
    infantry.Heat_Suspect = false;
    infantry.Heat_Latched = false;
}

void UpdateHeat(uint32_t now, bool loader_active)
{
    infantry.Estimated_Heat -= HEAT_COOL_PER_SECOND *
                               static_cast<float>(now - infantry.Last_Loop_Tick) * 0.001f;
    if (infantry.Estimated_Heat < 0.0f)
        infantry.Estimated_Heat = 0.0f;

    const bool spike = loader_active &&
                       infantry.left_snapshot.feedback.torque <= HEAT_LEFT_TORQUE_THRESHOLD_NM &&
                       infantry.right_snapshot.feedback.torque >= HEAT_RIGHT_TORQUE_THRESHOLD_NM;
    if (!spike)
    {
        infantry.Heat_Suspect = false;
        infantry.Heat_Latched = false;
    }
    else if (!infantry.Heat_Latched)
    {
        if (!infantry.Heat_Suspect)
        {
            infantry.Heat_Suspect = true;
            infantry.Heat_Start_Tick = now;
        }
        else if (now - infantry.Heat_Start_Tick >= HEAT_CONFIRM_MS)
        {
            infantry.Estimated_Heat += HEAT_PER_SHOT;
            infantry.Heat_Latched = true;
            infantry.Heat_Suspect = false;
        }
    }
}

/**
 * @brief 卡弹状态机（框架 Class_FSM 驱动）。
 *
 * NORMAL --(电流超阈值)--> SUSPECT --(持续 300 ms)--> HANDLING；
 * HANDLING 编码器到位后回 NORMAL，200 ms 未到位则进入 FAILED，撤销许可后复位。
 * SUSPECT 期间条件消失直接回 NORMAL。状态驻留时间用 HAL tick 差写入 Count_Time，
 * 由框架 FSM 管理状态切换；真实时间判定与老工程一致，漏拍不会延后保护。
 *
 * @param loader_active 本周期拨弹盘是否在出弹。
 * @return true 表示本周期由卡弹状态机接管拨弹盘（回退或保持回退）。
 */
bool UpdateJam(uint32_t now, bool loader_active)
{
    switch (infantry.Jam_FSM.Get_Now_Status_Serial())
    {
    case JAM_FAILED:
        // 超时故障锁存：持续扳机和队列事件都不能重新往前顶，OFF 才复位。
        DiscardEvents();
        infantry.Fire_State = FireState::IDLE;
        infantry.Single_Control_Started = false;
        infantry.Single_Holding = false;
        SetLoaderStopped();
        return true;

    case JAM_HANDLING:
        // 回退优先于单发控制；暂停其超时，保留原单发目标与到位保持状态。
        if (infantry.Single_Control_Started && infantry.Fire_State == FireState::SINGLE)
        {
            infantry.Single_Start_Tick += now - infantry.Last_Loop_Tick;
            if (infantry.Single_Holding)
                infantry.Single_Hold_Start_Tick += now - infantry.Last_Loop_Tick;
        }
        if (std::fabs(infantry.Jam_Target_Angle - infantry.loader_feedback.output_total_angle) <=
            JAM_DONE_ANGLE_RAD)
        {
            SetJamState(JAM_NORMAL, now);
        }
        else if (infantry.Jam_FSM.Status[JAM_HANDLING].Count_Time >= JAM_HANDLE_MS)
        {
            SetJamState(JAM_FAILED, now);
            DiscardEvents();
            infantry.Fire_State = FireState::IDLE;
            infantry.Single_Control_Started = false;
            infantry.Single_Holding = false;
            SetLoaderStopped();
            return true;
        }
        infantry.Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
        infantry.Loader_Group.Control(infantry.Jam_Target_Angle);
        return true;

    case JAM_SUSPECT:
        if (!loader_active ||
            std::abs(infantry.loader_feedback.current_raw) <= JAM_CURRENT_THRESHOLD)
        {
            SetJamState(JAM_NORMAL, now);
            return false;
        }
        if (infantry.Jam_FSM.Status[JAM_SUSPECT].Count_Time >= JAM_CONFIRM_MS)
        {
            SetJamState(JAM_HANDLING, now);
            infantry.Jam_Target_Angle = infantry.loader_feedback.output_total_angle - JAM_BACKOFF_RAD;
            infantry.Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
            infantry.Loader_Group.Control(infantry.Jam_Target_Angle);
            return true;
        }
        return false;

    case JAM_NORMAL:
    default:
        if (loader_active &&
            std::abs(infantry.loader_feedback.current_raw) > JAM_CURRENT_THRESHOLD)
        {
            SetJamState(JAM_SUSPECT, now);
        }
        return false;
    }
}

/** 初始化两台摩擦轮与拨弹盘；返回 false 时上层保持不控制硬件。 */
bool Shoot_InitHardware(void)
{
    if (infantry.Shoot_Initialized)
        return true;

    const bool left_ok = infantry.Friction_Left.Init(BoardConfig_Get().shoot_bus, FRICTION_LEFT_ID,
                                                     FRICTION_LEFT_FEEDBACK_ID,
                                                     Enum_DMMotor_Mode::SPEED, false,
                                                     DM3519_POSITION_MAX_RAD,
                                                     DM3519_VELOCITY_MAX_RAD_S,
                                                     DM3519_TORQUE_MAX_NM);
    const bool right_ok = infantry.Friction_Right.Init(BoardConfig_Get().shoot_bus, FRICTION_RIGHT_ID,
                                                       FRICTION_RIGHT_FEEDBACK_ID,
                                                       Enum_DMMotor_Mode::SPEED, false,
                                                       DM3519_POSITION_MAX_RAD,
                                                       DM3519_VELOCITY_MAX_RAD_S,
                                                       DM3519_TORQUE_MAX_NM);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = BoardConfig_Get().shoot_loader_bus;
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

    const bool loader_ok = infantry.Loader.Init(loader_config) && infantry.Loader_Group.Init(&infantry.Loader);

    infantry.Press_Start_Tick = 0U;
    infantry.Single_Start_Tick = 0U;
    infantry.Single_Control_Started = false;
    infantry.Single_Hold_Start_Tick = 0U;
    infantry.Post_Shot_Start_Tick = 0U;
    infantry.Jam_FSM.Init(JAM_NORMAL);
    infantry.Jam_State_Start_Tick = HAL_GetTick();
    infantry.Heat_Start_Tick = 0U;
    infantry.Single_Target_Angle = 0.0f;
    infantry.Single_Holding = false;
    infantry.Jam_Target_Angle = 0.0f;
    infantry.Estimated_Heat = 0.0f;
    infantry.Heat_Suspect = false;
    infantry.Heat_Latched = false;

    infantry.Shoot_Initialized = left_ok && right_ok && loader_ok;
    infantry.Last_Loop_Tick = HAL_GetTick();
    if (infantry.Shoot_Initialized)
    {
        // 电机端预先配置速度模式；初始化保持失能，许可由 RobotCmd 授予。
        (void) infantry.Friction_Left.RequestEnabled(false);
        (void) infantry.Friction_Right.RequestEnabled(false);
        (void) infantry.Loader_Group.RequestEnabled(false);
        StopAll();
    }
    else
    {
        (void) infantry.Friction_Left.RequestEnabled(false);
        (void) infantry.Friction_Right.RequestEnabled(false);
        (void) infantry.Loader_Group.RequestEnabled(false);
    }
    return infantry.Shoot_Initialized;
}

/**
 * @brief 老步兵云台板的 1 kHz 发射控制。
 *
 * 输入来自 ShootCmd：`shoot_mode == ON` 表示安全输出许可，`loader_speed_rad_s` 表示
 * 波轮映射后的连发拨弹盘速度（rad/s）。通道失效由 Input/RobotCmd 解除 shoot_mode，
 * 本函数随即停火并复位状态机。
 */
void Shoot_ApplyCommand(void)
{
    const uint32_t now = HAL_GetTick();
    // OFF 是安全撤销，不是松扳机。失联同周期停轮/停拨弹并清除状态。
    const bool enabled = app.command.shoot_mode == ShootMode::ON;
    (void) infantry.Friction_Left.RequestEnabled(enabled);
    (void) infantry.Friction_Right.RequestEnabled(enabled);
    (void) infantry.Loader_Group.RequestEnabled(enabled);
    if (!enabled)
    {
        StopAll();
        UpdateHeat(now, false);
        infantry.Last_Loop_Tick = now;
        return;
    }
    // Count_Time 表示真实毫秒；线程标志合并/漏拍不能把 300 ms 拖成 300 次调用。
    const auto jam_state = infantry.Jam_FSM.Get_Now_Status_Serial();
    infantry.Jam_FSM.Status[jam_state].Count_Time = now - infantry.Jam_State_Start_Tick;

    // PRESSING 仅表示摩擦轮准备阶段，不在此重复识别长短按。
    if (infantry.Fire_State == FireState::IDLE && app.command.friction_mode == FrictionMode::ON)
    {
        infantry.Fire_State = FireState::PRESSING;
        infantry.Press_Start_Tick = now;
    }
    // 连续模式优先；STOP 每周期至多接收一个不可覆盖的单发/三连发事件。
    if (app.command.loader_mode == LoaderMode::BURST ||
        app.command.loader_mode == LoaderMode::REVERSE)
    {
        infantry.Fire_State = FireState::BURST;
        infantry.Single_Holding = false;
    }
    else
    {
        if (infantry.Fire_State == FireState::BURST)
        {
            // 连发退出后先用零速度闭环制动，避免惯性转动叠加到下一次单发。
            infantry.Fire_State = FireState::BRAKING;
            infantry.Single_Control_Started = false;
            infantry.Single_Holding = false;
            infantry.Loader.speed_pid.Set_Integral_Error(0.0f);
            infantry.Loader.angle_pid.Set_Integral_Error(0.0f);
        }
        if (infantry.Fire_State == FireState::BRAKING && LoaderFeedbackFresh() &&
            std::fabs(infantry.loader_feedback.output_speed) <= LOADER_STOP_SPEED_RAD_S)
        {
            infantry.Fire_State = FireState::POST_SHOT;
            infantry.Post_Shot_Start_Tick = now;
        }
        ShootEvent event{};
        if (infantry.Fire_State != FireState::BRAKING &&
            jam_state != JAM_HANDLING && jam_state != JAM_FAILED &&
            MessageCenter::Shoot_Event_Queue.Pop(event))
        {
            // 事件仅在电机反馈新鲜时起步；故障期间清除，不恢复后补射。
            if (LoaderFeedbackFresh())
            {
                if (infantry.Fire_State != FireState::SINGLE)
                {
                    infantry.Single_Target_Angle = infantry.loader_feedback.output_total_angle;
                    // 速度环/角度环切换不继承连发积分，首发只追加新的弹位。
                    infantry.Loader.speed_pid.Set_Integral_Error(0.0f);
                    infantry.Loader.angle_pid.Set_Integral_Error(0.0f);
                }
                const float bullets = event.type == ShootEventType::ShootTriple ? 3.0f : 1.0f;
                infantry.Single_Target_Angle += bullets * ONE_BULLET_MOTOR_OUTPUT_RAD;
                infantry.Fire_State = FireState::SINGLE;
                // 摩擦轮准备不计入拨弹超时；首次真正进入角度控制时才启动计时。
                infantry.Single_Control_Started = false;
                infantry.Single_Holding = false;
            }
        }
        if (infantry.Fire_State == FireState::PRESSING &&
            app.command.friction_mode == FrictionMode::OFF)
        {
            infantry.Fire_State = FireState::POST_SHOT;
            infantry.Post_Shot_Start_Tick = now;
        }
        if (infantry.Fire_State == FireState::POST_SHOT &&
            now - infantry.Post_Shot_Start_Tick >= POST_SHOT_FRICTION_MS)
        {
            infantry.Fire_State = FireState::IDLE;
        }
    }

    // 失效时撤销整个机构许可并丢弃已识别动作，恢复只接受新的事件。
    if (!infantry.left_snapshot.online ||
        infantry.left_snapshot.fault ||
        !infantry.right_snapshot.online ||
        infantry.right_snapshot.fault || !LoaderFeedbackFresh())
    {
        (void) infantry.Friction_Left.RequestEnabled(false);
        (void) infantry.Friction_Right.RequestEnabled(false);
        (void) infantry.Loader_Group.RequestEnabled(false);
        DiscardEvents();
        StopAll();
        // 设备暂时掉线不能解除已经锁存的回退超时，仍需撤销发射许可。
        if (jam_state == JAM_FAILED)
            SetJamState(JAM_FAILED, now);
        UpdateHeat(now, false);
        infantry.Last_Loop_Tick = now;
        return;
    }

    const bool firing_requested = app.command.friction_mode == FrictionMode::ON ||
                                  infantry.Fire_State != FireState::IDLE;
    infantry.Friction_Left.SetSpeed(firing_requested ? -FRICTION_SPEED_RAD_S : 0.0f);
    infantry.Friction_Right.SetSpeed(firing_requested ? FRICTION_SPEED_RAD_S : 0.0f);

    // 回退期间不先执行原单发/连发控制，否则会改写目标或触发单发超时。
    if (jam_state == JAM_HANDLING || jam_state == JAM_FAILED)
    {
        (void) UpdateJam(now, false);
        UpdateHeat(now, infantry.Jam_FSM.Get_Now_Status_Serial() != JAM_FAILED);
        infantry.Last_Loop_Tick = now;
        return;
    }

    if (infantry.Fire_State == FireState::BRAKING)
    {
        // 制动不依赖摩擦轮转速或热量许可；设备在线门控已经在上方检查。
        infantry.Loader.speed_pid.Set_K_P(LOADER_SPEED_KP);
        infantry.Loader.speed_pid.Set_K_I(LOADER_SPEED_KI);
        infantry.Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        infantry.Loader_Group.Control(0.0f);
        UpdateHeat(now, false);
        infantry.Last_Loop_Tick = now;
        return;
    }

    bool loader_active = false;
    if (LoaderFeedbackFresh() && FrictionReady() && infantry.Estimated_Heat < HEAT_LIMIT)
    {
        if (infantry.Fire_State == FireState::SINGLE)
        {
            if (!infantry.Single_Control_Started)
            {
                infantry.Single_Start_Tick = now;
                infantry.Single_Control_Started = true;
            }
            loader_active = true;
            infantry.Loader.speed_pid.Set_K_P(LOADER_SINGLE_SPEED_KP);
            infantry.Loader.speed_pid.Set_K_I(LOADER_SINGLE_SPEED_KI);
            infantry.Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
            infantry.Loader_Group.Control(infantry.Single_Target_Angle);
            if (!infantry.Single_Holding &&
                (std::fabs(infantry.Single_Target_Angle - infantry.loader_feedback.output_total_angle) <=
                     SINGLE_DONE_ANGLE_RAD ||
                 now - infantry.Single_Start_Tick >= SINGLE_TIMEOUT_MS))
            {
                // 锁住结束时的实际角度，抑制惯性超调和机械回弹。
                infantry.Single_Target_Angle = infantry.loader_feedback.output_total_angle;
                infantry.Single_Hold_Start_Tick = now;
                infantry.Single_Holding = true;
            }
            else if (infantry.Single_Holding && now - infantry.Single_Hold_Start_Tick >= SINGLE_HOLD_MS)
            {
                infantry.Single_Holding = false;
                infantry.Post_Shot_Start_Tick = now;
                infantry.Fire_State = FireState::POST_SHOT;
            }
        }
        else if (infantry.Fire_State == FireState::BURST)
        {
            loader_active = true;
            infantry.Loader.speed_pid.Set_K_P(LOADER_SPEED_KP);
            infantry.Loader.speed_pid.Set_K_I(LOADER_SPEED_KI);
            infantry.Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
            const float speed = app.command.loader_mode == LoaderMode::REVERSE
                                    ? -std::fabs(app.command.loader_speed_rad_s)
                                    : app.command.loader_speed_rad_s;
            infantry.Loader_Group.Control(speed);
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
    infantry.Last_Loop_Tick = now;
}

/** 把云台板发射状态写入框架的统一反馈结构（速度 rad/s、角度 rad）。 */
void Shoot_UpdateFeedback(void)
{
    app.feedback.friction_left_speed_rad_s = infantry.left_snapshot.feedback.velocity;
    app.feedback.friction_right_speed_rad_s = infantry.right_snapshot.feedback.velocity;
    app.feedback.loader_angle_rad = infantry.loader_feedback.output_total_angle;
    app.feedback.loader_speed_rad_s = infantry.loader_feedback.output_speed;
    app.feedback.enabled = app.command.shoot_mode == ShootMode::ON &&
                           infantry.left_snapshot.ready &&
                           infantry.right_snapshot.ready &&
                           infantry.loader_snapshot.ready;
    app.feedback.online = infantry.left_snapshot.online &&
                          infantry.right_snapshot.online &&
                          infantry.loader_snapshot.online;
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
        *initialized = infantry.Shoot_Initialized ? 1.0f : 0.0f;
    if (left_feedback != nullptr)
        *left_feedback = infantry.Friction_Left.IsOnline() ? 1.0f : 0.0f;
    if (right_feedback != nullptr)
        *right_feedback = infantry.Friction_Right.IsOnline() ? 1.0f : 0.0f;
    if (loader_feedback != nullptr)
        *loader_feedback = infantry.Loader.IsOnline() ? 1.0f : 0.0f;
    if (friction_ready != nullptr)
        *friction_ready = FrictionReady() ? 1.0f : 0.0f;
    if (left_velocity != nullptr)
        *left_velocity = infantry.Friction_Left.feedback.velocity;
    if (right_velocity != nullptr)
        *right_velocity = infantry.Friction_Right.feedback.velocity;
    /* 摩擦轮指令方向与云台板原实现一致：左轮取负、右轮取正。 */
    const bool friction_commanded = app.command.shoot_mode == ShootMode::ON &&
                                    (app.command.friction_mode == FrictionMode::ON || infantry.Fire_State != FireState::IDLE);
    if (left_target != nullptr)
        *left_target = friction_commanded ? -FRICTION_SPEED_RAD_S : 0.0f;
    if (right_target != nullptr)
        *right_target = friction_commanded ? FRICTION_SPEED_RAD_S : 0.0f;
    if (fire_state != nullptr)
        *fire_state = static_cast<float>(infantry.Fire_State);
    if (press_duration_ms != nullptr)
        *press_duration_ms = (infantry.Fire_State == FireState::PRESSING)
                                 ? static_cast<float>(HAL_GetTick() - infantry.Press_Start_Tick)
                                 : 0.0f;
    if (left_motor_state != nullptr)
        *left_motor_state = static_cast<float>(infantry.Friction_Left.feedback.state);
    if (right_motor_state != nullptr)
        *right_motor_state = static_cast<float>(infantry.Friction_Right.feedback.state);
}

extern "C" void Shoot_GetLoaderDebug(Struct_Legacy_Loader_Debug *debug)
{
    if (debug == nullptr)
        return;

    const uint64_t timestamp_us = infantry.Loader.Get_Last_Feedback_Timestamp_Us();
    const uint64_t now_us = SYS_Timestamp.Get_Now_Microsecond();
    debug->encoder = static_cast<float>(infantry.Loader.feedback.encoder);
    debug->rotor_total_angle_degree = infantry.Loader.feedback.rotor_total_angle_degree;
    debug->output_total_angle_degree = infantry.Loader.feedback.output_total_angle_degree;
    debug->rotor_speed_rad_s = infantry.Loader.feedback.rotor_speed;
    debug->output_speed_rad_s = infantry.Loader.feedback.output_speed;
    debug->current_raw = static_cast<float>(infantry.Loader.feedback.current_raw);
    debug->feedback_age_ms = timestamp_us == 0U
                                 ? -1.0f
                                 : static_cast<float>(now_us - timestamp_us) * 0.001f;
    debug->speed_pid_out = infantry.Loader.feedback.pid.speed.out;
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
    const bool enabled = app.command.shoot_mode == ShootMode::ON;
    (void)ctx.friction_group.RequestEnabled(enabled);
    (void)ctx.loader_group.RequestEnabled(enabled);
    if (!enabled)
    {
        ctx.event_angle_active = false;
        return;
    }

    float friction_reference_rad_s = 0.0f;
    if (app.command.friction_mode == FrictionMode::ON)
    {
        friction_reference_rad_s = app.command.friction_speed_rad_s > 0.0f
                                       ? app.command.friction_speed_rad_s
                                       : kShootConfig.default_friction_speed_rad_s;
    }
    ctx.friction_group.Control(friction_reference_rad_s, friction_reference_rad_s);

    float loader_speed_target_rad_s = 0.0f;
    switch (app.command.loader_mode)
    {
    case LoaderMode::BURST:
    {
        // 连发以角速度控制，退出之前的事件角度保持；射速乘单弹角得到 rad/s。
        ctx.event_angle_active = false;
        ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        const float rate = app.command.shoot_rate_hz > 0.0f
                               ? app.command.shoot_rate_hz
                               : kShootConfig.default_rate_hz;
        loader_speed_target_rad_s = app.command.loader_speed_rad_s != 0.0f
                                        ? app.command.loader_speed_rad_s
                                        : rate * kShootConfig.one_bullet_angle_rad;
        break;
    }

    case LoaderMode::REVERSE:
        ctx.event_angle_active = false;
        ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        loader_speed_target_rad_s = app.command.loader_speed_rad_s != 0.0f
                                        ? -std::fabs(app.command.loader_speed_rad_s)
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
    app.feedback.friction_left_speed_rad_s =
        ctx.friction_left_snapshot.output_speed;
    app.feedback.friction_right_speed_rad_s =
        ctx.friction_right_snapshot.output_speed;
    app.feedback.loader_angle_rad = ctx.loader_snapshot.output_total_angle;
    app.feedback.loader_speed_rad_s = ctx.loader_snapshot.output_speed;
    app.feedback.enabled = app.command.shoot_mode == ShootMode::ON &&
                           ctx.friction_left_snapshot.ready &&
                           ctx.friction_right_snapshot.ready && ctx.loader_snapshot.ready;
    app.feedback.online = ctx.friction_left_snapshot.online &&
                          ctx.friction_right_snapshot.online &&
                          ctx.loader_snapshot.online;
}
#endif

#if SHOOT
static bool Shoot_InitHardware(void)
{
    app.command = {};
    app.feedback = {};
    app.feedback_divider = 0U;

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
}

#endif

#endif /* !LEGACY_INFANTRY_GIMBAL */

bool Shoot_Init(void)
{
    app.command = {};
    app.feedback = {};
    app.feedback_divider = 0U;
#if SHOOT
    return Shoot_InitHardware();
#else
    return true;
#endif
}

void Shoot_Update(void)
{
    ShootCmd command{};
    if (app.command_subscriber.Read(command))
    {
        app.command = command;
    }
    if (app.command.shoot_mode == ShootMode::OFF ||
        app.command.loader_mode != LoaderMode::STOP)
    {
        // 安全撤销/连续模式取消排队动作；按开始时的容量快照有界排空。
        DiscardEvents();
    }
#if SHOOT
#if LEGACY_INFANTRY_GIMBAL
    const bool initialized = infantry.Shoot_Initialized;
    if (initialized)
    {
        infantry.left_snapshot = infantry.Friction_Left.GetFeedbackSnapshot();
        infantry.right_snapshot = infantry.Friction_Right.GetFeedbackSnapshot();
        infantry.loader_snapshot = infantry.Loader.GetMotionSnapshot();
        // 控制判定用同一份拨弹盘反馈，避免 ISR 在角度/电流读取之间改写。
        const uint32_t interrupt_state = __get_PRIMASK();
        __disable_irq();
        __DMB();
        infantry.loader_feedback = infantry.Loader.feedback;
        __DMB();
        __set_PRIMASK(interrupt_state);
    }
#else
    const bool initialized = ctx.initialized;
    if (initialized)
    {
        ctx.friction_left_snapshot = ctx.friction_left.GetMotionSnapshot();
        ctx.friction_right_snapshot = ctx.friction_right.GetMotionSnapshot();
        ctx.loader_snapshot = ctx.loader.GetMotionSnapshot();
    }
#endif
    if (initialized)
    {
        Shoot_ApplyCommand();
        Shoot_UpdateFeedback();
    }
#endif
    if (++app.feedback_divider >= 10U)
    {
        app.feedback_divider = 0U;
        app.feedback_publisher.Publish(app.feedback);
    }
}

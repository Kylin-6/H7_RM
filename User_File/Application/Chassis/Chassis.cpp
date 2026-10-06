/**
 * @file Chassis.cpp
 * @brief 老步兵底盘应用：四路 DM 麦轮（速度模式）+ 本板 Yaw DM 电机（MIT 速度环）。
 * @details 四路 DM 麦轮负责底盘运动，另一路挂在本板的 Yaw DM 电机负责整机云台
 *          偏航轴。遥控整形、云台跟随与坐标旋转在 Input 层完成，本模块只做速率
 *          规划（平移在云台坐标系进行）、麦轮逆运动学、Yaw 控制律与下发，
 *          并发布底盘反馈与 Yaw 轴反馈。
 *          应用按构建期源码选择编入 ChassisBoard（H7_APP_CHASSIS），文件内不再
 *          保留功能条件编译；关闭时不编译、不调度、不发布底盘反馈。
 */

#include "Chassis.h"
#include "Chassis_Config.h"
#include "board_config.h"
#include "message_center.h"

#include "alg_trajectory.h"
#include "dmmotor.h"

#include <cmath>

static constexpr uint64_t CHASSIS_COMMAND_MAX_AGE_US = 100000U;

namespace
{
/** INS 新鲜度门限；失效时只停用角速度前馈，保留遥控 Yaw 速度控制。 */
constexpr uint64_t INS_MAX_AGE_US = 10000U;
/** 电机数量与反馈发布分频（2 ms 控制路径下 10 分频 = 100 Hz）。 */
constexpr uint8_t MOTOR_COUNT = 4U;
constexpr uint8_t FEEDBACK_DIVIDER = 10U;

/** 以最大步长逼近目标，用于 MIT 阻尼与力矩前馈的逐周期限速。 */
float MoveTowards(float current, float target, float maximum_delta)
{
    const float delta = target - current;

    if (std::fabs(delta) <= maximum_delta)
    {
        return target;
    }
    return current + (delta > 0.0f ? maximum_delta : -maximum_delta);
}

/** 单轴 S 曲线速度规划；失败时由调用者安全化四轮输出。 */
bool PlanAxis(Class_Trajectory &trajectory, float target, float *speed)
{
    const float planned_target =
        std::fabs(target) <= kInfantryChassisConfig.planning_threshold ? 0.0f : target;
    if (!trajectory.Set_Target_Velocity(planned_target) ||
        trajectory.TIM_Calculate_PeriodElapsedCallback() == TRAJECTORY_ERROR)
    {
        return false;
    }
    *speed = trajectory.Get_Velocity();
    return true;
}

struct LegacyChassisContext
{
    Publisher<ChassisFeedback> chassis_feedback_publisher{
        MessageCenter::Chassis_Feedback_Topic};
    /** Yaw 轴挂在本板，但对外仍是云台偏航轴，因此按 GimbalFeedback 发布。 */
    Publisher<GimbalFeedback> yaw_feedback_publisher{
        MessageCenter::Gimbal_Feedback_Topic};
    Class_DMMotor wheel_motor[MOTOR_COUNT];
    Class_DMMotor yaw_motor;
    Class_Trajectory x_trajectory;
    Class_Trajectory y_trajectory;
    bool translation_gimbal_frame = false;
    Class_Trajectory w_trajectory;
    Class_Trajectory yaw_trajectory;
    ChassisCmd command{};
    GimbalCmd yaw_command{};
    ChassisFeedback feedback{};
    GimbalFeedback yaw_feedback{};
    float planned_x = 0.0f;
    float planned_y = 0.0f;
    float planned_w = 0.0f;
    float yaw_speed = 0.0f;
    float yaw_kd = kInfantryChassisConfig.yaw_mit_kd_center;
    float yaw_torque_feedforward = 0.0f;
    uint8_t control_divider = 0U;
    uint8_t feedback_divider = 0U;
    bool initialized = false;
};

LegacyChassisContext ctx;

/**
 * @brief 麦轮逆运动学：三轴速度直接代数组合成四轮目标。
 * @note 不做轮距与半径换算（三轴本身已是轮速量纲），四轮同比缩放限幅。
 */
void ControlWheels(float velocity_x, float velocity_y, float velocity_w)
{
    float wheel_speed[MOTOR_COUNT];
    wheel_speed[0] = velocity_y + velocity_x + velocity_w;
    wheel_speed[1] = velocity_y - velocity_x + velocity_w;
    wheel_speed[2] = -velocity_y - velocity_x + velocity_w;
    wheel_speed[3] = velocity_x - velocity_y + velocity_w;

    float peak = kInfantryChassisConfig.wheel_speed_max;
    for (uint8_t index = 0U; index < MOTOR_COUNT; ++index)
    {
        peak = std::fmax(peak, std::fabs(wheel_speed[index]));
    }
    const float scale = kInfantryChassisConfig.wheel_speed_max / peak;
    for (uint8_t index = 0U; index < MOTOR_COUNT; ++index)
    {
        const float limited = Basic_Math_Constrain(wheel_speed[index] * scale,
                                                  -kInfantryChassisConfig.wheel_speed_max,
                                                   kInfantryChassisConfig.wheel_speed_max);
        (void)ctx.wheel_motor[index].SetSpeed(limited);
    }
}

/**
 * @brief Yaw 轴 MIT 速度环。
 * @details 摇杆速度先用 S 曲线平滑，再叠加底盘自转补偿并限制总速度；MIT 位置增益恒为 0、
 *          位置目标恒为 0，阻尼随摇杆推进减小、反向瞬间提高，力矩前馈由规划加速度换算。
 */
void ControlYaw(float chassis_yaw_rate_rad_s)
{
    const float stick_speed = Basic_Math_Constrain(
        ctx.yaw_command.yaw_speed_rad_s,
        -kInfantryChassisConfig.yaw_speed_max_rad_s,
        kInfantryChassisConfig.yaw_speed_max_rad_s);

    float target_speed = stick_speed;

    const float stick_ratio =
        std::fabs(stick_speed) / kInfantryChassisConfig.yaw_speed_max_rad_s;
    const float speed_previous = ctx.yaw_speed;
    if (std::fabs(target_speed) <= kInfantryChassisConfig.planning_threshold)
    {
        target_speed = 0.0f;
    }
    if (!ctx.yaw_trajectory.Set_Target_Velocity(target_speed) ||
        ctx.yaw_trajectory.TIM_Calculate_PeriodElapsedCallback() == TRAJECTORY_ERROR)
    {
        (void)ctx.yaw_trajectory.Reset(0.0f);
        ctx.yaw_speed = 0.0f;
        ctx.yaw_torque_feedforward = 0.0f;
        (void)ctx.yaw_motor.SetMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        (void)ctx.yaw_motor.RequestEnabled(false);
        return;
    }
    /* 自转补偿不经过摇杆 S 曲线，避免起转与变速时引入额外规划滞后。 */
    ctx.yaw_speed = Basic_Math_Constrain(
        ctx.yaw_trajectory.Get_Velocity() -
            kInfantryChassisConfig.yaw_rate_feedforward_gain * chassis_yaw_rate_rad_s,
        -kInfantryChassisConfig.yaw_total_speed_max_rad_s,
        kInfantryChassisConfig.yaw_total_speed_max_rad_s);

    float kd_target = kInfantryChassisConfig.yaw_mit_kd_center +
                      (kInfantryChassisConfig.yaw_mit_kd_moving -
                       kInfantryChassisConfig.yaw_mit_kd_center) *
                          stick_ratio;
    if (speed_previous * stick_speed < 0.0f)
    {
        kd_target = kInfantryChassisConfig.yaw_mit_kd_reverse;
    }
    ctx.yaw_kd = MoveTowards(ctx.yaw_kd, kd_target,
                                    kInfantryChassisConfig.yaw_mit_kd_slew_rate *
                                        kInfantryChassisConfig.control_dt_s);

    const float acceleration_command =
        (ctx.yaw_speed - speed_previous) / kInfantryChassisConfig.control_dt_s;
    const float torque_target = Basic_Math_Constrain(
        acceleration_command * kInfantryChassisConfig.yaw_mit_torque_gain,
        -kInfantryChassisConfig.yaw_mit_torque_max_nm,
        kInfantryChassisConfig.yaw_mit_torque_max_nm);
    ctx.yaw_torque_feedforward = MoveTowards(
        ctx.yaw_torque_feedforward, torque_target,
        kInfantryChassisConfig.yaw_mit_torque_slew_rate *
            kInfantryChassisConfig.control_dt_s);

    /* 位置目标 0、位置增益 0：速度由电机内部闭环，阻尼与外力矩由本层给。 */
    (void)ctx.yaw_motor.SetMIT(0.0f, ctx.yaw_speed, kInfantryChassisConfig.yaw_mit_kp,
                              ctx.yaw_kd, ctx.yaw_torque_feedforward);
}

/** 100 Hz 发布底盘反馈与 Yaw 轴反馈；未就绪或失效状态如实上报。 */
void PublishFeedback(bool ins_valid)
{
    bool online = true;
    bool ready = true;
    for (uint8_t index = 0U; index < MOTOR_COUNT; ++index)
    {
        online = online && ctx.wheel_motor[index].IsOnline();
        ready = ready && ctx.wheel_motor[index].GetFeedbackSnapshot().ready;
    }

    /* 本板不测量真实车体速度：按输入边界约定回写规划值（SI 归一化），
     * 与 Input 提交的量纲一致；enabled 表示四轮均就绪且当前不是 ZERO_FORCE。 */
    ctx.feedback.velocity_x_m_s = Chassis_TranslateX_ToSi(ctx.planned_x);
    ctx.feedback.velocity_y_m_s = Chassis_TranslateY_ToSi(ctx.planned_y);
    ctx.feedback.angular_velocity_rad_s = Chassis_Rotation_ToSi(ctx.planned_w);
    ctx.feedback.enabled =
        ctx.initialized && ctx.command.mode != ChassisMode::ZERO_FORCE && ready;
    ctx.feedback.online = online;
    ctx.chassis_feedback_publisher.Publish(ctx.feedback);

    /* Yaw 轴反馈供 Input 做平移坐标旋转与跟随判断；本板不拥有 Pitch。 */
    ctx.yaw_feedback.yaw_rad = ctx.yaw_motor.feedback.position;
    ctx.yaw_feedback.yaw_speed_rad_s = ctx.yaw_motor.feedback.velocity;
    ctx.yaw_feedback.ins_valid = ins_valid;
    ctx.yaw_feedback.enabled =
        ctx.initialized && ctx.yaw_command.mode == GimbalMode::IMU &&
        ctx.yaw_motor.GetFeedbackSnapshot().ready;
    ctx.yaw_feedback_publisher.Publish(ctx.yaw_feedback);
}
} // namespace


Struct_Chassis_Diagnostic_Input Chassis_GetDiagnostic(void)
{
    Struct_Chassis_Diagnostic_Input d{};
    d.initialized = ctx.initialized;
    // Fresh = 这份姿态是否可用于当前控制周期；设备 Online（liveness）由
    // 各 Device 内的 Daemon 判定并经 motor[i].online / IsOnline() 暴露。
    INS_State ins{};
    d.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ins, INS_MAX_AGE_US);
    d.permitted = ctx.command.mode != ChassisMode::ZERO_FORCE ||
                  ctx.yaw_command.mode == GimbalMode::IMU;
    for (uint8_t i = 0U; i < 5U; ++i)
        d.motor[i] = i < 4U ? ctx.wheel_motor[i].GetFeedbackSnapshot()
                           : ctx.yaw_motor.GetFeedbackSnapshot();
    return d;
}

/**
 * @brief 启动阶段注册四路麦轮与 Yaw 轴共五台 DM 电机。
 * @return 全部电机注册成功时为 true。
 * @note 仅调用一次；成功后先落 Yaw 安全目标并请求全部电机失能，
 *       不执行机械寻零，不等待电机反馈。
 */
bool Chassis_Init(void)
{
    ctx.command = {};
    ctx.feedback = {};
    ctx.feedback_divider = 0U;

    ctx.yaw_command = {};
    ctx.yaw_feedback = {};
    ctx.yaw_speed = 0.0f;
    ctx.yaw_kd = kInfantryChassisConfig.yaw_mit_kd_center;
    ctx.yaw_torque_feedforward = 0.0f;
    ctx.control_divider = 0U;

    bool initialized = true;
    for (uint8_t index = 0U; index < MOTOR_COUNT; ++index)
    {
        initialized = ctx.wheel_motor[index].Init(
                          BoardConfig_Get().chassis_wheel_bus,
                          kInfantryChassisConfig.motor_id[index],
                          kInfantryChassisConfig.motor_master_id[index],
                          Enum_DMMotor_Mode::SPEED, false,
                          kInfantryChassisConfig.motor_position_max_rad,
                          kInfantryChassisConfig.motor_velocity_max_rad_s,
                          kInfantryChassisConfig.motor_torque_max_nm) &&
                      initialized;
    }
    initialized = ctx.yaw_motor.Init(
                      BoardConfig_Get().gimbal_yaw_bus,
                      kInfantryChassisConfig.yaw_motor_id,
                      kInfantryChassisConfig.yaw_motor_master_id,
                      Enum_DMMotor_Mode::MIT, false,
                      kInfantryChassisConfig.yaw_position_max_rad,
                      kInfantryChassisConfig.yaw_velocity_max_rad_s,
                      kInfantryChassisConfig.yaw_torque_max_nm) &&
                  initialized;

    /* 三轴 S 曲线从零速起步；限幅沿用各轴现有速度与加速度上限。 */
    initialized = ctx.x_trajectory.Init(
                      kInfantryChassisConfig.velocity_x_max,
                      kInfantryChassisConfig.x_accel_limit,
                      kInfantryChassisConfig.x_jerk_limit,
                      kInfantryChassisConfig.control_dt_s) && initialized;
    initialized = ctx.y_trajectory.Init(
                      kInfantryChassisConfig.velocity_y_max,
                      kInfantryChassisConfig.y_accel_limit,
                      kInfantryChassisConfig.y_jerk_limit,
                      kInfantryChassisConfig.control_dt_s) && initialized;
    initialized = ctx.w_trajectory.Init(
                      kInfantryChassisConfig.angular_velocity_max,
                      kInfantryChassisConfig.w_accel_limit,
                      kInfantryChassisConfig.w_jerk_limit,
                      kInfantryChassisConfig.control_dt_s) && initialized;
    initialized = ctx.yaw_trajectory.Init(
                      kInfantryChassisConfig.yaw_speed_max_rad_s,
                      kInfantryChassisConfig.yaw_trajectory_accel_max,
                      kInfantryChassisConfig.yaw_trajectory_jerk_max,
                      kInfantryChassisConfig.control_dt_s) && initialized;

    /* 上电默认失能：先落 Yaw 安全目标，再对全部已配置电机请求失能；
     * 即使部分电机注册失败也尝试停住它们。 */
    (void)ctx.yaw_motor.SetMIT(0.0f, 0.0f, kInfantryChassisConfig.yaw_mit_kp,
                              ctx.yaw_kd, 0.0f);
    (void)ctx.yaw_motor.RequestEnabled(false);
    for (uint8_t index = 0U; index < MOTOR_COUNT; ++index)
    {
        (void)ctx.wheel_motor[index].RequestEnabled(false);
    }

    ctx.initialized = initialized;
    return initialized;
}

/**
 * @brief 1 kHz 读取新鲜命令、按 2 ms 分频规划并下发底盘与 Yaw 目标、更新反馈。
 * @note 底盘命令超过 100 ms 未刷新时默认 ZERO_FORCE；Yaw 命令按最新值读取，
 *       失联安全撤销由 RobotCmd 完成。设备反馈超时保护由 DM 驱动逐电机执行。
 */
void Chassis_Update(void)
{
    /* 仅在命令 Topic 仍新鲜时沿用目标；过期后使用默认 ZERO_FORCE 关闭输出。 */
    ctx.command = {};
    // 命令需存在且年龄不超过 100 ms；双板由 Transport 发布，本地由 RobotCmd 发布。
    (void) MessageCenter::Chassis_Command_Topic.ReadFresh(
        ctx.command, CHASSIS_COMMAND_MAX_AGE_US);

    /*
     * Yaw 命令按框架云台命令的语义读取：RobotCmd 只在目标变化或安全撤销时发布，
     * 因此用最新值（ReadWithMeta）而不是时效判定，否则摇杆保持不动会被判为过期而
     * 误失能；遥控失联时 RobotCmd 会把命令撤销为 DISABLED。
     */
    const auto yaw_message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.yaw_command = yaw_message.valid ? yaw_message.data : GimbalCmd{};

    INS_State ins_state{};
    const bool ins_valid =
        MessageCenter::INS_State_Topic.ReadFresh(ins_state, INS_MAX_AGE_US);

    if (ctx.initialized)
    {
        const bool wheels_enabled = ctx.command.mode != ChassisMode::ZERO_FORCE;
        const bool yaw_enabled = ctx.yaw_command.mode == GimbalMode::IMU;

        /* 使能请求是边沿语义：首次请求与状态变化才产生总线流量，
         * 掉线补发由 StatusTask 的 100 Hz ServiceAll 负责。 */
        for (uint8_t index = 0U; index < MOTOR_COUNT; ++index)
        {
            (void)ctx.wheel_motor[index].RequestEnabled(wheels_enabled);
        }
        (void)ctx.yaw_motor.RequestEnabled(yaw_enabled);

        /* 老步兵控制路径按 2 ms 执行：与老工程一致，同时把 DM 速度帧数量
         * 压回 FDCAN1 的承载范围内（周期通道只在目标更新时才发送）。 */
        ctx.control_divider++;
        if (ctx.control_divider >= kInfantryChassisConfig.control_divider)
        {
            ctx.control_divider = 0U;

            if (wheels_enabled)
            {
                GimbalFeedback yaw_feedback{};
                const bool gimbal_frame = MessageCenter::Gimbal_Feedback_Topic.ReadFresh(
                    yaw_feedback, 100000U) && yaw_feedback.enabled &&
                    std::isfinite(yaw_feedback.yaw_rad);
                const float angle = gimbal_frame ? Basic_Math_Modulus_Normalization(
                    yaw_feedback.yaw_rad - kInfantryChassisConfig.follow_forward_rad,
                    2.0f * 3.14159265358979323846f) : 0.0f;
                const float cosine = std::cos(angle);
                const float sine = std::sin(angle);
                if (gimbal_frame != ctx.translation_gimbal_frame)
                {
                    (void)ctx.x_trajectory.Reset(0.0f);
                    (void)ctx.y_trajectory.Reset(0.0f);
                    ctx.translation_gimbal_frame = gimbal_frame;
                }
                /* 命令接口保持底盘坐标；先还原云台坐标，再规划平移，
                 * 最后按当前角度旋转，避免 S 曲线滞后于小陀螺方向变化。 */
                const float target_x = Chassis_TranslateX_FromSi(ctx.command.velocity_x_m_s);
                const float target_y = Chassis_TranslateY_FromSi(ctx.command.velocity_y_m_s);
                /* 先把 SI 仲裁边界的目标还原为老步兵抽象速度量纲。 */
                const bool x_valid = PlanAxis(
                    ctx.x_trajectory,
                    target_x * cosine + target_y * sine, &ctx.planned_x);
                const bool y_valid = PlanAxis(
                    ctx.y_trajectory,
                    -target_x * sine + target_y * cosine, &ctx.planned_y);
                const bool w_valid = PlanAxis(
                    ctx.w_trajectory,
                    Chassis_Rotation_FromSi(ctx.command.angular_velocity_rad_s), &ctx.planned_w);
                if (x_valid && y_valid && w_valid)
                {
                    const float planned_x = ctx.planned_x;
                    ctx.planned_x = planned_x * cosine - ctx.planned_y * sine;
                    ctx.planned_y = planned_x * sine + ctx.planned_y * cosine;
                    ControlWheels(ctx.planned_x, ctx.planned_y, ctx.planned_w);
                }
                else
                {
                    ControlWheels(0.0f, 0.0f, 0.0f);
                    for (uint8_t index = 0U; index < MOTOR_COUNT; ++index)
                    {
                        (void)ctx.wheel_motor[index].RequestEnabled(false);
                    }
                    (void)ctx.x_trajectory.Reset(0.0f);
                    (void)ctx.y_trajectory.Reset(0.0f);
                    (void)ctx.w_trajectory.Reset(0.0f);
                    ctx.planned_x = ctx.planned_y = ctx.planned_w = 0.0f;
                }
            }
            else
            {
                /* 失能时清零规划状态，恢复后从零速起步。 */
                (void)ctx.x_trajectory.Reset(0.0f);
                (void)ctx.y_trajectory.Reset(0.0f);
                (void)ctx.w_trajectory.Reset(0.0f);
                ctx.planned_x = ctx.planned_y = ctx.planned_w = 0.0f;
            }

            if (yaw_enabled)
            {
                /* INS 不可用时只停用角速度前馈，保留摇杆速度控制。 */
                ControlYaw(ins_valid ? ins_state.gyro_z_rad_s : 0.0f);
            }
            else
            {
                /* 失能期间清掉 Yaw 规划与 MIT 状态，恢复时从零速、中心阻尼起步。 */
                (void)ctx.yaw_trajectory.Reset(0.0f);
                ctx.yaw_speed = 0.0f;
                ctx.yaw_kd = kInfantryChassisConfig.yaw_mit_kd_center;
                ctx.yaw_torque_feedforward = 0.0f;
            }
        }
    }

    /* 电机控制按 2 ms 执行，反馈消息按 100 Hz 发布。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= FEEDBACK_DIVIDER)
    {
        ctx.feedback_divider = 0U;
        PublishFeedback(ins_valid);
    }
}

bool Chassis_GetGimbalImu(INS_State &state)
{
    return MessageCenter::Gimbal_INS_State_Topic.ReadFresh(state, 100000U);
}

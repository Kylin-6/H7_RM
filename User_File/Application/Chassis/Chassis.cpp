/**
 * @file Chassis.cpp
 * @brief 底盘应用。同一份文件用互斥的编译开关承载两套实现：
 *
 * - `LEGACY_INFANTRY_CHASSIS`：老步兵底盘板。四路 DM 麦轮（速度模式）负责底盘运动，
 *   另有一路挂在本板的 Yaw DM 电机（MIT 速度环）负责整机云台偏航轴。遥控整形、云台
 *   跟随与坐标旋转在 Input 层完成，本模块只做速率规划、麦轮逆运动学、Yaw 控制律与
 *   下发，并发布底盘反馈与 Yaw 轴反馈。
 * - `CHASSIS`：基于四个舵轮模块的 AGV 底盘，参考 Meta-Embedded-NG 移植。运动学与
 *   舵向最短路径规则来自 MIT 许可证下的 Meta-Embedded-NG application/chassis，
 *   实现已适配本工程 Class_DJIMotor 接口。机械参数仍是待实车标定值。
 * @todo vx/vy/wz 的实车物理正方向须按电机安装和坐标系标定，不能仅由数组顺序推断。
 * @todo 舵向依赖 output_total_angle；增量编码器上电不提供绝对输出轴零位，
 *       需绝对编码器、寻零流程或已知上电姿态。
 */

#include "Chassis.h"
#include "Chassis_Config.h"

#include "message_center.h"
#include "board_config.h"

static constexpr uint64_t CHASSIS_COMMAND_MAX_AGE_US = 100000U;

#if LEGACY_INFANTRY_CHASSIS
#include "alg_slope.h"
#include "dmmotor.h"
#include <cmath>
#endif

#if CHASSIS
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
#endif

#if LEGACY_INFANTRY_CHASSIS

namespace
{
/** INS 新鲜度门限；失效时只停用角速度前馈，保留遥控 Yaw 速度控制。 */
constexpr uint64_t LEGACY_INS_MAX_AGE_US = 10000U;
/** 电机数量与反馈发布分频（2 ms 控制路径下 10 分频 = 100 Hz）。 */
constexpr uint8_t LEGACY_MOTOR_COUNT = 4U;
constexpr uint8_t LEGACY_FEEDBACK_DIVIDER = 10U;

/** 以最大步长逼近目标，用于 MIT 阻尼与力矩前馈的逐周期限速。 */
float Legacy_MoveTowards(float current, float target, float maximum_delta)
{
    const float delta = target - current;

    if (std::fabs(delta) <= maximum_delta)
    {
        return target;
    }
    return current + (delta > 0.0f ? maximum_delta : -maximum_delta);
}

/**
 * @brief 单轴速率受限规划。
 * @details 非对称速率策略（反向先刹停、松手按释放减速度、加速/减速分别限幅）留在本层，
 *          斜坡本身由框架 Class_Slope 执行，数值与老工程
 *          SpeedPlanning_UpdateRateLimited 逐项对应。
 */
float Legacy_PlanAxis(Class_Slope &slope, float target, float acceleration_limit,
                      float deceleration_limit, float release_limit,
                      float reversal_limit)
{
    /* 零速吸附：小于门限的目标直接归零，避免电机长期爬极小的速度。 */
    float planned_target =
        std::fabs(target) <= kLegacyChassisConfig.planning_threshold ? 0.0f : target;
    const float current = slope.Get_Out();

    float rate_limit;
    if (current * planned_target < 0.0f)
    {
        /* 反向：先按反向减速度刹停，下一周期再反向加速。 */
        planned_target = 0.0f;
        rate_limit = reversal_limit;
    }
    else if (planned_target == 0.0f)
    {
        rate_limit = release_limit;
    }
    else if (std::fabs(planned_target) > std::fabs(current))
    {
        rate_limit = acceleration_limit;
    }
    else
    {
        rate_limit = deceleration_limit;
    }

    const float step =
        std::fmax(rate_limit, 0.0f) * kLegacyChassisConfig.control_dt_s;
    slope.Set_Increase_Value(step);
    slope.Set_Decrease_Value(step);
    slope.Set_Target(planned_target);
    slope.TIM_Calculate_PeriodElapsedCallback();
    return slope.Get_Out();
}

/**
 * @brief 失能路径：把已规划速度按释放减速度拉回零。
 * @details 目标为 0 时速率策略只用到释放减速度，加速/减速上限不参与判断；本函数只维护
 *          规划状态与反馈，不产生任何电机输出，重新使能时从零速平滑起步。
 */
float Legacy_PlanToZero(Class_Slope &slope, float release_limit, float reversal_limit)
{
    return Legacy_PlanAxis(slope, 0.0f, release_limit, release_limit, release_limit,
                           reversal_limit);
}

struct LegacyChassisContext
{
    Publisher<ChassisFeedback> chassis_feedback_publisher{
        MessageCenter::Chassis_Feedback_Topic};
    /** Yaw 轴挂在本板，但对外仍是云台偏航轴，因此按 GimbalFeedback 发布。 */
    Publisher<GimbalFeedback> yaw_feedback_publisher{
        MessageCenter::Gimbal_Feedback_Topic};
    Class_DMMotor wheel_motor[LEGACY_MOTOR_COUNT];
    Class_DMMotor yaw_motor;
    Class_Slope x_slope;
    Class_Slope y_slope;
    Class_Slope w_slope;
    Class_Slope yaw_slope;
    ChassisCmd command{};
    GimbalCmd yaw_command{};
    ChassisFeedback feedback{};
    GimbalFeedback yaw_feedback{};
    float planned_x = 0.0f;
    float planned_y = 0.0f;
    float planned_w = 0.0f;
    float yaw_speed = 0.0f;
    float yaw_kd = kLegacyChassisConfig.yaw_mit_kd_center;
    float yaw_torque_feedforward = 0.0f;
    uint8_t control_divider = 0U;
    uint8_t feedback_divider = 0U;
    bool initialized = false;
};

LegacyChassisContext ctx;

/**
 * @brief 麦轮逆运动学：三轴速度直接代数组合成四轮目标。
 * @note 与老工程一致，不做轮距与半径换算（三轴本身已是轮速量纲），最后整轮限幅。
 */
void Legacy_ControlWheels(float velocity_x, float velocity_y, float velocity_w)
{
    float wheel_speed[LEGACY_MOTOR_COUNT];
    wheel_speed[0] = velocity_y + velocity_x + velocity_w;
    wheel_speed[1] = velocity_y - velocity_x + velocity_w;
    wheel_speed[2] = -velocity_y - velocity_x + velocity_w;
    wheel_speed[3] = velocity_x - velocity_y + velocity_w;

    for (uint8_t index = 0U; index < LEGACY_MOTOR_COUNT; ++index)
    {
        const float limited = Basic_Math_Constrain(wheel_speed[index],
                                                  -kLegacyChassisConfig.wheel_speed_max,
                                                   kLegacyChassisConfig.wheel_speed_max);
        (void)ctx.wheel_motor[index].SetSpeed(limited);
    }
}

/**
 * @brief Yaw 轴 MIT 速度环。
 * @details 摇杆速度减去底盘自转角速度前馈后用非对称速率规划平滑；MIT 位置增益恒为 0、
 *          位置目标恒为 0，阻尼随摇杆推进减小、反向瞬间提高，力矩前馈由规划加速度换算。
 */
void Legacy_ControlYaw(float chassis_yaw_rate_rad_s)
{
    const float stick_speed = Basic_Math_Constrain(
        ctx.yaw_command.yaw_speed_rad_s,
        -kLegacyChassisConfig.yaw_speed_max_rad_s,
        kLegacyChassisConfig.yaw_speed_max_rad_s);

    float target_speed = stick_speed -
                         kLegacyChassisConfig.yaw_rate_feedforward_gain *
                             chassis_yaw_rate_rad_s;
    target_speed = Basic_Math_Constrain(target_speed,
                                        -kLegacyChassisConfig.yaw_speed_max_rad_s,
                                        kLegacyChassisConfig.yaw_speed_max_rad_s);

    const float stick_ratio =
        std::fabs(stick_speed) / kLegacyChassisConfig.yaw_speed_max_rad_s;
    const float acceleration_limit =
        kLegacyChassisConfig.yaw_accel_limit_min +
        (kLegacyChassisConfig.yaw_accel_limit_max -
         kLegacyChassisConfig.yaw_accel_limit_min) *
            stick_ratio;

    const float speed_previous = ctx.yaw_speed;
    ctx.yaw_speed = Legacy_PlanAxis(ctx.yaw_slope, target_speed, acceleration_limit,
                                    kLegacyChassisConfig.yaw_decel_limit,
                                    kLegacyChassisConfig.yaw_release_limit,
                                    kLegacyChassisConfig.yaw_reverse_limit);

    float kd_target = kLegacyChassisConfig.yaw_mit_kd_center +
                      (kLegacyChassisConfig.yaw_mit_kd_moving -
                       kLegacyChassisConfig.yaw_mit_kd_center) *
                          stick_ratio;
    if (speed_previous * stick_speed < 0.0f)
    {
        kd_target = kLegacyChassisConfig.yaw_mit_kd_reverse;
    }
    ctx.yaw_kd = Legacy_MoveTowards(ctx.yaw_kd, kd_target,
                                    kLegacyChassisConfig.yaw_mit_kd_slew_rate *
                                        kLegacyChassisConfig.control_dt_s);

    const float acceleration_command =
        (ctx.yaw_speed - speed_previous) / kLegacyChassisConfig.control_dt_s;
    const float torque_target = Basic_Math_Constrain(
        acceleration_command * kLegacyChassisConfig.yaw_mit_torque_gain,
        -kLegacyChassisConfig.yaw_mit_torque_max_nm,
        kLegacyChassisConfig.yaw_mit_torque_max_nm);
    ctx.yaw_torque_feedforward = Legacy_MoveTowards(
        ctx.yaw_torque_feedforward, torque_target,
        kLegacyChassisConfig.yaw_mit_torque_slew_rate *
            kLegacyChassisConfig.control_dt_s);

    /* 位置目标 0、位置增益 0：速度由电机内部闭环，阻尼与外力矩由本层给。 */
    (void)ctx.yaw_motor.SetMIT(0.0f, ctx.yaw_speed, kLegacyChassisConfig.yaw_mit_kp,
                              ctx.yaw_kd, ctx.yaw_torque_feedforward);
}

/** 100 Hz 发布底盘反馈与 Yaw 轴反馈；未就绪或失效状态如实上报。 */
void Legacy_PublishFeedback(bool ins_valid)
{
    bool online = true;
    bool ready = true;
    for (uint8_t index = 0U; index < LEGACY_MOTOR_COUNT; ++index)
    {
        online = online && ctx.wheel_motor[index].IsOnline();
        ready = ready && ctx.wheel_motor[index].GetFeedbackSnapshot().ready;
    }

    /* 本板不测量真实车体速度：按输入边界约定回写规划值（SI 归一化），
     * 与 Input 提交的量纲一致；enabled 表示四轮均就绪且当前不是 ZERO_FORCE。 */
    ctx.feedback.velocity_x_m_s = LegacyChassis_TranslateX_ToSi(ctx.planned_x);
    ctx.feedback.velocity_y_m_s = LegacyChassis_TranslateY_ToSi(ctx.planned_y);
    ctx.feedback.angular_velocity_rad_s = LegacyChassis_Rotation_ToSi(ctx.planned_w);
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

#elif CHASSIS

namespace
{
struct ChassisContext
{
    Publisher<ChassisFeedback> feedback_publisher{MessageCenter::Chassis_Feedback_Topic};
    ChassisCmd command{};
    ChassisFeedback feedback{};
    uint8_t feedback_divider = 0U;
    Class_DJIMotor wheel_motor[4];
    Class_DJIMotor steer_motor[4];
    Struct_DJIMotor_Motion_Snapshot wheel_snapshot[4];
    Struct_DJIMotor_Motion_Snapshot steer_snapshot[4];
    Class_DJIMotor_Group wheel_group;
    Class_DJIMotor_Group steer_group;
    bool initialized = false;
    int8_t wheel_direction[4] = {1, 1, 1, 1};
};

ChassisContext ctx;
}

static PID_InitTypeDef Chassis_MakePID(const ChassisPidConfig &config)
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

static void Chassis_CalculateTargets(float wheel_target_rad_s[4],
                                     float steer_target_rad[4])
{
    /* 四轮位置的旋转项为 ±wz·半宽/半长；符号按下方轮索引数组固定。
       物理前/左和正转方向必须由实车接线及坐标标定确认。 */
    const float vx_m_s = ctx.command.velocity_x_m_s;
    const float vy_m_s = ctx.command.velocity_y_m_s;
    const float wz_rad_s = ctx.command.angular_velocity_rad_s;
    const float wheel_vx[4] = {
        vx_m_s + wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s + wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s - wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s - wz_rad_s * kChassisConfig.half_width_m,
    };
    const float wheel_vy[4] = {
        vy_m_s + wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s - wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s - wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s + wz_rad_s * kChassisConfig.half_length_m,
    };

    for (uint8_t index = 0; index < 4; ++index)
    {
        const float velocity_m_s = std::sqrt(wheel_vx[index] * wheel_vx[index] +
                                            wheel_vy[index] * wheel_vy[index]);
        const float current_angle_rad =
            ctx.steer_snapshot[index].output_total_angle;
        if (velocity_m_s < kChassisConfig.stop_speed_m_s)
        {
            // 近零轮速时保持当前舵角，避免 atan2 的方向随微小输入跳变。
            wheel_target_rad_s[index] = 0.0f;
            steer_target_rad[index] = current_angle_rad;
            continue;
        }

        const float target_angle_rad = std::atan2(wheel_vy[index], wheel_vx[index]) +
                                       kChassisConfig.steer_offset_rad[index];
        float difference_rad = std::remainder(target_angle_rad - current_angle_rad,
                                              2.0f * kPiRad);
        /* remainder 将误差压到 [-π, π]；超过 ±π/2 时舵角少转 π、轮速取反。 */
        if (difference_rad > kPiRad / 2.0f)
        {
            difference_rad -= kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else if (difference_rad < -kPiRad / 2.0f)
        {
            difference_rad += kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else
        {
            ctx.wheel_direction[index] = 1;
        }

        steer_target_rad[index] = current_angle_rad + difference_rad;
        // 线速度除以轮半径得到输出轴 rad/s；后续闭环由 DJI 驱动的现有 PID 执行。
        wheel_target_rad_s[index] = (velocity_m_s / kChassisConfig.wheel_radius_m) *
                                    ctx.wheel_direction[index];
    }
}

static void Chassis_UpdateFeedback(void)
{
    float wheel_vx[4];
    float wheel_vy[4];
    bool online = true;
    bool ready = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        const float heading_rad =
            ctx.steer_snapshot[index].output_total_angle -
            kChassisConfig.steer_offset_rad[index];
        const float linear_speed_m_s =
            ctx.wheel_snapshot[index].output_speed *
            kChassisConfig.wheel_radius_m;
        wheel_vx[index] = linear_speed_m_s * std::cos(heading_rad);
        wheel_vy[index] = linear_speed_m_s * std::sin(heading_rad);
        online = online && ctx.wheel_snapshot[index].online &&
                 ctx.steer_snapshot[index].online;
        ready = ready && ctx.wheel_snapshot[index].ready &&
                ctx.steer_snapshot[index].ready;
    }

    const float vx = (wheel_vx[0] + wheel_vx[1] + wheel_vx[2] + wheel_vx[3]) * 0.25f;
    const float vy = (wheel_vy[0] + wheel_vy[1] + wheel_vy[2] + wheel_vy[3]) * 0.25f;
    const float wz_x = ((wheel_vx[0] - wheel_vx[2]) +
                        (wheel_vx[1] - wheel_vx[3])) /
                       (4.0f * kChassisConfig.half_width_m);
    const float wz_y = ((wheel_vy[0] - wheel_vy[1]) +
                        (wheel_vy[3] - wheel_vy[2])) /
                       (4.0f * kChassisConfig.half_length_m);

    // 一阶平滑 y += alpha * (x - y)，在 1 kHz 控制周期更新；100 Hz 仅是发布频率。
    // 初始输出沿用初始化时的零值，feedback_alpha 是每个控制周期的权重。
    ctx.feedback.velocity_x_m_s += kChassisConfig.feedback_alpha *
        (vx - ctx.feedback.velocity_x_m_s);
    ctx.feedback.velocity_y_m_s += kChassisConfig.feedback_alpha *
        (vy - ctx.feedback.velocity_y_m_s);
    ctx.feedback.angular_velocity_rad_s += kChassisConfig.feedback_alpha *
        (0.5f * (wz_x + wz_y) - ctx.feedback.angular_velocity_rad_s);
    ctx.feedback.enabled = ctx.command.mode != ChassisMode::ZERO_FORCE && ready;
    ctx.feedback.online = online;
}
#else /* 无底盘硬件路径 */

namespace
{
/** 未启用底盘硬件路径时的空骨架：只保留消息端点与状态，不访问任何电机。 */
struct ChassisContext
{
    Publisher<ChassisFeedback> feedback_publisher{MessageCenter::Chassis_Feedback_Topic};
    ChassisCmd command{};
    ChassisFeedback feedback{};
    uint8_t feedback_divider = 0U;
};

ChassisContext ctx;
} // namespace

#endif /* 无底盘硬件路径 */

#if LEGACY_INFANTRY_CHASSIS
Struct_Chassis_Diagnostic_Input Chassis_GetDiagnostic(void)
{
    Struct_Chassis_Diagnostic_Input d{};
    d.initialized = ctx.initialized;
    INS_State ins{};
    d.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ins, LEGACY_INS_MAX_AGE_US);
    d.permitted = ctx.command.mode != ChassisMode::ZERO_FORCE ||
                  ctx.yaw_command.mode == GimbalMode::IMU;
    for (uint8_t i = 0U; i < 5U; ++i)
        d.motor[i] = i < 4U ? ctx.wheel_motor[i].GetFeedbackSnapshot()
                           : ctx.yaw_motor.GetFeedbackSnapshot();
    return d;
}
#endif

bool Chassis_Init(void)
{
    ctx.command = {};
    ctx.feedback = {};
    ctx.feedback_divider = 0U;

#if LEGACY_INFANTRY_CHASSIS
    /* 本文件内 legacy 段优先于框架舵轮段（见上方 #if/#elif），CHASSIS 宏只用于
     * board 装配一致性，这里不再单独判重。 */
    ctx.yaw_command = {};
    ctx.yaw_feedback = {};
    ctx.yaw_speed = 0.0f;
    ctx.yaw_kd = kLegacyChassisConfig.yaw_mit_kd_center;
    ctx.yaw_torque_feedforward = 0.0f;
    ctx.control_divider = 0U;

    bool initialized = true;
    for (uint8_t index = 0U; index < LEGACY_MOTOR_COUNT; ++index)
    {
        initialized = ctx.wheel_motor[index].Init(
                          BoardConfig_Get().chassis_wheel_bus,
                          kLegacyChassisConfig.motor_id[index],
                          kLegacyChassisConfig.motor_master_id[index],
                          Enum_DMMotor_Mode::SPEED, false,
                          kLegacyChassisConfig.motor_position_max_rad,
                          kLegacyChassisConfig.motor_velocity_max_rad_s,
                          kLegacyChassisConfig.motor_torque_max_nm) &&
                      initialized;
    }
    initialized = ctx.yaw_motor.Init(
                      BoardConfig_Get().gimbal_yaw_bus,
                      kLegacyChassisConfig.yaw_motor_id,
                      kLegacyChassisConfig.yaw_motor_master_id,
                      Enum_DMMotor_Mode::MIT, false,
                      kLegacyChassisConfig.yaw_position_max_rad,
                      kLegacyChassisConfig.yaw_velocity_max_rad_s,
                      kLegacyChassisConfig.yaw_torque_max_nm) &&
                  initialized;

    /* 三个受控轴都从零速起步；Class_Slope 用目标值优先，逐周期速率由规划层给。 */
    ctx.x_slope.Init(0.0f, 0.0f, Slope_First_TARGET);
    ctx.y_slope.Init(0.0f, 0.0f, Slope_First_TARGET);
    ctx.w_slope.Init(0.0f, 0.0f, Slope_First_TARGET);
    ctx.yaw_slope.Init(0.0f, 0.0f, Slope_First_TARGET);

    /* 上电默认失能：先落 Yaw 安全目标，再对全部已配置电机请求失能；
     * 即使部分电机注册失败也尝试停住它们。 */
    (void)ctx.yaw_motor.SetMIT(0.0f, 0.0f, kLegacyChassisConfig.yaw_mit_kp,
                              ctx.yaw_kd, 0.0f);
    (void)ctx.yaw_motor.RequestEnabled(false);
    for (uint8_t index = 0U; index < LEGACY_MOTOR_COUNT; ++index)
    {
        (void)ctx.wheel_motor[index].RequestEnabled(false);
    }

    ctx.initialized = initialized;
    return initialized;
#elif CHASSIS
    Struct_DJIMotor_Init_Config wheel_config{};
    wheel_config.hfdcan = BoardConfig_Get().chassis_wheel_bus;
    wheel_config.motor_type = Enum_DJIMotor_Type::M3508;
    wheel_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    wheel_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入现为 rad/s；以下增益来源未标定，需实车重新整定。
    wheel_config.speed_pid = Chassis_MakePID(kChassisConfig.wheel_speed_pid);

    Struct_DJIMotor_Init_Config steer_config{};
    steer_config.hfdcan = BoardConfig_Get().chassis_steer_bus;
    steer_config.motor_type = Enum_DJIMotor_Type::M3508;
    steer_config.close_loop = DJI_MOTOR_ANGLE_LOOP | DJI_MOTOR_SPEED_LOOP;
    steer_config.outer_loop = DJI_MOTOR_ANGLE_LOOP;
    // 角度环输出为 rad/s：原 200/1000 deg/s 限幅作物理等效转换。
    // Kp/Ki 和舵轮速度环增益没有可信实车来源，启用前均需重新整定。
    steer_config.angle_pid = Chassis_MakePID(kChassisConfig.steer_angle_pid);
    steer_config.speed_pid = Chassis_MakePID(kChassisConfig.steer_speed_pid);

    bool initialized = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        wheel_config.can_id = kChassisConfig.motor_id[index];
        steer_config.can_id = kChassisConfig.motor_id[index];
        initialized = ctx.wheel_motor[index].Init(wheel_config) && initialized;
        initialized = ctx.steer_motor[index].Init(steer_config) && initialized;
    }
    initialized = initialized && ctx.wheel_group.Init(
        &ctx.wheel_motor[0], &ctx.wheel_motor[1],
        &ctx.wheel_motor[2], &ctx.wheel_motor[3]);
    initialized = initialized && ctx.steer_group.Init(
        &ctx.steer_motor[0], &ctx.steer_motor[1],
        &ctx.steer_motor[2], &ctx.steer_motor[3]);
    ctx.initialized = initialized;
    if (initialized)
    {
        (void)ctx.wheel_group.RequestEnabled(false);
        (void)ctx.steer_group.RequestEnabled(false);
    }
    return initialized;
#else
    return true;
#endif
}

void Chassis_Update(void)
{
    /* 仅在命令 Topic 仍新鲜时沿用目标；过期后使用默认 ZERO_FORCE 关闭输出。 */
    ChassisCmd command{};
    if (MessageCenter::Chassis_Command_Topic.ReadFresh(
            command, CHASSIS_COMMAND_MAX_AGE_US))
    {
        ctx.command = command;
    }
    else
    {
        ctx.command = {};
    }

#if LEGACY_INFANTRY_CHASSIS
    /*
     * Yaw 命令按框架云台命令的语义读取：RobotCmd 只在目标变化或安全撤销时发布，
     * 因此用最新值（ReadWithMeta）而不是时效判定，否则摇杆保持不动会被判为过期而
     * 误失能；遥控失联时 RobotCmd 会把命令撤销为 DISABLED。
     */
    const auto yaw_message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.yaw_command = yaw_message.valid ? yaw_message.data : GimbalCmd{};

    INS_State ins_state{};
    const bool ins_valid =
        MessageCenter::INS_State_Topic.ReadFresh(ins_state, LEGACY_INS_MAX_AGE_US);

    if (ctx.initialized)
    {
        const bool wheels_enabled = ctx.command.mode != ChassisMode::ZERO_FORCE;
        const bool yaw_enabled = ctx.yaw_command.mode == GimbalMode::IMU;

        /* 使能请求是边沿语义：首次请求与状态变化才产生总线流量，
         * 掉线补发由 StatusTask 的 100 Hz ServiceAll 负责。 */
        for (uint8_t index = 0U; index < LEGACY_MOTOR_COUNT; ++index)
        {
            (void)ctx.wheel_motor[index].RequestEnabled(wheels_enabled);
        }
        (void)ctx.yaw_motor.RequestEnabled(yaw_enabled);

        /* 老步兵控制路径按 2 ms 执行：与老工程一致，同时把 DM 速度帧数量
         * 压回 FDCAN1 的承载范围内（周期通道只在目标更新时才发送）。 */
        ctx.control_divider++;
        if (ctx.control_divider >= kLegacyChassisConfig.control_divider)
        {
            ctx.control_divider = 0U;

            if (wheels_enabled)
            {
                /* 先把 SI 仲裁边界的目标还原为老步兵抽象速度量纲。 */
                ctx.planned_x = Legacy_PlanAxis(
                    ctx.x_slope,
                    LegacyChassis_TranslateX_FromSi(ctx.command.velocity_x_m_s),
                    kLegacyChassisConfig.x_accel_limit,
                    kLegacyChassisConfig.x_decel_limit,
                    kLegacyChassisConfig.x_release_limit,
                    kLegacyChassisConfig.x_reverse_limit);
                ctx.planned_y = Legacy_PlanAxis(
                    ctx.y_slope,
                    LegacyChassis_TranslateY_FromSi(ctx.command.velocity_y_m_s),
                    kLegacyChassisConfig.y_accel_limit,
                    kLegacyChassisConfig.y_decel_limit,
                    kLegacyChassisConfig.y_release_limit,
                    kLegacyChassisConfig.y_reverse_limit);
                ctx.planned_w = Legacy_PlanAxis(
                    ctx.w_slope,
                    LegacyChassis_Rotation_FromSi(
                        ctx.command.angular_velocity_rad_s),
                    kLegacyChassisConfig.w_accel_limit,
                    kLegacyChassisConfig.w_decel_limit,
                    kLegacyChassisConfig.w_release_limit,
                    kLegacyChassisConfig.w_reverse_limit);

                Legacy_ControlWheels(ctx.planned_x, ctx.planned_y, ctx.planned_w);
            }
            else
            {
                /* 失能期间让三轴规划按释放减速度回零，重新使能时从零速平滑起步。 */
                ctx.planned_x = Legacy_PlanToZero(
                    ctx.x_slope, kLegacyChassisConfig.x_release_limit,
                    kLegacyChassisConfig.x_reverse_limit);
                ctx.planned_y = Legacy_PlanToZero(
                    ctx.y_slope, kLegacyChassisConfig.y_release_limit,
                    kLegacyChassisConfig.y_reverse_limit);
                ctx.planned_w = Legacy_PlanToZero(
                    ctx.w_slope, kLegacyChassisConfig.w_release_limit,
                    kLegacyChassisConfig.w_reverse_limit);
            }

            if (yaw_enabled)
            {
                /* INS 不可用时只停用角速度前馈，保留摇杆速度控制。 */
                Legacy_ControlYaw(ins_valid ? ins_state.gyro_z_rad_s : 0.0f);
            }
            else
            {
                /* 失能期间清掉 Yaw 规划与 MIT 状态，恢复时从零速、中心阻尼起步。 */
                ctx.yaw_speed = Legacy_PlanToZero(
                    ctx.yaw_slope, kLegacyChassisConfig.yaw_release_limit,
                    kLegacyChassisConfig.yaw_reverse_limit);
                ctx.yaw_kd = kLegacyChassisConfig.yaw_mit_kd_center;
                ctx.yaw_torque_feedforward = 0.0f;
            }
        }
    }

    /* 电机控制按 2 ms 执行，反馈消息按 100 Hz 发布。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= LEGACY_FEEDBACK_DIVIDER)
    {
        ctx.feedback_divider = 0U;
        Legacy_PublishFeedback(ins_valid);
    }
#elif CHASSIS
    if (ctx.initialized)
    {
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            ctx.wheel_snapshot[index] = ctx.wheel_motor[index].GetMotionSnapshot();
            ctx.steer_snapshot[index] = ctx.steer_motor[index].GetMotionSnapshot();
        }
        const bool enabled = ctx.command.mode != ChassisMode::ZERO_FORCE;
        (void)ctx.wheel_group.RequestEnabled(enabled);
        (void)ctx.steer_group.RequestEnabled(enabled);
        if (enabled)
        {
            float wheel_target_rad_s[4];
            float steer_target_rad[4];
            Chassis_CalculateTargets(wheel_target_rad_s, steer_target_rad);
            ctx.wheel_group.Control(wheel_target_rad_s[0], wheel_target_rad_s[1],
                                        wheel_target_rad_s[2], wheel_target_rad_s[3]);
            ctx.steer_group.Control(steer_target_rad[0], steer_target_rad[1],
                                        steer_target_rad[2], steer_target_rad[3]);
        }
        Chassis_UpdateFeedback();
    }

    /* 电机控制按 1 kHz 执行，反馈消息按 100 Hz 发布。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
#else
    /* 电机控制按 1 kHz 执行，反馈消息按 100 Hz 发布。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
#endif
}

/**
 * @file Chassis.cpp
 * @brief 底盘应用。同一份文件用互斥的编译开关承载两套实现：
 *
 * - `LEGACY_INFANTRY`：老步兵 DM 四电机麦轮底盘（移植自 rm/demo 的 APP/ChassisTask.c
 *   与 User/algorithm/CHASSIS_ALG.c）。遥控整形与云台跟随在 Communication 层完成，
 *   本层只负责速度规划、麦轮逆运动学与电机下发。
 * - `CHASSIS`：基于四个舵轮模块的 AGV 底盘，参考 Meta-Embedded-NG 移植。运动学与
 *   舵向最短路径规则来自 MIT 许可证下的 Meta-Embedded-NG application/chassis，
 *   实现已适配本工程 Class_DJIMotor 接口。机械参数仍是待实车标定值。
 *
 * 两套实现默认都不参与固件构建。
 */

#include "Chassis.h"

#include "message_center.h"

#if LEGACY_INFANTRY
#include "SpeedPlanning.h"
#include "dmmotor.h"
#include "fdcan.h"
#include <cmath>
#endif

#if CHASSIS
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
#endif

static Subscriber<ChassisCmd> Chassis_Command_Subscriber(
    MessageCenter::Chassis_Command_Topic);
static Publisher<ChassisFeedback> Chassis_Feedback_Publisher(
    MessageCenter::Chassis_Feedback_Topic);
static ChassisCmd Chassis_Command;
static ChassisFeedback Chassis_Feedback;
static uint8_t Chassis_Feedback_Divider;

#if LEGACY_INFANTRY

/* ============================== 控制参数 ============================== */

/** 单轮速度限幅，沿用老步兵原始速度量纲。 */
static constexpr float CHASSIS_WHEEL_SPEED_MAX = 30.0f;
/** 速度规划控制周期，与 Control_Task 的 1 kHz 调度一致。 */
static constexpr float CHASSIS_CONTROL_DT = 0.001f;
/** 零速吸附门限。 */
static constexpr float CHASSIS_PLANNING_THRESHOLD = 0.1f;

/** 三轴非对称速率限制，单位“速度单位/秒”。 */
static constexpr float CHASSIS_X_ACCEL_LIMIT = 180.0f;
static constexpr float CHASSIS_X_DECEL_LIMIT = 180.0f;
static constexpr float CHASSIS_X_RELEASE_LIMIT = 120.0f;
static constexpr float CHASSIS_X_REVERSE_LIMIT = 300.0f;
static constexpr float CHASSIS_Y_ACCEL_LIMIT = 180.0f;
static constexpr float CHASSIS_Y_DECEL_LIMIT = 180.0f;
static constexpr float CHASSIS_Y_RELEASE_LIMIT = 120.0f;
static constexpr float CHASSIS_Y_REVERSE_LIMIT = 300.0f;
static constexpr float CHASSIS_W_ACCEL_LIMIT = 350.0f;
static constexpr float CHASSIS_W_DECEL_LIMIT = 350.0f;
static constexpr float CHASSIS_W_RELEASE_LIMIT = 250.0f;
static constexpr float CHASSIS_W_REVERSE_LIMIT = 600.0f;

/** 四个底盘 DM 电机在 FDCAN1 上的节点 ID 与主控接收 ID，与 demo 的 bsp_CAN.c 一致。 */
static constexpr uint8_t CHASSIS_MOTOR_CAN_ID[4] = {0x50U, 0x51U, 0x52U, 0x53U};
static constexpr uint16_t CHASSIS_MOTOR_MASTER_ID[4] = {0x60U, 0x61U, 0x62U, 0x63U};
/** 底盘电机反馈量程：位置 ±3.14 rad、速度 ±200 rad/s、转矩 ±10 N·m（DM3519）。 */
static constexpr float CHASSIS_MOTOR_POSITION_MAX_RAD = 3.14f;
static constexpr float CHASSIS_MOTOR_VELOCITY_MAX_RAD_S = 200.0f;
static constexpr float CHASSIS_MOTOR_TORQUE_MAX_NM = 10.0f;

static Class_DMMotor Chassis_Motor[4];
static SpeedPlanningState Chassis_X_Planning;
static SpeedPlanningState Chassis_Y_Planning;
static SpeedPlanningState Chassis_W_Planning;
static bool Chassis_Initialized;
static bool Chassis_Output_Enabled;
static float Chassis_Planned_Velocity_X;
static float Chassis_Planned_Velocity_Y;
static float Chassis_Planned_Velocity_W;

/** 使能或失能四台底盘电机；状态未变化时不重复下发命令。 */
static void Chassis_SetEnabled(bool enabled)
{
    if (enabled == Chassis_Output_Enabled)
    {
        return;
    }
    Chassis_Output_Enabled = enabled;

    for (uint32_t index = 0U; index < 4U; ++index)
    {
        if (enabled)
        {
            (void)Chassis_Motor[index].Enable();
        }
        else
        {
            (void)Chassis_Motor[index].Disable();
        }
    }
}

/**
 * @brief 麦轮逆运动学：把底盘三轴速度分解为四轮目标速度并下发。
 * @details 与 demo 的 Chassis_Analysis_Vel 一致：轮速由 vx/vy/w 直接代数组合，
 *          不做轮距与半径换算，最后整轮限幅。
 * @note 原实现会把组合结果强制转换为 int16_t 再赋回 float，本版本保留浮点精度。
 */
static void Chassis_ControlMotors(float velocity_x, float velocity_y, float velocity_w)
{
    float wheel_speed[4];
    wheel_speed[0] = velocity_y + velocity_x + velocity_w;
    wheel_speed[1] = velocity_y - velocity_x + velocity_w;
    wheel_speed[2] = -velocity_y - velocity_x + velocity_w;
    wheel_speed[3] = velocity_x - velocity_y + velocity_w;

    for (uint32_t index = 0U; index < 4U; ++index)
    {
        if (wheel_speed[index] > CHASSIS_WHEEL_SPEED_MAX)
        {
            wheel_speed[index] = CHASSIS_WHEEL_SPEED_MAX;
        }
        else if (wheel_speed[index] < -CHASSIS_WHEEL_SPEED_MAX)
        {
            wheel_speed[index] = -CHASSIS_WHEEL_SPEED_MAX;
        }

        Chassis_Motor[index].SetSpeed(wheel_speed[index]);
    }
}

#endif /* LEGACY_INFANTRY */

#if CHASSIS
static constexpr float CHASSIS_HALF_LENGTH_M = 0.163f;
static constexpr float CHASSIS_HALF_WIDTH_M = 0.163f;
static constexpr float CHASSIS_WHEEL_RADIUS_M = 0.058f;
static constexpr float CHASSIS_WHEEL_PERIMETER_M =
    2.0f * 3.14159265358979323846f * CHASSIS_WHEEL_RADIUS_M;
static constexpr float CHASSIS_FEEDBACK_ALPHA = 0.032258f;
static constexpr float CHASSIS_STOP_SPEED_M_S = 0.001f;
static constexpr float Chassis_Steer_Offset_Deg[4] = {
    102.5f, 12.5f, 137.5f, 145.0f};

static Class_DJIMotor Chassis_Wheel_Motor[4];
static Class_DJIMotor Chassis_Steer_Motor[4];
static Class_DJIMotor_Group Chassis_Wheel_Group;
static Class_DJIMotor_Group Chassis_Steer_Group;
static bool Chassis_Initialized;
static bool Chassis_Output_Enabled;
static int8_t Chassis_Wheel_Direction[4] = {1, 1, 1, 1};

static PID_InitTypeDef Chassis_MakePID(float kp, float ki, float kd,
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

static float Chassis_NormalizeAngle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void Chassis_SetEnabled(bool enabled)
{
    if (enabled == Chassis_Output_Enabled)
    {
        return;
    }
    Chassis_Output_Enabled = enabled;
    if (enabled)
    {
        Chassis_Wheel_Group.Enable();
        Chassis_Steer_Group.Enable();
    }
    else
    {
        Chassis_Wheel_Group.Disable();
        Chassis_Steer_Group.Disable();
    }
}

static void Chassis_CalculateTargets(float wheel_target[4], float steer_target[4])
{
    /* 将底盘坐标系速度分解为四个舵轮各自的平移速度向量。 */
    const float vx = Chassis_Command.velocity_x_m_s;
    const float vy = Chassis_Command.velocity_y_m_s;
    const float wz = Chassis_Command.angular_velocity_rad_s;
    const float wheel_vx[4] = {
        vx + wz * CHASSIS_HALF_WIDTH_M,
        vx + wz * CHASSIS_HALF_WIDTH_M,
        vx - wz * CHASSIS_HALF_WIDTH_M,
        vx - wz * CHASSIS_HALF_WIDTH_M,
    };
    const float wheel_vy[4] = {
        vy + wz * CHASSIS_HALF_LENGTH_M,
        vy - wz * CHASSIS_HALF_LENGTH_M,
        vy - wz * CHASSIS_HALF_LENGTH_M,
        vy + wz * CHASSIS_HALF_LENGTH_M,
    };

    for (uint8_t index = 0; index < 4; ++index)
    {
        const float velocity = std::sqrt(wheel_vx[index] * wheel_vx[index] +
                                         wheel_vy[index] * wheel_vy[index]);
        const float current_angle =
            Chassis_Steer_Motor[index].feedback.output_total_angle;
        if (velocity < CHASSIS_STOP_SPEED_M_S)
        {
            wheel_target[index] = 0.0f;
            steer_target[index] = current_angle;
            continue;
        }

        float target_angle = std::atan2(wheel_vy[index], wheel_vx[index]) *
                             (180.0f / 3.14159265358979323846f) +
                             Chassis_Steer_Offset_Deg[index];
        float difference = Chassis_NormalizeAngle(target_angle - current_angle);
        /* 舵向误差超过 90° 时反转轮速，缩短舵电机需要旋转的路径。 */
        if (difference > 90.0f)
        {
            difference -= 180.0f;
            Chassis_Wheel_Direction[index] = -1;
        }
        else if (difference < -90.0f)
        {
            difference += 180.0f;
            Chassis_Wheel_Direction[index] = -1;
        }
        else
        {
            Chassis_Wheel_Direction[index] = 1;
        }

        steer_target[index] = current_angle + difference;
        wheel_target[index] = velocity * (360.0f / CHASSIS_WHEEL_PERIMETER_M) *
                              Chassis_Wheel_Direction[index];
    }
}

static void Chassis_UpdateFeedback(void)
{
    float wheel_vx[4];
    float wheel_vy[4];
    bool online = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        const float heading =
            (Chassis_Steer_Motor[index].feedback.output_total_angle -
             Chassis_Steer_Offset_Deg[index]) *
            (3.14159265358979323846f / 180.0f);
        const float linear_speed =
            Chassis_Wheel_Motor[index].feedback.output_speed *
            (CHASSIS_WHEEL_PERIMETER_M / 360.0f);
        wheel_vx[index] = linear_speed * std::cos(heading);
        wheel_vy[index] = linear_speed * std::sin(heading);
        online = online && Chassis_Wheel_Motor[index].online &&
                 Chassis_Steer_Motor[index].online;
    }

    const float vx = (wheel_vx[0] + wheel_vx[1] + wheel_vx[2] + wheel_vx[3]) * 0.25f;
    const float vy = (wheel_vy[0] + wheel_vy[1] + wheel_vy[2] + wheel_vy[3]) * 0.25f;
    const float wz_x = ((wheel_vx[0] - wheel_vx[2]) +
                        (wheel_vx[1] - wheel_vx[3])) /
                       (4.0f * CHASSIS_HALF_WIDTH_M);
    const float wz_y = ((wheel_vy[0] - wheel_vy[1]) +
                        (wheel_vy[3] - wheel_vy[2])) /
                       (4.0f * CHASSIS_HALF_LENGTH_M);

    Chassis_Feedback.velocity_x_m_s += CHASSIS_FEEDBACK_ALPHA *
        (vx - Chassis_Feedback.velocity_x_m_s);
    Chassis_Feedback.velocity_y_m_s += CHASSIS_FEEDBACK_ALPHA *
        (vy - Chassis_Feedback.velocity_y_m_s);
    Chassis_Feedback.angular_velocity_rad_s += CHASSIS_FEEDBACK_ALPHA *
        (0.5f * (wz_x + wz_y) - Chassis_Feedback.angular_velocity_rad_s);
    Chassis_Feedback.enabled = Chassis_Output_Enabled;
    Chassis_Feedback.online = online;
}
#endif

bool Chassis_Init(void)
{
    Chassis_Command = {};
    Chassis_Feedback = {};
    Chassis_Feedback_Divider = 0U;

#if LEGACY_INFANTRY
    bool initialized = true;
    for (uint32_t index = 0U; index < 4U; ++index)
    {
        initialized = Chassis_Motor[index].Init(&hfdcan1,
                                               CHASSIS_MOTOR_CAN_ID[index],
                                               CHASSIS_MOTOR_MASTER_ID[index],
                                               Enum_DMMotor_Mode::SPEED,
                                               false,
                                               CHASSIS_MOTOR_POSITION_MAX_RAD,
                                               CHASSIS_MOTOR_VELOCITY_MAX_RAD_S,
                                               CHASSIS_MOTOR_TORQUE_MAX_NM) &&
                      initialized;
    }

    SpeedPlanning_Init(&Chassis_X_Planning, 0.0f);
    SpeedPlanning_Init(&Chassis_Y_Planning, 0.0f);
    SpeedPlanning_Init(&Chassis_W_Planning, 0.0f);
    Chassis_Planned_Velocity_X = 0.0f;
    Chassis_Planned_Velocity_Y = 0.0f;
    Chassis_Planned_Velocity_W = 0.0f;

    Chassis_Initialized = initialized;
    Chassis_Output_Enabled = true;
    if (initialized)
    {
        /* 上电默认失能，与老步兵 SafetyTask 一致：等遥控健康互锁解锁后才输出。 */
        Chassis_SetEnabled(false);
    }
    return initialized;
#elif CHASSIS
    Struct_DJIMotor_Init_Config wheel_config{};
    wheel_config.hfdcan = &hfdcan1;
    wheel_config.motor_type = Enum_DJIMotor_Type::M3508;
    wheel_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    wheel_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    wheel_config.speed_pid = Chassis_MakePID(4.5f, 0.05f, 0.0f, 3000.0f, 16000.0f);

    Struct_DJIMotor_Init_Config steer_config{};
    steer_config.hfdcan = &hfdcan2;
    steer_config.motor_type = Enum_DJIMotor_Type::M3508;
    steer_config.close_loop = DJI_MOTOR_ANGLE_LOOP | DJI_MOTOR_SPEED_LOOP;
    steer_config.outer_loop = DJI_MOTOR_ANGLE_LOOP;
    steer_config.angle_pid = Chassis_MakePID(30.0f, 0.2f, 0.0f, 200.0f, 1000.0f);
    steer_config.speed_pid = Chassis_MakePID(4.0f, 4.0f, 0.0f, 3000.0f, 15000.0f);

    bool initialized = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        wheel_config.can_id = index + 1U;
        steer_config.can_id = index + 1U;
        initialized = Chassis_Wheel_Motor[index].Init(wheel_config) && initialized;
        initialized = Chassis_Steer_Motor[index].Init(steer_config) && initialized;
    }
    initialized = initialized && Chassis_Wheel_Group.Init(
        &Chassis_Wheel_Motor[0], &Chassis_Wheel_Motor[1],
        &Chassis_Wheel_Motor[2], &Chassis_Wheel_Motor[3]);
    initialized = initialized && Chassis_Steer_Group.Init(
        &Chassis_Steer_Motor[0], &Chassis_Steer_Motor[1],
        &Chassis_Steer_Motor[2], &Chassis_Steer_Motor[3]);
    Chassis_Initialized = initialized;
    Chassis_Output_Enabled = true;
    if (initialized)
    {
        Chassis_SetEnabled(false);
    }
    return initialized;
#else
    return true;
#endif
}

void Chassis_Update(void)
{
    /* 每个控制周期都尝试接收新命令；无新数据时保留上一帧目标。 */
    ChassisCmd command;
    if (Chassis_Command_Subscriber.Read(command))
    {
        Chassis_Command = command;
    }

#if LEGACY_INFANTRY
    if (Chassis_Initialized)
    {
        const bool enabled = Chassis_Command.mode != ChassisMode::ZERO_FORCE;
        Chassis_SetEnabled(enabled);

        if (enabled)
        {
            /* 速度规划：三轴各自按非对称速率限制平滑，反向时先刹停再反向加速。 */
            Chassis_Planned_Velocity_X = SpeedPlanning_UpdateRateLimited(
                Chassis_Command.velocity_x_m_s, &Chassis_X_Planning, CHASSIS_CONTROL_DT,
                CHASSIS_X_ACCEL_LIMIT, CHASSIS_X_DECEL_LIMIT,
                CHASSIS_X_RELEASE_LIMIT, CHASSIS_X_REVERSE_LIMIT,
                CHASSIS_PLANNING_THRESHOLD);
            Chassis_Planned_Velocity_Y = SpeedPlanning_UpdateRateLimited(
                Chassis_Command.velocity_y_m_s, &Chassis_Y_Planning, CHASSIS_CONTROL_DT,
                CHASSIS_Y_ACCEL_LIMIT, CHASSIS_Y_DECEL_LIMIT,
                CHASSIS_Y_RELEASE_LIMIT, CHASSIS_Y_REVERSE_LIMIT,
                CHASSIS_PLANNING_THRESHOLD);
            Chassis_Planned_Velocity_W = SpeedPlanning_UpdateRateLimited(
                Chassis_Command.angular_velocity_rad_s, &Chassis_W_Planning, CHASSIS_CONTROL_DT,
                CHASSIS_W_ACCEL_LIMIT, CHASSIS_W_DECEL_LIMIT,
                CHASSIS_W_RELEASE_LIMIT, CHASSIS_W_REVERSE_LIMIT,
                CHASSIS_PLANNING_THRESHOLD);

            Chassis_ControlMotors(Chassis_Planned_Velocity_X,
                                  Chassis_Planned_Velocity_Y,
                                  Chassis_Planned_Velocity_W);
        }

        /* 底盘不测量实际速度，反馈字段表示本周期下发的规划目标与设备在线状态。 */
        bool online = true;
        for (uint32_t index = 0U; index < 4U; ++index)
        {
            online = online && Chassis_Motor[index].IsOnline();
        }
        Chassis_Feedback.velocity_x_m_s = Chassis_Planned_Velocity_X;
        Chassis_Feedback.velocity_y_m_s = Chassis_Planned_Velocity_Y;
        Chassis_Feedback.angular_velocity_rad_s = Chassis_Planned_Velocity_W;
        Chassis_Feedback.enabled = Chassis_Output_Enabled;
        Chassis_Feedback.online = online;
    }
#elif CHASSIS
    if (Chassis_Initialized)
    {
        const bool enabled = Chassis_Command.mode != ChassisMode::ZERO_FORCE;
        Chassis_SetEnabled(enabled);
        if (enabled)
        {
            float wheel_target[4];
            float steer_target[4];
            Chassis_CalculateTargets(wheel_target, steer_target);
            Chassis_Wheel_Group.Control(wheel_target[0], wheel_target[1],
                                        wheel_target[2], wheel_target[3]);
            Chassis_Steer_Group.Control(steer_target[0], steer_target[1],
                                        steer_target[2], steer_target[3]);
        }
        Chassis_UpdateFeedback();
    }
#endif

    /* 电机控制按 1 kHz 执行，反馈消息按 100 Hz 发布。 */
    Chassis_Feedback_Divider++;
    if (Chassis_Feedback_Divider >= 10U)
    {
        Chassis_Feedback_Divider = 0U;
        Chassis_Feedback_Publisher.Publish(Chassis_Feedback);
    }
}

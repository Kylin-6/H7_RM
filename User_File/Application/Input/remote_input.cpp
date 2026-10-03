/**
 * @file remote_input.cpp
 * @brief Remote 输入源适配：S.BUS 硬件输入（底盘板 / 单板）与板间 0x065 转发（云台板）。
 * @details
 * 两种输入硬件由编译开关区分，都只产生 Remote 来源的 ControlInput：
 *
 * - `LEGACY_INFANTRY_CHASSIS`：老步兵底盘板。UART5 S.BUS → 健康互锁 → 摇杆整形与
 *   速度档映射 → 云台跟随/坐标旋转 → 提交底盘与 Yaw 目标；同时把遥控关键通道经
 *   FDCAN2 的 0x065 转发给云台板，并刷新 0x070/0x075 下行帧。
 * - 默认：单板安全模板。UART5 S.BUS → 健康互锁 → 仅供调试的底盘目标。
 *
 * 两条路径都在 ControlTask 上下文调用 InputState_SubmitRemote()；UART/CAN 中断只缓存
 * 原始数据，不在中断上下文写 InputState。链路失效时提交空输入，由 SourceArbitration
 * 输出 safe state。
 */

#include "remote_input.h"

#include "input_state.h"

#if LEGACY_INFANTRY_CHASSIS

#include "Chassis_Config.h"
#include "RobotCmd.h"
#include "alg_basic.h"
#include "board_config.h"
#include "gimbal_board.h"
#include "sbus.h"
#include "usart.h"

#include <cmath>

namespace
{
/* ============================== 遥控通道约定 ============================== */
/* 索引对应 SBUS 原始通道（解析后已减中位 1024），与老工程 sbus_channel_bsp.c 一致。 */

/** 开关 1：> 0 时允许底盘跟随云台。 */
constexpr unsigned kChannelFollowSwitch = 4U;
/** 前后方向摇杆。 */
constexpr unsigned kChannelTranslateX = 1U;
/** 左右方向摇杆。 */
constexpr unsigned kChannelTranslateY = 0U;
/** 旋转摇杆：>= 0 时交给跟随逻辑，否则为手动旋转。 */
constexpr unsigned kChannelRotation = 9U;
/** 三档速度限幅通道。 */
constexpr unsigned kChannelSpeedGear = 6U;
/** 云台偏航摇杆。 */
constexpr unsigned kChannelYaw = 3U;
/** 火控（发射）开关，经 0x065 转发。 */
constexpr unsigned kChannelFireSwitch = 5U;
/** 发射速度（波轮），经 0x065 转发。 */
constexpr unsigned kChannelShootSpeed = 8U;
/** 云台俯仰轴，经 0x065 转发。 */
constexpr unsigned kChannelPitch = 2U;

/* ============================== 输入整形参数 ============================== */
/* 死区与指数曲线沿用老工程 bsp_def.h；实遥控满量程幅值来自 SBUS 驱动约定。 */

constexpr float kChannelMax = 784.0f;
constexpr float kTranslateXDeadband = 0.04f;
constexpr float kTranslateYDeadband = 0.04f;
constexpr float kRotationDeadband = 0.04f;
constexpr float kTranslateXExpo = 0.35f;
constexpr float kTranslateYExpo = 0.35f;
constexpr float kRotationExpo = 0.30f;
constexpr float kYawDeadband = 0.03f;
constexpr float kYawExpo = 0.55f;
/** 跟随开关判决门限。 */
constexpr int16_t kFollowSwitchThreshold = 0;
/** 火控开关极性：实车开关方向与云台板约定相反，转发前取反。 */
constexpr bool kFireSwitchInvert = true;

/* ============================== 链路时序 ============================== */

/** 解锁去抖：连续健康该时长才重新允许输出。 */
constexpr uint32_t kRecoveryMs = 200U;
/** 健康帧新鲜度门限。 */
constexpr uint32_t kFrameFreshMs = 50U;
/** 板间下行帧分频：ControlTask 为 1 kHz，2 对应 2 ms，与老工程一致。 */
constexpr uint8_t kBoardDivider = 2U;

/* ============================== 内部状态 ============================== */

Class_GimbalBoard gimbal_board;
bool initialized;
/** 上电默认锁定；连续健康 200 ms 才解锁，失联立即锁定。 */
bool armed;
bool ever_healthy;
uint32_t last_healthy_ms;
uint32_t last_unhealthy_ms;
uint8_t board_divider;

/** 摇杆整形：按满量程归一化，再做死区与三次指数曲线。 */
float ShapeStick(int16_t channel, float deadband, float expo)
{
    const float normalized =
        Basic_Math_Constrain(static_cast<float>(channel) / kChannelMax, -1.0f, 1.0f);
    const float magnitude = std::fabs(normalized);

    if (magnitude <= deadband)
    {
        return 0.0f;
    }
    const float shaped = (magnitude - deadband) / (1.0f - deadband);
    const float curved = (1.0f - expo) * shaped + expo * shaped * shaped * shaped;
    return normalized < 0.0f ? -curved : curved;
}

/** 三档速度通道 [-784, 784] 线性映射为 [0, maximum]。 */
float MapSpeedGear(int16_t gear_channel, float maximum)
{
    const float limit = (static_cast<float>(gear_channel) + kChannelMax) * maximum /
                        (2.0f * kChannelMax);
    return Basic_Math_Constrain(limit, 0.0f, maximum);
}

/** 云台相对底盘正前方的偏差，回绕到 [-π, π]。 */
float GimbalForwardError(float yaw_rad)
{
    return Basic_Math_Modulus_Normalization(
        yaw_rad - kLegacyChassisConfig.follow_forward_rad, 2.0f * kPiRad);
}

/** 把操作者坐标系下的平移速度旋转到底盘坐标系。 */
void RotateVelocityByGimbal(float angle_rad, float *velocity_x, float *velocity_y)
{
    const float input_x = *velocity_x;
    const float input_y = *velocity_y;
    const float cosine = std::cos(angle_rad);
    const float sine = std::sin(angle_rad);

    *velocity_x = input_x * cosine - input_y * sine;
    *velocity_y = input_x * sine + input_y * cosine;
}

/**
 * @brief 刷新底盘板到云台板的三个下行帧。
 * @param frame 健康且已解锁时的 SBUS 帧；链路失效或未解锁时传 nullptr，转发零通道，
 *              避免云台板继续使用最后一帧旧摇杆值。
 * @note 0x070 的地面系 Yaw 目前没有独立陀螺仪来源，与老工程一致暂用 Yaw 电机角度；
 *       0x075 的裁判数据未接入，按老工程填 0。
 */
void ForwardBoardFrames(const Struct_SBUS_Frame *frame)
{
    int16_t fire_switch = 0;
    int16_t shoot_speed = 0;
    int16_t pitch = 0;
    if (frame != nullptr)
    {
        fire_switch = frame->channels[kChannelFireSwitch];
        if (kFireSwitchInvert)
        {
            fire_switch = static_cast<int16_t>(-fire_switch);
        }
        shoot_speed = frame->channels[kChannelShootSpeed];
        pitch = frame->channels[kChannelPitch];
    }
    (void)gimbal_board.SendRemoteChannels(fire_switch, shoot_speed, pitch);

    GimbalFeedback yaw_feedback{};
    const float yaw_rad = RobotCmd_GetGimbalFeedback(yaw_feedback) && yaw_feedback.enabled
                              ? yaw_feedback.yaw_rad
                              : 0.0f;
    (void)gimbal_board.SendChassisYaw(yaw_rad, yaw_rad);

    (void)gimbal_board.SendRobotStatus(0U, 0U, 0U);
}
} // namespace

bool RemoteInput_Init(void)
{
    initialized = false;
    armed = false;
    ever_healthy = false;
    last_healthy_ms = HAL_GetTick();
    last_unhealthy_ms = last_healthy_ms;
    board_divider = 0U;

    /* 输入仲裁状态一并复位：上电即处于 Remote 失联安全态。 */
    InputState_Reset();
    InputState_SetTime(last_healthy_ms);

    /* S.BUS 固定 UART5；板间下行帧走 BoardConfig 指定的板间链路总线（FDCAN2）。 */
    const bool sbus_ready = SBUS_Init(&huart5);
    const bool board_ready = gimbal_board.Init(BoardConfig_Get().remote_forward_bus);
    initialized = sbus_ready && board_ready;
    return initialized;
}

void RemoteInput_Update(void)
{
    if (!initialized)
    {
        return;
    }

    Struct_SBUS_Frame frame{};
    const uint32_t now = HAL_GetTick();
    InputState_SetTime(now);

    const bool healthy = SBUS_ReadLatest(&frame) &&
                         now - frame.timestamp_ms <= kFrameFreshMs &&
                         !frame.frame_lost && !frame.failsafe;

    /* 维护武装互锁：连续健康 200 ms 才解锁，坏帧或失联立即锁定。 */
    if (healthy)
    {
        last_healthy_ms = now;
        ever_healthy = true;
    }
    else
    {
        last_unhealthy_ms = now;
        if (armed)
        {
            armed = false;
        }
    }
    if (!armed && ever_healthy && now - last_unhealthy_ms >= kRecoveryMs &&
        now - last_healthy_ms <= kFrameFreshMs)
    {
        armed = true;
    }

    /* 板间下行帧按 2 ms 刷新，与老工程 GimbalTask 周期一致，避免压满 FDCAN2。 */
    board_divider++;
    if (board_divider >= kBoardDivider)
    {
        board_divider = 0U;
        ForwardBoardFrames(healthy && armed ? &frame : nullptr);
    }

    if (!armed)
    {
        /* 未解锁：提交空输入，由 SourceArbitration 输出 safe state。 */
        InputState_SubmitRemote({});
        return;
    }

    /* 三轴速度上限由档位通道缩放；量纲仍是老步兵实车抽象速度，见 Chassis_Config.h。 */
    const float speed_limit_x = MapSpeedGear(frame.channels[kChannelSpeedGear],
                                             kLegacyChassisConfig.velocity_x_max);
    const float speed_limit_y = MapSpeedGear(frame.channels[kChannelSpeedGear],
                                             kLegacyChassisConfig.velocity_y_max);
    const float rotation_limit = MapSpeedGear(frame.channels[kChannelSpeedGear],
                                              kLegacyChassisConfig.angular_velocity_max);

    float velocity_x =
        ShapeStick(frame.channels[kChannelTranslateX], kTranslateXDeadband,
                   kTranslateXExpo) *
        speed_limit_x;
    float velocity_y =
        -ShapeStick(frame.channels[kChannelTranslateY], kTranslateYDeadband,
                    kTranslateYExpo) *
        speed_limit_y;
    float velocity_w =
        ShapeStick(frame.channels[kChannelRotation], kRotationDeadband,
                   kRotationExpo) *
        rotation_limit;

    /* Yaw 轴是真实 rad/s 语义，直接提交；符号与老工程一致（摇杆正方向对应负输出）。 */
    GimbalCmd gimbal_command{};
    gimbal_command.mode = GimbalMode::IMU;
    gimbal_command.yaw_speed_rad_s =
        -ShapeStick(frame.channels[kChannelYaw], kYawDeadband, kYawExpo) *
        kLegacyChassisConfig.yaw_speed_max_rad_s;

    /*
     * 平移方向始终相对云台：用 Yaw 轴反馈把操作者坐标系速度旋转到底盘坐标系。
     * 反馈不可用时不旋转：此时角度按 0 处理会算出接近 180° 的偏差，把速度方向
     * 整个翻转（老工程已修过的实车故障），因此宁可不旋转。
     */
    GimbalFeedback yaw_feedback{};
    const bool yaw_feedback_valid =
        RobotCmd_GetGimbalFeedback(yaw_feedback) && yaw_feedback.enabled;

    float forward_error = 0.0f;
    if (yaw_feedback_valid)
    {
        forward_error = GimbalForwardError(yaw_feedback.yaw_rad);
        RotateVelocityByGimbal(forward_error, &velocity_x, &velocity_y);
    }

    /*
     * 旋转摇杆非负时放弃手动旋转：开关 1 抬起交由底盘跟随云台，否则原地不转；
     * 摇杆回拉（< 0）时保留上面的手动旋转值。
     */
    ChassisCmd chassis_command{};
    if (frame.channels[kChannelRotation] >= 0)
    {
        if (frame.channels[kChannelFollowSwitch] > kFollowSwitchThreshold)
        {
            velocity_w = yaw_feedback_valid
                             ? Basic_Math_Constrain(
                                   forward_error * kLegacyChassisConfig.follow_kp,
                                   -rotation_limit, rotation_limit)
                             : 0.0f;
            chassis_command.mode = ChassisMode::FOLLOW_GIMBAL_YAW;
        }
        else
        {
            velocity_w = 0.0f;
            chassis_command.mode = ChassisMode::NO_FOLLOW;
        }
    }
    else
    {
        chassis_command.mode = ChassisMode::NO_FOLLOW;
    }

    /*
     * 边界量纲转换：老步兵抽象速度 → 框架 SI 仲裁边界，同一份比例定义在
     * Chassis_Config.h，Chassis 侧用对应的 *_FromSi 还原。上限再夹一次，
     * 避免浮点误差让 SourceArbitration 的边界校验判为非法而整体失去许可。
     */
    chassis_command.velocity_x_m_s =
        Basic_Math_Constrain(LegacyChassis_TranslateX_ToSi(velocity_x),
                             -INPUT_MAX_TRANSLATION_M_S, INPUT_MAX_TRANSLATION_M_S);
    chassis_command.velocity_y_m_s =
        Basic_Math_Constrain(LegacyChassis_TranslateY_ToSi(velocity_y),
                             -INPUT_MAX_TRANSLATION_M_S, INPUT_MAX_TRANSLATION_M_S);
    chassis_command.angular_velocity_rad_s =
        Basic_Math_Constrain(LegacyChassis_Rotation_ToSi(velocity_w),
                             -INPUT_MAX_ROTATION_RAD_S, INPUT_MAX_ROTATION_RAD_S);

    ControlInput remote{};
    remote.chassis = chassis_command;
    remote.gimbal = gimbal_command;
    /* 发射命令由 0x065 原始通道转发，本板没有 Shoot 执行器，不提交 ShootCmd。 */
    remote.received_ms = frame.timestamp_ms;
    remote.valid = true;
    InputState_SubmitRemote(remote);
}

#else /* 单板安全模板 */

#include "sbus.h"
#include "usart.h"

namespace
{
/* 步兵测试分支的零基通道索引；CH5 跟随须有底盘朝向反馈后再接入。 */
constexpr unsigned TRANSLATE_X = 1U; // CH2
constexpr unsigned TRANSLATE_Y = 0U; // CH1
constexpr unsigned SPEED_GEAR = 6U;  // CH7
constexpr unsigned ROTATION = 9U;    // CH10，负半轴为手动旋转
constexpr float CHANNEL_RANGE = 784.0f;
constexpr int16_t NEUTRAL_THRESHOLD = 50;
constexpr uint32_t FRAME_FRESH_MS = 50U;
constexpr uint32_t RECOVERY_MS = 200U;

bool receiver_ready;
bool armed;
uint32_t last_unhealthy_ms;

float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float Axis(int16_t channel)
{
    // 原始通道先限幅到 [-1, 1]，再设置中心死区；物理速度量程由调用处换算。
    const float normalized = Clamp(static_cast<float>(channel) / CHANNEL_RANGE,
                                   -1.0f, 1.0f);
    return normalized > -0.04f && normalized < 0.04f ? 0.0f : normalized;
}

bool Neutral(const Struct_SBUS_Frame &frame)
{
    /* 解锁必须先松开平移和云台两轴；CH10 在旧遥控上是偏置开关。 */
    constexpr unsigned axes[] = {0U, 1U, 2U, 3U};
    for (unsigned index : axes)
    {
        if (frame.channels[index] < -NEUTRAL_THRESHOLD ||
            frame.channels[index] > NEUTRAL_THRESHOLD)
        {
            return false;
        }
    }
    return true;
}
}

bool RemoteInput_Init(void)
{
    armed = false;
    last_unhealthy_ms = HAL_GetTick();
    InputState_Reset();
    InputState_SetTime(last_unhealthy_ms);
    // S.BUS Device 负责协议解析，并通过 BSP UART 接收；应用只读取完整帧快照。
    receiver_ready = SBUS_Init(&huart5);
    return receiver_ready;
}

void RemoteInput_Update(void)
{
    Struct_SBUS_Frame frame{};
    const uint32_t now = HAL_GetTick();
    InputState_SetTime(now);
    const bool healthy = receiver_ready && SBUS_ReadLatest(&frame) &&
                         now - frame.timestamp_ms <= FRAME_FRESH_MS &&
                         !frame.frame_lost && !frame.failsafe;
    if (!healthy)
    {
        last_unhealthy_ms = now;
        armed = false;
        InputState_SubmitRemote({});
        return;
    }

    if (!armed)
    {
        // 健康帧且摇杆连续回中 200 ms 才解锁；失联或未回中都会重新计时。
        if (!Neutral(frame))
        {
            last_unhealthy_ms = now;
            InputState_SubmitRemote({});
            return;
        }
        if (now - last_unhealthy_ms < RECOVERY_MS)
        {
            InputState_SubmitRemote({});
            return;
        }
        armed = true;
    }

    // 速度档映射到 [0, 1]，只缩放 SI 速度目标；此处尚未接入云台和发射通道。
    const float gear = Clamp((static_cast<float>(frame.channels[SPEED_GEAR]) +
                              CHANNEL_RANGE) / (2.0f * CHANNEL_RANGE), 0.0f, 1.0f);
    ChassisCmd chassis{};
    chassis.velocity_x_m_s = Axis(frame.channels[TRANSLATE_X]) * gear * INPUT_MAX_TRANSLATION_M_S;
    chassis.velocity_y_m_s = -Axis(frame.channels[TRANSLATE_Y]) * gear * INPUT_MAX_TRANSLATION_M_S;
    if (frame.channels[ROTATION] < 0)
    {
        chassis.angular_velocity_rad_s = Axis(frame.channels[ROTATION]) * gear *
                                         INPUT_MAX_ROTATION_RAD_S;
    }
    if (chassis.velocity_x_m_s != 0.0f || chassis.velocity_y_m_s != 0.0f ||
        chassis.angular_velocity_rad_s != 0.0f)
    {
        chassis.mode = ChassisMode::NO_FOLLOW;
    }
    ControlInput remote{};
    remote.chassis = chassis;
    remote.received_ms = frame.timestamp_ms;
    remote.valid = true;
    InputState_SubmitRemote(remote);
}

#endif /* LEGACY_INFANTRY_CHASSIS */

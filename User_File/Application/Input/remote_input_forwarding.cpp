/**
 * @file remote_input_forwarding.cpp
 * @brief Remote 输入源适配：老步兵底盘板的 S.BUS 输入 + 板间下行帧转发。
 * @details
 * UART5 S.BUS → 健康互锁 → 摇杆整形与速度档映射 → 云台跟随/坐标旋转 →
 * 提交底盘与 Yaw 目标；同时把遥控关键通道经 FDCAN2 的 0x065 转发给云台板，
 * 并刷新 0x070/0x075 下行帧。只产生 Remote 来源的 ControlInput；UART/CAN
 * 中断只缓存原始数据，不在中断上下文写 InputState。链路失效时提交空输入，
 * 由 SourceArbitration 输出 safe state。
 * 本文件仅编入 ChassisBoard（构建期源码选择）；单板安全模板的纯 S.BUS
 * 调试输入见 remote_input.cpp。
 */

#include "remote_input.h"

#include "input_state.h"


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

/** CH5 手动保护开关：< 0 时按遥控失联锁定输入。 */
constexpr unsigned kChannelSafetySwitch = 4U;
/** CH8 跟随开关：> 0 时允许底盘跟随云台。 */
constexpr unsigned kChannelFollowSwitch = 7U;
/** 前后方向摇杆。 */
constexpr unsigned kChannelTranslateX = 1U;
/** 左右方向摇杆。 */
constexpr unsigned kChannelTranslateY = 0U;
/** 旋转摇杆：>= +350 时交给跟随逻辑，否则为手动旋转。 */
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
constexpr int16_t kRotationTrigger = 350;
constexpr float kTranslateXExpo = 0.35f;
constexpr float kTranslateYExpo = 0.35f;
constexpr float kRotationExpo = 0.30f;
constexpr float kYawDeadband = 0.03f;
constexpr float kYawExpo = 0.30f;
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
bool manual_protection;
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

/** CH7 三挡速度：负位 30%、中位 60%、正位 100%，分界为 ±392。 */
float MapSpeedGear(int16_t gear_channel, float maximum)
{
    const float ratio = gear_channel < -392 ? 0.30f :
                        gear_channel > 392 ? 1.0f : 0.60f;
    return maximum * ratio;
}

/** 云台相对底盘正前方的偏差，回绕到 [-π, π]。 */
float GimbalForwardError(float yaw_rad)
{
    return Basic_Math_Modulus_Normalization(
        yaw_rad - kInfantryChassisConfig.follow_forward_rad, 2.0f * kPiRad);
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
    manual_protection = false;
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

    const bool frame_healthy = SBUS_ReadLatest(&frame) &&
                               now - frame.timestamp_ms <= kFrameFreshMs &&
                               !frame.frame_lost && !frame.failsafe;
    manual_protection = frame_healthy && frame.channels[kChannelSafetySwitch] < 0;
    const bool healthy = frame_healthy && !manual_protection;

    /* 维护武装互锁：连续健康 200 ms 才解锁，坏帧、失联或 CH5 保护立即锁定。 */
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
                                             kInfantryChassisConfig.velocity_x_max);
    const float speed_limit_y = MapSpeedGear(frame.channels[kChannelSpeedGear],
                                             kInfantryChassisConfig.velocity_y_max);
    const float rotation_limit = MapSpeedGear(frame.channels[kChannelSpeedGear],
                                              kInfantryChassisConfig.angular_velocity_max);

    float velocity_x =
        ShapeStick(frame.channels[kChannelTranslateX], kTranslateXDeadband,
                   kTranslateXExpo) *
        speed_limit_x;
    float velocity_y =
        -ShapeStick(frame.channels[kChannelTranslateY], kTranslateYDeadband,
                    kTranslateYExpo) *
        speed_limit_y;
    /* +350 为零速边界，向负满杆连续增加自转速度，保持原来的旋转方向。 */
    const float rotation_stick = Basic_Math_Constrain(
        (static_cast<float>(kRotationTrigger) - frame.channels[kChannelRotation]) /
            (kChannelMax + kRotationTrigger),
        0.0f, 1.0f);
    const float manual_rotation =
        -((1.0f - kRotationExpo) * rotation_stick +
          kRotationExpo * rotation_stick * rotation_stick * rotation_stick) * rotation_limit;

    /* Yaw 轴是真实 rad/s 语义，直接提交；符号与老工程一致（摇杆正方向对应负输出）。 */
    GimbalCmd gimbal_command{};
    gimbal_command.mode = GimbalMode::IMU;
    gimbal_command.yaw_speed_rad_s =
        -ShapeStick(frame.channels[kChannelYaw], kYawDeadband, kYawExpo) *
        kInfantryChassisConfig.yaw_speed_max_rad_s;

    /*
     * 平移方向始终相对云台：用 Yaw 轴反馈把操作者坐标系速度旋转到底盘坐标系。
     * 反馈不可用时不旋转：此时角度按 0 处理会算出接近 180° 的偏差，把速度方向
     * 整个翻转（老工程已修过的实车故障），因此宁可不旋转。
     */
    GimbalFeedback yaw_feedback{};
    const bool yaw_feedback_valid =
        RobotCmd_GetGimbalFeedback(yaw_feedback) && yaw_feedback.enabled &&
        std::isfinite(yaw_feedback.yaw_rad);

    float forward_error = 0.0f;
    if (yaw_feedback_valid)
    {
        forward_error = GimbalForwardError(yaw_feedback.yaw_rad);
        RotateVelocityByGimbal(forward_error, &velocity_x, &velocity_y);
    }

    /*
     * 小陀螺优先：进入自转区时不做朝向对齐；退出后由 CH8 允许跟随。
     * 偏差是云台相对底盘正面的角度，实车旋转输出取反使偏差趋近零。
     */
    ChassisCmd chassis_command{};
    float follow_rotation = 0.0f;
    const bool follow_enabled = frame.channels[kChannelRotation] >= kRotationTrigger &&
                                frame.channels[kChannelFollowSwitch] > kFollowSwitchThreshold &&
                                yaw_feedback_valid;
    if (follow_enabled)
    {
        const float follow_limit = std::fmin(rotation_limit,
                                             kInfantryChassisConfig.follow_rotation_max);
        /* 死区外扣除死区宽度，避免跨过边界时修正速度跳变。 */
        const float follow_error = std::copysign(
            std::fmax(std::fabs(forward_error) - kInfantryChassisConfig.follow_deadband_rad,
                      0.0f),
            forward_error);
        follow_rotation = Basic_Math_Constrain(
            -follow_error * kInfantryChassisConfig.follow_kp, -follow_limit, follow_limit);
        chassis_command.mode = ChassisMode::FOLLOW_GIMBAL_YAW;
    }
    else
    {
        chassis_command.mode = ChassisMode::NO_FOLLOW;
    }
    const float velocity_w = Basic_Math_Constrain(manual_rotation + follow_rotation,
                                                 -rotation_limit, rotation_limit);

    /*
     * 边界量纲转换：老步兵抽象速度 → 框架 SI 仲裁边界，同一份比例定义在
     * Chassis_Config.h，Chassis 侧用对应的 *_FromSi 还原。上限再夹一次，
     * 避免浮点误差让 SourceArbitration 的边界校验判为非法而整体失去许可。
     */
    chassis_command.velocity_x_m_s =
        Basic_Math_Constrain(Chassis_TranslateX_ToSi(velocity_x),
                             -INPUT_MAX_TRANSLATION_M_S, INPUT_MAX_TRANSLATION_M_S);
    chassis_command.velocity_y_m_s =
        Basic_Math_Constrain(Chassis_TranslateY_ToSi(velocity_y),
                             -INPUT_MAX_TRANSLATION_M_S, INPUT_MAX_TRANSLATION_M_S);
    chassis_command.angular_velocity_rad_s =
        Basic_Math_Constrain(Chassis_Rotation_ToSi(velocity_w),
                             -INPUT_MAX_ROTATION_RAD_S, INPUT_MAX_ROTATION_RAD_S);

    ControlInput remote{};
    remote.chassis = chassis_command;
    remote.gimbal = gimbal_command;
    /* 发射命令由 0x065 原始通道转发，本板没有 Shoot 执行器，不提交 ShootCmd。 */
    remote.received_ms = frame.timestamp_ms;
    remote.valid = true;
    InputState_SubmitRemote(remote);
}

bool RemoteInput_IsLinkOnline(void)
{
    /* 直接返回 SBUS Device 内部 Daemon 的 liveness 结果，不自行计算超时；
     * 健康互锁（50 ms 帧新鲜度、失控位、200 ms 回中解锁）保持独立。 */
    return initialized && SBUS_IsOnline();
}

bool RemoteInput_IsManualProtection(void)
{
    return initialized && manual_protection;
}

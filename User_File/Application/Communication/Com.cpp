/**
 * @file Com.cpp
 * @brief 老步兵上层输入适配实现：SBUS 遥控 → RobotCmd 目标，并转发云台板链路。
 * @details
 * 移植自 rm/demo 的 APP/TransmitTask.c、APP/SafetyTask.c 与 APP/GimbalTask.c 中的
 * 遥控部分。所有整形常量保持与 demo 的 User/bsp/bsp_def.h 一致。
 */

#include "Com.h"

#include "RobotCmd.h"
#include "SpeedPlanning.h"
#include "bsp_ws2812.h"
#include "fdcan.h"
#include "gimbal_board.h"
#include "sbus.h"

#if LEGACY_INFANTRY

/* ============================ 遥控通道索引 ============================ */

/* 索引对应 SBUS 原始通道（解析后已减中位 1024），与 demo 的 SBUS_ChannelBSP_Update 一致。 */

/** 开关 1：> 0 时允许底盘跟随云台。 */
#define RC_INDEX_FOLLOW_SWITCH (4U)
/** 前后方向摇杆。 */
#define RC_INDEX_TRANSLATE_X (1U)
/** 左右方向摇杆。 */
#define RC_INDEX_TRANSLATE_Y (0U)
/** 旋转摇杆：>= 0 时交给跟随逻辑，否则为手动旋转。 */
#define RC_INDEX_ROTATION (9U)
/** 三档速度限幅通道。 */
#define RC_INDEX_SPEED_GEAR (6U)
/** 云台偏航摇杆。 */
#define RC_INDEX_GIMBAL_YAW (3U)

/* ============================== 控制参数 ============================== */

/** 遥控失联判定门限，毫秒。 */
#define COMMUNICATION_LOSS_TIMEOUT_MS (200U)
/** 遥控恢复需要持续健康的时长，毫秒。 */
#define COMMUNICATION_RECOVERY_TIME_MS (200U)

/** 底盘三轴目标速度上限，沿用老步兵原始速度量纲（非 m/s）。 */
#define CHASSIS_TARGET_SPEED_X_MAX (30.0F)
#define CHASSIS_TARGET_SPEED_Y_MAX (30.0F)
#define CHASSIS_TARGET_ROTATION_MAX (50.0F)
/** 云台偏航摇杆速度上限，rad/s；必须与 Gimbal.cpp 的 GIMBAL_YAW_SPEED_MAX 一致。 */
#define GIMBAL_TARGET_YAW_SPEED_MAX (15.0F)

/** 摇杆死区与指数曲线参数，与 demo 的 bsp_def.h 一致。 */
#define RC_CHASSIS_X_DEADBAND (0.04F)
#define RC_CHASSIS_Y_DEADBAND (0.04F)
#define RC_CHASSIS_W_DEADBAND (0.04F)
#define RC_CHASSIS_X_EXPO (0.35F)
#define RC_CHASSIS_Y_EXPO (0.35F)
#define RC_CHASSIS_W_EXPO (0.30F)
#define RC_GIMBAL_YAW_DEADBAND (0.03F)
#define RC_GIMBAL_YAW_EXPO (0.55F)

/** 云台跟随开关的判决门限。 */
#define RC_FOLLOW_SWITCH_THRESHOLD (0)
/** 跟随云台时底盘角速度的比例增益。 */
#define CHASSIS_GIMBAL_FOLLOW_KP (8.0F)
/** 机械安装中云台偏航为 180° 时对应底盘正前方，rad。 */
#define CHASSIS_GIMBAL_FORWARD_RAD (3.14159265F)
/** 2π，rad。 */
#define CHASSIS_TWO_PI_RAD (6.28318531F)

/** 板间链路下发分频：Control_Task 为 1 kHz，2 对应 2 ms，与老工程的 GimbalTask 周期一致。 */
#define COMMUNICATION_BOARD_DIVIDER (2U)

static Class_GimbalBoard Communication_Gimbal_Board;
static bool Communication_Armed = false;
static uint8_t Communication_Board_Divider;

/** 将三档速度通道 [-784, 784] 线性映射为 [0, maximum_speed]。 */
static float Communication_MapSpeedGear(int16_t gear_channel, float maximum_speed)
{
    const float speed_limit = ((float)gear_channel + SBUS_CHANNEL_MAX) * maximum_speed /
                              (2.0F * SBUS_CHANNEL_MAX);

    if (speed_limit < 0.0F)
    {
        return 0.0F;
    }
    if (speed_limit > maximum_speed)
    {
        return maximum_speed;
    }

    return speed_limit;
}

/**
 * @brief 计算机械安装下云台相对底盘正前方的偏差。
 * @details 机械安装中云台偏航为 180° 时对应底盘正前方；结果回绕到 [-π, π]，
 *          避免跨越 ±π 时产生接近 2π 的突变。
 */
static float Communication_GetGimbalForwardError(float gimbal_yaw_rad)
{
    float forward_error = gimbal_yaw_rad - CHASSIS_GIMBAL_FORWARD_RAD;

    while (forward_error > CHASSIS_GIMBAL_FORWARD_RAD)
    {
        forward_error -= CHASSIS_TWO_PI_RAD;
    }
    while (forward_error < -CHASSIS_GIMBAL_FORWARD_RAD)
    {
        forward_error += CHASSIS_TWO_PI_RAD;
    }

    return forward_error;
}

/** 将遥控给出的底盘坐标系平移速度旋转到当前云台朝向对应的坐标系。 */
static void Communication_RotateVelocityByGimbal(float gimbal_yaw_rad,
                                                 float* velocity_x,
                                                 float* velocity_y)
{
    const float input_velocity_x = *velocity_x;
    const float input_velocity_y = *velocity_y;
    const float cosine = cosf(gimbal_yaw_rad);
    const float sine = sinf(gimbal_yaw_rad);

    *velocity_x = input_velocity_x * cosine - input_velocity_y * sine;
    *velocity_y = input_velocity_x * sine + input_velocity_y * cosine;
}

/** 跟随云台时按偏差生成底盘角速度，并限制在旋转上限内。 */
static float Communication_GetGimbalFollowSpeed(float gimbal_forward_error,
                                                float rotation_limit)
{
    const float follow_speed = gimbal_forward_error * CHASSIS_GIMBAL_FOLLOW_KP;

    if (follow_speed > rotation_limit)
    {
        return rotation_limit;
    }
    if (follow_speed < -rotation_limit)
    {
        return -rotation_limit;
    }

    return follow_speed;
}

/** 武装状态指示灯：红灯表示电机失能，蓝灯表示已解锁，与 demo 的灯色一致。 */
static void Communication_IndicateArmed(bool armed)
{
    if (armed)
    {
        BSP_WS2812.Set_RGB(0x00U, 0x00U, 0xFFU);
    }
    else
    {
        BSP_WS2812.Set_RGB(0xFFU, 0x00U, 0x00U);
    }

    /*
     * 老工程的 BSP_WS2812_Set_RGB 在设置颜色后立即发送一帧，框架的 Set_RGB 只改
     * 颜色缓存、刷新依赖 TIM 的 10 ms 分发。这里补一次即时发送，让状态跳变时的
     * 灯色立刻可见，而不是最多延迟一个分发周期。发送 25 字节约 32 us。
     */
    BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
}

/**
 * @brief 维护遥控健康互锁，并同步武装指示灯。
 * @details 对应 demo 的 SafetyTask：上电默认锁定（所有电机路径保持失能命令），
 *          连续健康 200 ms 才解锁，健康帧超时 200 ms 立即重新锁定。
 */
static void Communication_UpdateArmState(void)
{
    if (!Communication_Armed)
    {
        if (SBUS_IsControlHealthyFor(COMMUNICATION_RECOVERY_TIME_MS))
        {
            Communication_Armed = true;
            Communication_IndicateArmed(true);
        }
    }
    else if (SBUS_IsControlLostFor(COMMUNICATION_LOSS_TIMEOUT_MS))
    {
        Communication_Armed = false;
        Communication_IndicateArmed(false);
    }
}

bool Communication_Init(void)
{
    Communication_Armed = false;
    Communication_Board_Divider = 0U;
    /* 上电默认失能，与 demo 一致先点亮红色指示灯。 */
    Communication_IndicateArmed(false);

    const bool sbus_ready = SBUS_Init();
    /* 老步兵的底盘板固定在 FDCAN2 上向云台板发送状态。 */
    const bool board_ready = Communication_Gimbal_Board.Init(&hfdcan2);

    return sbus_ready && board_ready;
}

void Communication_Update(void)
{
    int16_t channels[SBUS_CHANNEL_COUNT] = {0};
    uint8_t frame_lost = 0U;
    uint8_t failsafe = 0U;

    const bool available = SBUS_GetLatestFrame(channels, &frame_lost, &failsafe);
    (void)frame_lost;
    (void)failsafe;

    Communication_UpdateArmState();

    /* 云台反馈每周期读取：供板间链路与底盘坐标旋转共同使用。 */
    GimbalFeedback gimbal_feedback{};
    const bool gimbal_feedback_valid = RobotCmd_GetGimbalFeedback(gimbal_feedback);

    /* 板间链路按 2 ms 下发，与老工程的 GimbalTask 周期一致，避免压满 CAN 总线。 */
    Communication_Board_Divider++;
    if (Communication_Board_Divider >= COMMUNICATION_BOARD_DIVIDER)
    {
        Communication_Board_Divider = 0U;

        /* 遥控帧只在拿到有效数据时转发；状态帧与解锁与否无关。 */
        if (available)
        {
            Communication_Gimbal_Board.SendRemoteChannels(channels);
        }

        /* 地面系 Yaw 尚无外部陀螺仪来源，与 demo 一致地暂用云台电机角度代替。 */
        const float chassis_yaw_rad = gimbal_feedback_valid ? gimbal_feedback.yaw_rad : 0.0F;
        Communication_Gimbal_Board.SendChassisYaw(chassis_yaw_rad, chassis_yaw_rad);
        /* 裁判系统未接入，热量上限 / 冷却 / 机器人 ID 均为 0。 */
        Communication_Gimbal_Board.SendRobotStatus(0U, 0U, 0U);
    }

    if (!Communication_Armed)
    {
        /* 未解锁：显式下发安全默认，避免上一帧命令残留造成意外动作。 */
        ChassisCmd safe_chassis{};
        GimbalCmd safe_gimbal{};
        RobotCmd_SetChassis(safe_chassis);
        RobotCmd_SetGimbal(safe_gimbal);
        return;
    }

    const float speed_limit_x =
        Communication_MapSpeedGear(channels[RC_INDEX_SPEED_GEAR], CHASSIS_TARGET_SPEED_X_MAX);
    const float speed_limit_y =
        Communication_MapSpeedGear(channels[RC_INDEX_SPEED_GEAR], CHASSIS_TARGET_SPEED_Y_MAX);
    const float rotation_limit =
        Communication_MapSpeedGear(channels[RC_INDEX_SPEED_GEAR], CHASSIS_TARGET_ROTATION_MAX);

    float velocity_x =
        SpeedPlanning_ApplyDeadbandExpo((float)channels[RC_INDEX_TRANSLATE_X], SBUS_CHANNEL_MAX,
                                        RC_CHASSIS_X_DEADBAND, RC_CHASSIS_X_EXPO) *
        speed_limit_x;
    float velocity_y =
        -SpeedPlanning_ApplyDeadbandExpo((float)channels[RC_INDEX_TRANSLATE_Y], SBUS_CHANNEL_MAX,
                                         RC_CHASSIS_Y_DEADBAND, RC_CHASSIS_Y_EXPO) *
        speed_limit_y;
    float velocity_w =
        SpeedPlanning_ApplyDeadbandExpo((float)channels[RC_INDEX_ROTATION], SBUS_CHANNEL_MAX,
                                        RC_CHASSIS_W_DEADBAND, RC_CHASSIS_W_EXPO) *
        rotation_limit;

    /*
     * 平移方向始终相对云台保持一致，便于云台转动时操控。
     * 云台反馈不可用时不做旋转：此时角度按 0 处理会被换算成 -180° 偏差，把速度
     * 方向整个翻转，表现为前后左右全反。
     */
    float gimbal_forward_error = 0.0F;
    if (gimbal_feedback_valid)
    {
        gimbal_forward_error = Communication_GetGimbalForwardError(gimbal_feedback.yaw_rad);
        Communication_RotateVelocityByGimbal(gimbal_forward_error, &velocity_x, &velocity_y);
    }

    /*
     * 旋转摇杆非负时放弃手动旋转：开关 1 抬起交由底盘跟随云台，否则原地静止。
     * 摇杆回拉（< 0）时保留上面的手动旋转值。
     */
    ChassisCmd chassis_command{};
    if (channels[RC_INDEX_ROTATION] >= 0 &&
        channels[RC_INDEX_FOLLOW_SWITCH] > RC_FOLLOW_SWITCH_THRESHOLD)
    {
        velocity_w = gimbal_feedback_valid
                         ? Communication_GetGimbalFollowSpeed(gimbal_forward_error,
                                                              rotation_limit)
                         : 0.0F;
        chassis_command.mode = ChassisMode::FOLLOW_GIMBAL_YAW;
    }
    else if (channels[RC_INDEX_ROTATION] >= 0)
    {
        velocity_w = 0.0F;
        chassis_command.mode = ChassisMode::NO_FOLLOW;
    }
    else
    {
        chassis_command.mode = ChassisMode::NO_FOLLOW;
    }

    chassis_command.velocity_x_m_s = velocity_x;
    chassis_command.velocity_y_m_s = velocity_y;
    chassis_command.angular_velocity_rad_s = velocity_w;

    GimbalCmd gimbal_command{};
    const float yaw_stick = SpeedPlanning_ApplyDeadbandExpo(
        (float)channels[RC_INDEX_GIMBAL_YAW], SBUS_CHANNEL_MAX,
        RC_GIMBAL_YAW_DEADBAND, RC_GIMBAL_YAW_EXPO);
    /* 符号与 demo 一致：摇杆正方向对应偏航负方向输出。 */
    gimbal_command.yaw_speed_rad_s = -yaw_stick * GIMBAL_TARGET_YAW_SPEED_MAX;
    gimbal_command.mode = GimbalMode::IMU;

    RobotCmd_SetChassis(chassis_command);
    RobotCmd_SetGimbal(gimbal_command);
}

#else

bool Communication_Init(void)
{
    return true;
}

void Communication_Update(void)
{
}

#endif /* LEGACY_INFANTRY */

/**
 * @file Com.cpp
 * @brief 通信应用实现：板间输入适配与健康互锁（老步兵云台板配置）。
 * @details
 * 输入适配的数值与云台板原工程逐项一致，只是把「通道 -> 目标」的换算从轴侧
 * （原 Pitch / Shoot 模块）集中到输入侧：
 *
 * - Pitch 通道：两级一阶低通（每级 tau = 25 ms，总延迟约 50 ms）+ 线性映射到
 *   DM-IMU 限位 [-40 度, +15 度]；
 * - 火控开关：使用 [-500, +500] 双阈值滞回，避免必须拨到通道端点；
 * - 波轮档位：[-780, 740] 线性映射到 [0, 拨弹盘输出最大速度]。
 *
 * 云台与发射的目标都经 `RobotCmd` 发布，多个上层输入同时存在时仍由 RobotCmd
 * 统一仲裁；链路失效时这里只负责下发安全命令。
 */

#include "Com.h"

#include "message_center.h"

#include "alg_filter_iir.h"

#include <cstdint>
#include <string.h>

void Communication_Callback(uint8_t* Buffer, uint16_t Length)
{
    (void)Buffer;
    (void)Length;
}

#if LEGACY_INFANTRY_GIMBAL

#include "Pitch.h"
#include "chassis_board.h"
#include "fdcan.h"

namespace
{
/* Pitch 通道 -> DM-IMU 目标角（数值取自云台板原 Pitch 模块）。 */
constexpr float kPitchChannelMin = -770.0f;
constexpr float kPitchChannelSpan = 1520.0f;
constexpr float kPitchChannelFilterTauS = 0.025f;
/** 由时间常数换算的一阶低通截止频率：fc = 1 / (2*pi*tau)，Hz。 */
constexpr float kPitchChannelFilterCutoffHz =
    1.0f / (6.283185307179586f * kPitchChannelFilterTauS);

/* 火控开关双阈值：端点约为 +/-780，中间区保持上次状态以抑制抖动。 */
constexpr int16_t kFirePressedThreshold = -500;
constexpr int16_t kFireReleasedThreshold = 500;

/* 波轮档位 -> 拨弹盘输出速度（M2006，减速比 36，转子上限 4500 rpm）。 */
constexpr int16_t kDialMin = -780;
constexpr int16_t kDialMax = 740;
constexpr float kLoaderMaxRotorRpm = 4500.0f;
constexpr float kM2006GearRatio = 36.0f;
constexpr float kShootTwoPi = 6.283185307179586f;
constexpr float kLoaderMaxOutputRadS =
    kLoaderMaxRotorRpm * kShootTwoPi / 60.0f / kM2006GearRatio;

Class_ChassisBoard chassis_board;
bool communication_initialized;
bool fire_trigger_pressed;
/* Pitch 通道两级一阶低通（框架 Class_Filter_IIR_First_Order 级联），
 * 每级时间常数 25 ms，总延迟约 50 ms，与原手写实现一致。 */
Class_Filter_IIR_First_Order pitch_filter_stage1;
Class_Filter_IIR_First_Order pitch_filter_stage2;
/* 云台 / 发射命令直发 Topic：Topic 为 Latest-Value 语义，重复值发布幂等，
 * 不再经 RobotCmd 全局 setter 中转，也无须逐字段 change-detection。 */
Publisher<GimbalCmd> Gimbal_Command_Publisher(MessageCenter::Gimbal_Command_Topic);
Publisher<ShootCmd> Shoot_Command_Publisher(MessageCenter::Shoot_Command_Topic);

/** 通道值线性映射到 DM-IMU Pitch 限位。 */
float MapPitchChannel(float channel)
{
    const float ratio = 1.0f - (channel - kPitchChannelMin) / kPitchChannelSpan;
    return PITCH_TARGET_MIN_RAD + ratio * (PITCH_TARGET_MAX_RAD - PITCH_TARGET_MIN_RAD);
}

/** 波轮档位线性映射到拨弹盘输出速度，负档位为 0。 */
float MapDialToLoaderSpeed(int16_t dial)
{
    if (dial < kDialMin)
    {
        dial = kDialMin;
    }
    else if (dial > kDialMax)
    {
        dial = kDialMax;
    }
    return static_cast<float>(dial - kDialMin) * kLoaderMaxOutputRadS /
           static_cast<float>(kDialMax - kDialMin);
}

/** 发布一次云台与发射命令。 */
void PublishCommands(const GimbalCmd &gimbal_command, const ShootCmd &shoot_command)
{
    Gimbal_Command_Publisher.Publish(gimbal_command);
    Shoot_Command_Publisher.Publish(shoot_command);
}
} // namespace

void Communication_Init(void)
{
    if (communication_initialized)
    {
        return;
    }

    /* 板间链路走云台板的 FDCAN2，与底盘板的下行帧一致。 */
    chassis_board.Init(&hfdcan2);
    fire_trigger_pressed = false;
    /* 两级低通：每级 tau = 25 ms（原工程数值），1 kHz 采样。 */
    pitch_filter_stage1.Init(kPitchChannelFilterCutoffHz, 1000.0f);
    pitch_filter_stage2.Init(kPitchChannelFilterCutoffHz, 1000.0f);
    communication_initialized = true;
}

/**
 * @brief 读取板间通道并发布云台 / 发射命令。
 *
 * 链路健康：Pitch 通道经滤波映射后发布 `GimbalMode::IMU` 目标，火控开关与波轮
 * 档位发布到 `ShootCmd` 的扳机使能与连发速度。链路失效（100 ms 无新帧）：
 * 发布 `GimbalMode::DISABLED` 与关闭的 `ShootCmd`，两个 Application 同周期停手。
 */
void Communication_Update(void)
{
    if (!communication_initialized)
    {
        return;
    }

    int16_t fire = 0;
    int16_t dial = 0;
    int16_t pitch = 0;
    const bool channels_valid = chassis_board.GetFire(&fire) &&
                                chassis_board.GetDial(&dial) &&
                                chassis_board.GetPitch(&pitch);

    if (!channels_valid)
    {
        /* 安全互锁：不再清除滤波历史，链路恢复后目标由限速率路径平滑过渡。 */
        fire_trigger_pressed = false;
        GimbalCmd gimbal_command{};
        gimbal_command.mode = GimbalMode::DISABLED;
        ShootCmd shoot_command{};
        PublishCommands(gimbal_command, shoot_command);
        return;
    }

    if (fire <= kFirePressedThreshold)
    {
        fire_trigger_pressed = true;
    }
    else if (fire >= kFireReleasedThreshold)
    {
        fire_trigger_pressed = false;
    }
    const bool trigger_pressed = fire_trigger_pressed;

    GimbalCmd gimbal_command{};
    gimbal_command.mode = GimbalMode::IMU;
    /* 云台板没有 Yaw 目标输入：Yaw 锁在使能时刻的姿态，目标角由 Gimbal 保持。 */
    gimbal_command.yaw_angle_rad = 0.0f;
    gimbal_command.yaw_speed_rad_s = 0.0f;
    /* 两级级联低通：首帧由 Set_Now 自动对齐通道值（与原实现一致）；
     * 链路失效期间不喂数据、不清历史，恢复后经限速率路径平滑过渡。 */
    pitch_filter_stage1.Set_Now(static_cast<float>(pitch));
    pitch_filter_stage1.TIM_Calculate_PeriodElapsedCallback();
    pitch_filter_stage2.Set_Now(pitch_filter_stage1.Get_Out());
    pitch_filter_stage2.TIM_Calculate_PeriodElapsedCallback();
    gimbal_command.pitch_angle_rad =
        MapPitchChannel(pitch_filter_stage2.Get_Out());
    gimbal_command.pitch_speed_rad_s = 0.0f;

    ShootCmd shoot_command{};
    shoot_command.shoot_mode = trigger_pressed ? ShootMode::ON : ShootMode::OFF;
    /* 摩擦轮由发射状态机自己驱动，这两个字段只作为上层可读的意图描述。 */
    shoot_command.friction_mode =
        trigger_pressed ? FrictionMode::ON : FrictionMode::OFF;
    shoot_command.loader_mode = trigger_pressed ? LoaderMode::BURST : LoaderMode::STOP;
    shoot_command.loader_speed_deg_s = MapDialToLoaderSpeed(dial);

    PublishCommands(gimbal_command, shoot_command);
}

bool Communication_GetRawChannels(int16_t *fire, int16_t *dial, int16_t *pitch)
{
    if (!communication_initialized)
    {
        return false;
    }

    int16_t local_fire = 0;
    int16_t local_dial = 0;
    int16_t local_pitch = 0;
    const bool valid = chassis_board.GetFire(&local_fire) &&
                       chassis_board.GetDial(&local_dial) &&
                       chassis_board.GetPitch(&local_pitch);

    if (fire != nullptr)
    {
        *fire = local_fire;
    }
    if (dial != nullptr)
    {
        *dial = local_dial;
    }
    if (pitch != nullptr)
    {
        *pitch = local_pitch;
    }
    return valid;
}

#else

void Communication_Init(void)
{
}

void Communication_Update(void)
{
}

bool Communication_GetRawChannels(int16_t *fire, int16_t *dial, int16_t *pitch)
{
    if (fire != nullptr)
    {
        *fire = 0;
    }
    if (dial != nullptr)
    {
        *dial = 0;
    }
    if (pitch != nullptr)
    {
        *pitch = 0;
    }
    return false;
}

#endif /* LEGACY_INFANTRY_GIMBAL */

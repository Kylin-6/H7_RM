/**
 * @file remote_input_forwarded.cpp
 * @brief Remote 输入源适配：底盘板 0x065 板间转发（老步兵云台板）。
 * @details
 * 底盘板 0x065 → Class_ChassisBoard 缓存 → 本层整形（Pitch 两级低通 /
 * 火控滞回 / 波轮档位映射）→ InputState_SubmitRemote()。
 * 只产生 Remote 来源的 ControlInput；CAN 中断只缓存原始数据，不在中断
 * 上下文写 InputState。链路失效时提交空输入，由 SourceArbitration 输出
 * safe state。本文件仅编入 GimbalBoard（构建期源码选择）。
 */

#include "remote_input.h"

#include "input_state.h"

#include "Gimbal_Config.h"
#include "board_config.h"
#include "chassis_board.h"
#include "fdcan.h"

#include "alg_filter_iir.h"

#include <string.h>
#include <cmath>

// EmberProbe 只读观测，ControlTask 唯一写入者；无效时通道值不用于控制。
int16_t Remote_Pitch_Channel = 0;
bool Remote_Pitch_Valid = false;

namespace
{
/* Pitch 通道 -> DM-IMU 目标角（数值取自云台板原 Pitch 模块）。 */
constexpr float kPitchChannelMin = -770.0f;
constexpr float kPitchChannelSpan = 1520.0f;
constexpr float kPitchChannelFilterTauS = 0.025f;
/* 老工程使用后向欧拉系数 dt/(tau+dt)；框架 IIR 使用 1-exp(-2πfc/fs)。
 * Init 时反解 fc，复用组件并保持原递推系数，不能直接套 fc=1/(2πtau)。 */
constexpr float kControlPeriodS = 0.001f;

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
bool remote_input_initialized;
bool fire_trigger_pressed;
/** 链路建立/恢复后须先见到一次松开位，才允许重新锁存按下；避免插回链路时
 *  停留在开位的火控开关立即恢复摩擦轮。 */
bool require_fire_release;
uint32_t trigger_start_ms;
uint32_t shoot_event_sequence;
constexpr uint32_t kLongPressMs = 300U;
/* Pitch 通道两级一阶低通（框架 Class_Filter_IIR_First_Order 级联），
 * 每级时间常数 25 ms，总延迟约 50 ms，与原手写实现一致。 */
Class_Filter_IIR_First_Order pitch_filter_stage1;
Class_Filter_IIR_First_Order pitch_filter_stage2;

/** 通道值线性映射到 DM-IMU Pitch 限位。 */
float MapPitchChannel(float channel)
{
    const float ratio = 1.0f - (channel - kPitchChannelMin) / kPitchChannelSpan;
    const auto config = Gimbal_Default_Config();
    return config.pitch_min + ratio * (config.pitch_max - config.pitch_min);
}

/** 波轮档位线性映射到拨弹盘输出速度（输出轴 rad/s），负档位为 0。 */
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
} // namespace

bool RemoteInput_Init(void)
{
    if (remote_input_initialized)
    {
        return true;
    }

    /* 板间链路走云台板的 FDCAN2，与底盘板的下行帧一致。 */
    if (!chassis_board.Init(BoardConfig_Get().remote_forward_bus))
    {
        return false;
    }
    fire_trigger_pressed = false;
    require_fire_release = true;
    trigger_start_ms = 0U;
    shoot_event_sequence = 0U;
    /* 两级低通：每级 tau = 25 ms（原工程数值），1 kHz 采样。 */
    constexpr float alpha = kControlPeriodS / (kPitchChannelFilterTauS + kControlPeriodS);
    const float cutoff_hz = -std::log1p(-alpha) / (6.283185307179586f * kControlPeriodS);
    pitch_filter_stage1.Init(cutoff_hz, 1000.0f);
    pitch_filter_stage2.Init(cutoff_hz, 1000.0f);
    /* 输入仲裁状态一并复位：上电即处于 Remote 失联安全态。 */
    InputState_Reset();
    InputState_SetTime(HAL_GetTick());
    remote_input_initialized = true;
    return true;
}

/**
 * @brief 读取板间通道并把遥控输入提交到 InputState。
 *
 * 链路健康：Pitch 通道经滤波映射后作为 GimbalMode::IMU 目标提交，火控开关与
 * 波轮档位作为 ShootCmd 意图提交；SourceArbitration 统一仲裁后由 RobotCmd
 * 发布。链路失效（100 ms 无新帧）：提交空输入 → 仲裁 disarm → 云台 DISABLED、
 * Shoot OFF，两个 Application 同周期停手。
 */
void RemoteInput_Update(void)
{
    if (!remote_input_initialized)
    {
        return;
    }

    /* 仲裁时钟只在 ControlTask 上下文推进。 */
    const uint32_t now_ms = HAL_GetTick();
    InputState_SetTime(now_ms);

    // 三个通道必须来自同一帧，不能在独立 getter 之间被 CAN ISR 更新。
    // ReadChannels 的 100 ms 判断是数据 freshness（本帧是否仍可用于控制），
    // 与 ChassisBoard Daemon 的链路在线结论相互独立，二者都保留。
    Struct_ChassisBoard_Channels channels{};
    const bool channels_valid = chassis_board.ReadChannels(channels);
    const int16_t fire = channels.fire;
    const int16_t dial = channels.dial;
    const int16_t pitch = channels.pitch;
    Remote_Pitch_Channel = pitch;
    Remote_Pitch_Valid = channels_valid;

    if (!channels_valid)
    {
        /* 安全互锁：提交空输入由仲裁输出 safe state；不再清除滤波历史，
         * 链路恢复后目标由限速率路径平滑过渡。 */
        fire_trigger_pressed = false;
        require_fire_release = true;
        trigger_start_ms = now_ms;
        InputState_SubmitRemote({});
        return;
    }

    const bool was_pressed = fire_trigger_pressed;
    if (fire >= kFireReleasedThreshold)
    {
        fire_trigger_pressed = false;
        require_fire_release = false;
    }
    else if (fire <= kFirePressedThreshold && !require_fire_release)
    {
        fire_trigger_pressed = true;
    }
    /* 中间区或未过恢复释放门：保持当前锁存值（恢复后必为未按）。 */
    const bool trigger_pressed = fire_trigger_pressed;

    if (trigger_pressed && !was_pressed)
    {
        trigger_start_ms = now_ms;
    }
    const bool short_release = was_pressed && !trigger_pressed &&
                               now_ms - trigger_start_ms < kLongPressMs;
    if (short_release)
    {
        ++shoot_event_sequence;
    }
    const bool burst = trigger_pressed && now_ms - trigger_start_ms >= kLongPressMs;
    ControlInput remote_input{};
    remote_input.shoot_event_sequence = shoot_event_sequence;

    GimbalCmd gimbal_command{};
    gimbal_command.mode = GimbalMode::IMU;
    /* 本板只拥有 Pitch：Yaw 字段不参与控制，也不初始化 Yaw 电机。 */
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
    remote_input.gimbal = gimbal_command;

    ShootCmd shoot_command{};
    // ON 表示健康输入授予输出许可；松扳机不撤销单发/延时停轮，失联才 OFF。
    shoot_command.shoot_mode = ShootMode::ON;
    /* 输入层识别长短按：短按为事件，长按为持续 BURST。 */
    shoot_command.friction_mode =
        trigger_pressed ? FrictionMode::ON : FrictionMode::OFF;
    shoot_command.loader_mode = burst ? LoaderMode::BURST : LoaderMode::STOP;
    /* 拨弹盘输出轴速度，rad/s。 */
    shoot_command.loader_speed_rad_s = MapDialToLoaderSpeed(dial);
    remote_input.shoot = shoot_command;

    remote_input.received_ms = now_ms;
    remote_input.valid = true;
    InputState_SubmitRemote(remote_input);
}

bool RemoteInput_IsLinkOnline(void)
{
    /* 直接返回 ChassisBoard 内部 Daemon 的 liveness 结果，不自行计算超时。 */
    return remote_input_initialized && chassis_board.IsOnline();
}

bool RemoteInput_GetRawChannels(int16_t *fire, int16_t *dial, int16_t *pitch)
{
    if (!remote_input_initialized)
    {
        return false;
    }

    Struct_ChassisBoard_Channels channels{};
    const bool valid = chassis_board.ReadChannels(channels);

    if (fire != nullptr)
    {
        *fire = channels.fire;
    }
    if (dial != nullptr)
    {
        *dial = channels.dial;
    }
    if (pitch != nullptr)
    {
        *pitch = channels.pitch;
    }
    return valid;
}

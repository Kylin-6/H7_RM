#ifndef SHOOT_CONFIG_H
#define SHOOT_CONFIG_H

#include "../physical_units.h"

#include <cstdint>

// 发射机构与控制参数；总线归属由 BoardConfig 提供。
#if !LEGACY_INFANTRY_GIMBAL
struct ShootPidConfig
{
    float kp;
    float ki;
    float kd;
    float integral_limit; // 积分项输出上限，与该环输出同单位。
    float output_limit; // 角度环为 rad/s；其余环按驱动级联契约配置。
};

struct ShootConfig
{
    float default_friction_speed_rad_s = 25.0f; // 摩擦轮开启且命令速度不为正时的备用输出轴速度。
    float default_rate_hz = 10.0f; // 连发未提供正射速时的备用弹数/s。
    float one_bullet_angle_rad = DegToRad(36.0f); // 拨弹盘输出轴每个弹位的机械角，用于事件和射速换算。
    float reverse_speed_rad_s = DegToRad(-360.0f); // 反转模式未提供非零速度时的备用角速度。
    float friction_gear_ratio = 1.0f; // 摩擦轮直驱，不使用 M3508 默认减速比 19。
    uint8_t friction_left_id = 3U; // 左摩擦轮 DJI 节点编号。
    uint8_t friction_right_id = 2U; // 右摩擦轮 DJI 节点编号，初始化中配置 reverse=true。
    uint8_t loader_id = 8U; // 拨弹电机 DJI 节点编号，必须与总线其他设备兼容。
    ShootPidConfig friction_speed_pid{7.5f, 5.0f, 0.0f, 16000.0f, 16000.0f};
    ShootPidConfig loader_current_pid{1.0f, 50.0f, 0.0f, 12000.0f, 12000.0f};
    ShootPidConfig loader_speed_pid{7.5f, 20.0f, 0.0f, 12000.0f, 12000.0f};
    ShootPidConfig loader_angle_pid{10.0f, 0.0f, 0.0f,
                                    0.0f, DegToRad(360.0f)};
};

constexpr ShootConfig kShootConfig{};

#endif

#if LEGACY_INFANTRY_GIMBAL
// 老步兵标定参数集中于应用配置；接线由 BoardConfig 提供。
namespace InfantryShootConfig
{
constexpr float SHOOT_PI = 3.14159265358979323846f;

/* 硬件 ID 与云台板原工程一致。 */
constexpr uint8_t FRICTION_LEFT_ID = 0x07U;
constexpr uint16_t FRICTION_LEFT_FEEDBACK_ID = 0x027U;
constexpr uint8_t FRICTION_RIGHT_ID = 0x08U;
constexpr uint16_t FRICTION_RIGHT_FEEDBACK_ID = 0x028U;
constexpr uint8_t LOADER_ID = 1U;

/* 反馈时效；协议补交由驱动负责。 */
constexpr uint32_t FEEDBACK_TIMEOUT_MS = 100U;

/* 机构参数。 */
constexpr float FRICTION_SPEED_RAD_S = 25.0f;
constexpr float FRICTION_READY_TOLERANCE_RAD_S = 1.0f;
constexpr float DM3519_VELOCITY_MAX_RAD_S = 200.0f;
constexpr float DM3519_TORQUE_MAX_NM = 10.0f;
constexpr float DM3519_POSITION_MAX_RAD = 12.5f;
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
// 连发退出后停稳再接收单发，按输出轴编码器速度判断（rad/s）。
constexpr float LOADER_STOP_SPEED_RAD_S = 0.2f;

/* 安全参数，集中在一起便于按实车标定。 */
constexpr int16_t JAM_CURRENT_THRESHOLD = 3800;
constexpr uint32_t JAM_CONFIRM_MS = 300U;
// 回退未到位的超时上限，不作为固定回退时长。
constexpr uint32_t JAM_HANDLE_MS = 200U;
constexpr float JAM_DONE_ANGLE_RAD = 2.0f * SHOOT_PI / 180.0f;
// 卡弹时从当前位置反向回退半个弹位，输出轴角度 rad。
constexpr float JAM_BACKOFF_RAD = 0.5f * ONE_BULLET_MOTOR_OUTPUT_RAD;
constexpr float HEAT_LEFT_TORQUE_THRESHOLD_NM = -0.6f;
constexpr float HEAT_RIGHT_TORQUE_THRESHOLD_NM = 0.5f;
constexpr uint32_t HEAT_CONFIRM_MS = 20U;
constexpr float HEAT_PER_SHOT = 10.0f;
constexpr float HEAT_COOL_PER_SECOND = 35.0f;
constexpr float HEAT_LIMIT = 220.0f;

} // namespace InfantryShootConfig
#endif

#endif

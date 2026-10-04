#ifndef SHOOT_H
#define SHOOT_H

/**
 * @brief ControlTask 启动时调用一次，注册三个电机及两个发送组。
 * @return 全部设备注册与发送组绑定成功返回 true。
 */
bool Shoot_Init(void);
/**
 * @brief 每 1 ms 在 RobotCmd_Update 之后调用，消费连续目标与离散事件。
 * @note 反馈每 10 ms 发布；应用未检查命令年龄，上层负责持续刷新安全目标。
 */
void Shoot_Update(void);

#if SHOOT && LEGACY_INFANTRY_GIMBAL
#include "message_types.h"
/** 仅供 ControlTask 采集。 */
Struct_Shoot_Diagnostic Shoot_GetDiagnostic();
struct Struct_Legacy_Loader_Debug
{
    float encoder;
    float rotor_total_angle_degree;
    float output_total_angle_degree;
    float rotor_speed_rad_s;
    float output_speed_rad_s;
    float current_raw;
    float feedback_age_ms;
    float speed_pid_out;
};

/**
 * @brief 导出老步兵云台板发射诊断量，供 USART1 JustFloat 遥测使用。
 * @note  与云台板原工程（H7_RM）同名同参，通道含义见 TransportTask.cpp。
 *        默认配置（DJI 摩擦轮）下不提供该接口。
 */
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
                               float *right_motor_state);

/** @brief 导出 M2006/C610 原始反馈与控制量，供 JustFloat 排查。 */
extern "C" void Shoot_GetLoaderDebug(Struct_Legacy_Loader_Debug *debug);
#endif

#endif

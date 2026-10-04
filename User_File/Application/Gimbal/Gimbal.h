#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "message_types.h"

#include "Gimbal_Config.h"

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE,
    Gimbal_Status_ENABLING,
    Gimbal_Status_READY,
    Gimbal_Status_FAULT,
    Gimbal_Status_CONFIG_ERROR,
};

/** ControlTask 的 1 kHz 入口；状态发布为 100 Hz。 */
void Gimbal_Update(void);

/** 仅启动阶段调用一次；复制配置并注册驱动，不等待应答、不置零、不切换电机模式。 */
bool Gimbal_Init(const Struct_Gimbal_Config &config = Gimbal_Default_Config());
/** 返回应用状态快照；仅供任务上下文读取，不允许外部改写状态或直接控制电机。 */
Enum_Gimbal_Status Gimbal_GetStatus(void);
/** EmberProbe 只读观测；ControlTask 唯一写入者。运行中跨字段读取非原子快照。 */
struct Struct_Gimbal_Debug
{
    uint32_t tick_ms = 0U;
    float command_rad = 0.0f;
    float target_rad = 0.0f;
    float target_speed_rad_s = 0.0f;
    float actual_rad = 0.0f;
    float actual_speed_rad_s = 0.0f;
    float command_torque_nm = 0.0f; // 电机协议方向；提交成功不等于执行成功。
    float feedback_torque_nm = 0.0f; // 电机协议方向。
    bool submitted = false;
    bool controlling = false;
    bool ins_valid = false;
    bool motor_ready = false;
};
extern Struct_Gimbal_Debug Gimbal_Debug;
/** 仅供同一 ControlTask 上下文的诊断层采集，禁止跨任务直接读取应用私有状态。 */
Struct_Gimbal_Diagnostic Gimbal_GetDiagnostic();
#endif

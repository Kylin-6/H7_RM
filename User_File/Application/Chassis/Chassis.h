#ifndef CHASSIS_H
#define CHASSIS_H

#include "dmmotor.h"
#include "message_types.h"

struct Struct_Chassis_Diagnostic_Input
{
    bool initialized = false;
    bool ins_valid = false;
    bool permitted = false;
    Struct_DMMotor_Snapshot motor[5]{}; ///< 四轮按配置顺序，最后一路为 Yaw。
};

/** 调参观察快照；ControlTask 写入，StatusTask 通过 Topic 读取。SI 单位。 */
struct Struct_Yaw_Tuning
{
    float target_rad = 0.0f;
    float actual_rad = 0.0f;
    float error_rad = 0.0f;
    float stick_rad_s = 0.0f;
    float pid_rad_s = 0.0f;
    float base_rad_s = 0.0f;
    float ff_rad_s = 0.0f; ///< 已换算到电机方向的有符号前馈。
    float command_rad_s = 0.0f;
    float motor_rad_s = 0.0f;
    float kd = 0.0f;
    float torque_nm = 0.0f;
    bool imu_valid = false;
    bool active = false;
};
bool Chassis_ReadYawTuning(Struct_Yaw_Tuning &sample);

/** 启动阶段仅调用一次；注册设备，失败返回 false，不执行机械寻零。 */
bool Chassis_Init(void);
/** 同一 ControlTask 的 1 kHz 入口；消费新鲜速度命令，提交输出，100 Hz 发布反馈。 */
void Chassis_Update(void);
/** 仅供同一 ControlTask 上下文的诊断层采集，不从其他任务读取应用 Context。 */
Struct_Chassis_Diagnostic_Input Chassis_GetDiagnostic(void);

/** 读取云台回传 INS；超过 100 ms 或无完整帧时返回 false，保持输出不变。 */
bool Chassis_GetGimbalImu(INS_State &state);

#endif

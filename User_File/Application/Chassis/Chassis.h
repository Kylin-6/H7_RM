#ifndef CHASSIS_H
#define CHASSIS_H

#include "dmmotor.h"

struct Struct_Chassis_Diagnostic_Input
{
    bool initialized = false;
    bool ins_valid = false;
    bool permitted = false;
    Struct_DMMotor_Snapshot motor[5]{}; ///< 四轮按配置顺序，最后一路为 Yaw。
};

/** 启动阶段仅调用一次；注册设备，失败返回 false，不执行机械寻零。 */
bool Chassis_Init(void);
/** 同一 ControlTask 的 1 kHz 入口；消费新鲜速度命令，提交输出，100 Hz 发布反馈。 */
void Chassis_Update(void);
/** 仅供同一 ControlTask 上下文的诊断层采集，不从其他任务读取应用 Context。 */
Struct_Chassis_Diagnostic_Input Chassis_GetDiagnostic(void);

#endif

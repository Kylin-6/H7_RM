#ifndef CHASSIS_H
#define CHASSIS_H

/** 初始化底盘电机及控制参数。 */
bool Chassis_Init(void);
/** 底盘应用的 1 kHz 周期入口。 */
void Chassis_Update(void);

#if LEGACY_INFANTRY_CHASSIS
#include "dmmotor.h"
struct Struct_Chassis_Diagnostic_Input
{
    bool initialized = false;
    bool ins_valid = false;
    bool permitted = false;
    Struct_DMMotor_Snapshot motor[5]{}; ///< 四轮按配置顺序，最后一路为 Yaw。
};
/** 仅供 ControlTask 在 Chassis_Update 后读取，不从其他任务读取应用 Context。 */
Struct_Chassis_Diagnostic_Input Chassis_GetDiagnostic(void);
#endif

#endif

#ifndef CHASSIS_H
#define CHASSIS_H

/** 初始化底盘电机及控制参数。 */
bool Chassis_Init(void);
/** 底盘应用的 1 kHz 周期入口。 */
void Chassis_Update(void);

#if LEGACY_INFANTRY_CHASSIS
/** ControlTask 初始化失败时发布故障，之后不覆盖。 */
void Chassis_DiagnosticInitFailure(void);
/** TIM_1ms_Task 的 10 ms 入口：唯一灯色写入者及 SPI 刷新入口。 */
void Chassis_LED_Update(void);
#endif

#endif

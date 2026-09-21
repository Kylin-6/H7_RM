#ifndef SHOOT_H
#define SHOOT_H

/** 初始化摩擦轮、拨弹盘电机及控制参数。 */
bool Shoot_Init(void);
/** 发射应用的 1 kHz 周期入口。 */
void Shoot_Update(void);

#if SHOOT && LEGACY_INFANTRY_GIMBAL
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
#endif

#endif

#ifndef SHOOT_H
#define SHOOT_H

/** 初始化摩擦轮、拨弹盘电机及控制参数。 */
bool Shoot_Init(void);
/** 发射应用的 1 kHz 周期入口。 */
void Shoot_Update(void);

#if SHOOT && LEGACY_INFANTRY_GIMBAL
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

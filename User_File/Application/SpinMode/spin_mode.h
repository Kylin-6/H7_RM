#ifndef SPIN_MODE_H
#define SPIN_MODE_H

#include "alg_fsm.h"

typedef enum
{
    Mecanum,
    Omnidirectional
} ChassisMode_e;

typedef enum
{
    SpinMode_GIMBAL_FOLLOW = 0,
    SpinMode_GIMBAL_LOCK,
    SpinMode_CHASSIS_FOLLOW
} SpinMode_e;

typedef struct
{
    float x;
    float y;
    float w;
    float forward;
    /** @brief rad；云台跟随时为校准后的相对角目标，其余模式为世界 Yaw 目标。 */
    float gimbal_yaw_target;
    /** @brief rad；仅底盘跟随时有效，正值表示底盘应逆时针转，其余模式为 0。 */
    float chassis_yaw_error;
} SpinOutput_t;

typedef struct
{
    float zero_point;
    SpinMode_e mode;
} SpinConfig_t;

/**
 * @author zzm
 * @brief 云台系运动指令到车体系速度及相对速度前馈的计算接口。
 * @note 先 Init 再计算；调用方在同一上下文更新有限输入并读取结果。
 *       X 向前、Y 向左、逆时针为正；角度 rad，角速度 rad/s，平移速度 m/s。
 *       theta 减去 zero_point 后表示云台朝向减底盘朝向；已归零时 zero_point 为 0。
 *       各模式的 w 均由调用方的 vw 提供。
 *       world_angle 作为锁定及底盘跟随模式的云台目标输出；保留输入的连续角度。
 *       云台跟随在进入模式后的首次计算锁存相对角目标；计算前应更新 theta。
 *       chassis_yaw_error 交给调用方生成 vw；本模块不含角度/速度控制器。
 *       forward 叠加到云台相对底盘的速度目标。
 */
class SpinMode
{
  public:
    /** @brief 配置非法时返回 false 并保留原状态；NULL 使用零偏移、云台跟随模式。 */
    bool Init(const SpinConfig_t *config = NULL);
    void Set_WorldTarget(float world_angle);
    void Set_theta(float theta);
    void Set_MoveTarget(float vx, float vy, float vw);
    void Set_Spin_Mode(SpinMode_e mode);

    void TIM_Calculate_PeriodElapsedCallback();

    SpinOutput_t Get_Output() const;
    SpinMode_e Get_Spin_Mode() const;
    float Get_WorldTarget() const;

  protected:
    Class_FSM<3> SpinMode_FSM;
    SpinOutput_t Output = {};
    float zero_point = 0.0f;
    float world_angle = 0.0f;
    float theta = 0.0f;
    float x_target = 0.0f;
    float y_target = 0.0f;
    float w_target = 0.0f;
    float relative_target = 0.0f;
    bool capture_relative_target = true;
};

#endif

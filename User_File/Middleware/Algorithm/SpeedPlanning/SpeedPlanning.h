/**
 * @file SpeedPlanning.h
 * @brief 带死区/指数整形与非对称加减速限制的速度规划组件。
 * @details
 * 由 rm/demo（老步兵工程）的 User/algorithm/SpeedPlanning 移植而来，接口与算法语义
 * 保持原样，只补充 C++ 互操作声明。全部为无状态或有状态纯计算，不访问外设。
 *
 * 单位约定：除 dt 为秒、normalized_time 为无量纲外，speed 与各 limit 一律使用调用方
 * 自己的速度量纲；limit 的单位是“速度单位/秒”。
 */

#ifndef SPEED_PLANNING_H
#define SPEED_PLANNING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    float current_speed;
    float target_speed_last;
    float time_since_change;
    uint8_t transition_active;
    uint8_t transition_accelerating;
} SpeedPlanningState;

void SpeedPlanning_Init(SpeedPlanningState* state, float initial_speed);
float SpeedPlanning_SCurve(float normalized_time);
float SpeedPlanning_ApplyDeadbandExpo(float input, float input_max, float deadband_ratio, float expo);
float SpeedPlanning_Update(float target_speed, SpeedPlanningState* state, float dt, float transition_time, float target_threshold);
float SpeedPlanning_UpdateAsymmetric(float target_speed, SpeedPlanningState* state, float dt, float acceleration_time, float deceleration_time, float target_threshold);
float SpeedPlanning_UpdateRateLimited(float target_speed, SpeedPlanningState* state, float dt,
                                      float acceleration_limit, float deceleration_limit,
                                      float release_deceleration_limit, float reversal_deceleration_limit,
                                      float zero_threshold);

#ifdef __cplusplus
}
#endif

#endif
